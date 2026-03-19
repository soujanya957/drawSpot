"""
draw_gcode.robot.navigator
===========================
Autonomous navigation and setup for the draw_gcode pipeline.

Approach geometry
-----------------
The robot always approaches from the x≈0 side of the canvas (the TL/BL
edge), so it never crosses the canvas surface.

  Standoff point  = TL_corner − x_axis × CANVAS_STANDOFF_MM
                    (CANVAS_STANDOFF_MM mm to the left of the TL corner,
                     along the canvas −X direction)

  Desired heading = +canvas_x_axis direction (robot faces toward the canvas)

Two-phase nav to guarantee the canvas is never stepped on:
  Phase 1 — if the robot's current position projects past the TL edge (onto
             the canvas side), first move it into the approach lane
             (standoff X, current Y) before turning toward the standoff.
  Phase 2 — walk straight to the standoff point.
  Phase 3 — rotate in place to face +canvas_x_axis.

Public API
----------
  walk_to_canvas(cmd_client, vicon_client) -> bool
  open_for_marker(cmd_client)              -> bool
  move_to_draw_pose(cmd_client, vicon_client) -> bool

All functions return False if emergency stop (SPACE) fires.
"""

import math
import time

import numpy as np
from google.protobuf import duration_pb2

from bosdyn.api import robot_command_pb2
from bosdyn.api.geometry_pb2 import Quaternion
from bosdyn.client.robot_command import RobotCommandBuilder

import draw_gcode._ctrl as _ctrl

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

CANVAS_STANDOFF_MM  = 400   # distance from the TL corner where the robot stops (mm)
CANVAS_SAFE_MARGIN  = 200   # extra buffer: if robot is within this of TL edge, reroute (mm)
BRUSH_LENGTH_MM     = 150.0 # marker length below gripper tip (mm)
PEN_UP_MM           = 60.0  # clearance above canvas surface for travel (mm)
NAV_SPEED           = 0.35  # m/s
BASE_CMD_DUR        = 0.6   # velocity command duration (s)
YAW_ALIGN_TOL       = 0.05  # rad (~3°) — acceptable alignment error

_DOWN_QUAT    = Quaternion(w=0.7071068, x=0.0, y=0.7071068, z=0.0)
_HANDOFF_QUAT = Quaternion(w=0.9239,   x=0.0, y=0.3827,   z=0.0)   # ~45° tilt, easy handoff


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _dur(s: float):
    return duration_pb2.Duration(seconds=int(s), nanos=int((s - int(s)) * 1e9))


def _rotation_matrix(q) -> np.ndarray:
    qx, qy, qz, qw = q
    return np.array([
        [1 - 2*(qy**2 + qz**2),   2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw)],
        [  2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),   2*(qy*qz - qx*qw)],
        [  2*(qx*qz - qy*qw),   2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
    ], dtype=float)


def _body_yaw(body) -> float:
    qx, qy, qz, qw = body.rotation_quat
    return math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))


def _wrap(a: float) -> float:
    while a > math.pi:  a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a


# ---------------------------------------------------------------------------
# Arm helpers
# ---------------------------------------------------------------------------

def _arm_odom(cmd_client, x, y, z, quat, dur_s=2.0):
    cmd = robot_command_pb2.RobotCommand()
    ac  = cmd.synchronized_command.arm_command.arm_cartesian_command
    ac.root_frame_name = "odom"
    pt  = ac.pose_trajectory_in_task.points.add()
    pt.pose.position.x = x;  pt.pose.position.y = y;  pt.pose.position.z = z
    pt.pose.rotation.CopyFrom(quat)
    pt.time_since_reference.CopyFrom(_dur(dur_s))
    cmd_client.robot_command(cmd)


def _arm_body(cmd_client, x, y, z, quat, dur_s=2.0):
    cmd = robot_command_pb2.RobotCommand()
    ac  = cmd.synchronized_command.arm_command.arm_cartesian_command
    ac.root_frame_name = "body"
    pt  = ac.pose_trajectory_in_task.points.add()
    pt.pose.position.x = x;  pt.pose.position.y = y;  pt.pose.position.z = z
    pt.pose.rotation.CopyFrom(quat)
    pt.time_since_reference.CopyFrom(_dur(dur_s))
    cmd_client.robot_command(cmd)


# ---------------------------------------------------------------------------
# Navigation primitives
# ---------------------------------------------------------------------------

_YAW_DRIVE_THRESHOLD = 0.4   # rad (~23°) — rotate-only below this before driving

def _nav_step(cmd_client, body, target_mm: np.ndarray, stop_mm: float) -> bool:
    """One tick toward target_mm. Returns True when within stop_mm.

    Rotate-first strategy: if yaw error exceeds _YAW_DRIVE_THRESHOLD, spin in
    place until roughly aligned, then drive forward with minor yaw correction.
    This prevents the robot from spinning endlessly when the target is to its side.
    """
    pos   = body.position[:2]
    delta = target_mm[:2] - pos
    dist  = float(np.linalg.norm(delta))
    if dist < stop_mm:
        return True

    R        = _rotation_matrix(body.rotation_quat)[:2, :2]
    dir_body = R.T @ (delta / dist)
    yaw_err  = math.atan2(float(dir_body[1]), float(dir_body[0]))

    if abs(yaw_err) > _YAW_DRIVE_THRESHOLD:
        # Rotate in place toward target — no forward motion
        v_rot = float(np.clip(yaw_err * 2.0, -0.5, 0.5))
        cmd_client.robot_command(
            RobotCommandBuilder.synchro_velocity_command(v_x=0, v_y=0, v_rot=v_rot),
            end_time_secs=time.time() + BASE_CMD_DUR,
        )
    else:
        # Roughly aligned — drive forward with small yaw correction
        v_rot = float(np.clip(yaw_err * 1.5, -0.3, 0.3))
        cmd_client.robot_command(
            RobotCommandBuilder.synchro_velocity_command(v_x=NAV_SPEED, v_y=0, v_rot=v_rot),
            end_time_secs=time.time() + BASE_CMD_DUR,
        )
    return False


def _walk_to_point(cmd_client, vicon_client,
                   target_mm: np.ndarray, stop_mm: float, label: str) -> bool:
    """
    Walk to target_mm (world XY, in mm) until within stop_mm.
    Prints progress once per second. Returns False if estop fires.

    Uses Vicon's predicted position when the body is occluded (some markers
    temporarily blocked) — Vicon still provides a valid pose in that case.
    Only blocks if spot_body is None (subject completely lost).
    """
    print(f"  Walking to {label}…")
    last_print = 0.0
    _no_body_warned = False
    while True:
        if not _ctrl.check():
            return False

        frame = vicon_client.latest_frame
        if frame is None or frame.spot_body is None:
            if not _no_body_warned:
                print("    [WAIT] spot_base not visible in Vicon — waiting for tracking…")
                _no_body_warned = True
            time.sleep(0.05)
            continue
        _no_body_warned = False

        body = frame.spot_body
        dist = float(np.linalg.norm(body.position[:2] - target_mm[:2]))

        now = time.time()
        if now - last_print >= 1.0:
            bx, by = body.position[:2]
            tx, ty = target_mm[:2]
            occ_tag = " [occluded]" if body.occluded else ""
            print(f"    Spot ({bx:+7.0f}, {by:+7.0f}) mm{occ_tag}"
                  f"  →  {label} ({tx:+7.0f}, {ty:+7.0f}) mm"
                  f"  dist {dist:.0f} mm")
            last_print = now

        if _nav_step(cmd_client, body, target_mm, stop_mm):
            cmd_client.robot_command(
                RobotCommandBuilder.synchro_velocity_command(0, 0, 0),
                end_time_secs=time.time() + 0.3,
            )
            bx, by = body.position[:2]
            print(f"  Reached {label}  (Spot at ({bx:+.0f}, {by:+.0f}) mm,"
                  f"  dist {dist:.0f} mm)")
            return True

        time.sleep(0.05)


def _align_to_yaw(cmd_client, vicon_client, target_yaw: float) -> bool:
    """Rotate in place to face target_yaw (rad, world frame). Returns False if estop."""
    print(f"  Aligning heading to {math.degrees(target_yaw):.1f}°…")
    last_print = 0.0

    for _ in range(200):
        if not _ctrl.check():
            return False

        frame = vicon_client.latest_frame
        if frame is None or frame.spot_body is None or frame.spot_body.occluded:
            time.sleep(0.1)
            continue

        yaw_err = _wrap(target_yaw - _body_yaw(frame.spot_body))

        now = time.time()
        if now - last_print >= 1.0:
            print(f"    yaw error {math.degrees(yaw_err):+.1f}°")
            last_print = now

        if abs(yaw_err) < YAW_ALIGN_TOL:
            cmd_client.robot_command(
                RobotCommandBuilder.synchro_velocity_command(0, 0, 0),
                end_time_secs=time.time() + 0.3,
            )
            print(f"  Heading aligned  (error {math.degrees(yaw_err):.1f}°)")
            return True

        v_rot = float(np.clip(yaw_err * 2.5, -0.5, 0.5))
        cmd_client.robot_command(
            RobotCommandBuilder.synchro_velocity_command(0, 0, v_rot),
            end_time_secs=time.time() + 0.3,
        )
        time.sleep(0.1)

    print("  [WARN] Yaw alignment timed out — continuing.")
    return True


# ---------------------------------------------------------------------------
# walk_to_canvas
# ---------------------------------------------------------------------------

def walk_to_canvas(cmd_client, vicon_client) -> bool:
    """
    Walk to a standoff position to the LEFT of the canvas (x≈0 side),
    then rotate to face in the +canvas_x_axis direction (+X world).

    The approach is always from outside the canvas boundary:
      standoff = TL_corner − canvas_x_axis × CANVAS_STANDOFF_MM

    If the robot is currently on the canvas side (projected x > TL edge − margin),
    it first moves to the approach lane (standoff_x, current_y) to clear the
    canvas before heading to the final standoff position.
    """
    # ── Get canvas geometry ───────────────────────────────────────────────
    print("  Waiting for canvas frame from Vicon…")
    deadline = time.time() + 8.0
    frame = None
    while time.time() < deadline:
        if not _ctrl.check():
            return False
        frame = vicon_client.latest_frame
        if frame and frame.canvas and frame.canvas.is_valid():
            break
        time.sleep(0.1)

    if frame is None or frame.canvas is None or not frame.canvas.is_valid():
        print("  [FAIL] Canvas not visible in Vicon.")
        return False

    c = frame.canvas

    # Canvas x_axis = TL → TR (points in world +X for our setup)
    x_axis_2d = c.x_axis[:2]
    x_axis_2d = x_axis_2d / np.linalg.norm(x_axis_2d)

    # Near-edge midpoint: midpoint of TL (corners[0]) and BL (corners[3])
    # This centers Spot on the near edge so the arm covers the full canvas width
    tl_2d = c.corners[0][:2]
    bl_2d = c.corners[3][:2]
    near_mid_2d = (tl_2d + bl_2d) / 2.0

    # Standoff: CANVAS_STANDOFF_MM outside the near-edge midpoint, along −x_axis
    standoff_xy = near_mid_2d - x_axis_2d * CANVAS_STANDOFF_MM
    standoff_mm = np.array([standoff_xy[0], standoff_xy[1], 0.0])

    # Heading: face in +canvas_x_axis direction (toward the canvas interior)
    desired_yaw = math.atan2(float(x_axis_2d[1]), float(x_axis_2d[0]))

    print(f"  Canvas TL       ({tl_2d[0]:+.0f}, {tl_2d[1]:+.0f}) mm")
    print(f"  Canvas BL       ({bl_2d[0]:+.0f}, {bl_2d[1]:+.0f}) mm")
    print(f"  Near-edge mid   ({near_mid_2d[0]:+.0f}, {near_mid_2d[1]:+.0f}) mm")
    print(f"  Standoff target ({standoff_xy[0]:+.0f}, {standoff_xy[1]:+.0f}) mm"
          f"  —  {CANVAS_STANDOFF_MM:.0f} mm from near edge along −x_axis")
    print(f"  Target heading  {math.degrees(desired_yaw):.1f}°  (+canvas x_axis)")

    # ── Check if robot needs a detour to avoid crossing the canvas ────────
    cur_frame = vicon_client.latest_frame
    if cur_frame and cur_frame.spot_body and not cur_frame.spot_body.occluded:
        robot_pos = cur_frame.spot_body.position[:2]
        # Project robot onto canvas x_axis relative to TL corner.
        # Positive = robot is on the TR/canvas side of TL (needs detour).
        robot_x_proj = float(np.dot(robot_pos - tl_2d, x_axis_2d))

        if robot_x_proj > -(CANVAS_SAFE_MARGIN):
            # Robot is too close to or past the TL edge — route via approach lane
            # (same x as standoff, same y as robot) to stay clear of canvas.
            lane_mm = np.array([standoff_xy[0], robot_pos[1], 0.0])
            print()
            print(f"  Robot is on canvas side (x_proj = {robot_x_proj:.0f} mm from TL edge).")
            print(f"  Detouring to approach lane first to avoid stepping on canvas.")
            if not _walk_to_point(cmd_client, vicon_client, lane_mm, 100.0,
                                   "approach lane"):
                return False
            if not _ctrl.check():
                return False

    # ── Phase 2: walk to standoff ─────────────────────────────────────────
    print()
    if not _walk_to_point(cmd_client, vicon_client, standoff_mm, 80.0, "canvas standoff"):
        return False
    if not _ctrl.check():
        return False

    # ── Phase 3: align heading ────────────────────────────────────────────
    print()
    return _align_to_yaw(cmd_client, vicon_client, desired_yaw)


# ---------------------------------------------------------------------------
# open_for_marker
# ---------------------------------------------------------------------------

def open_for_marker(cmd_client) -> bool:
    """
    Extend arm to a comfortable forward handoff pose (angled, NOT top-down),
    open gripper, wait for operator to place the marker and press ENTER to
    close the gripper, then wait for a second ENTER to confirm the grip looks
    good before proceeding.

    Key bindings during this step:
      ENTER (1st) — close gripper around marker
      ENTER (2nd) — confirm marker is secure, ready to proceed
      SPACE       — emergency stop at any point
    """
    print("  Moving arm to marker handoff pose…")
    # Angled forward pose — easy for a person to insert a marker
    _arm_body(cmd_client, x=0.65, y=0.0, z=0.05, quat=_HANDOFF_QUAT, dur_s=2.5)
    if not _ctrl.sleep(3.0):
        return False

    print("  Opening gripper…")
    cmd_client.robot_command(RobotCommandBuilder.claw_gripper_open_command())
    if not _ctrl.sleep(1.0):
        return False

    if not _ctrl.wait_confirm("Place the marker / pen in the open gripper, then press ENTER to close."):
        return False

    print("  Closing gripper…")
    cmd_client.robot_command(RobotCommandBuilder.claw_gripper_close_command())
    if not _ctrl.sleep(1.5):
        return False

    if not _ctrl.wait_confirm("Marker secured. Check grip looks good, then press ENTER to proceed."):
        return False

    print("  Marker acquired.")
    return True


# ---------------------------------------------------------------------------
# move_to_draw_pose
# ---------------------------------------------------------------------------

def move_to_draw_pose(cmd_client, vicon_client) -> bool:
    """Move arm above canvas centre, marker pointing straight down."""
    print("  Moving to draw pose…")
    frame = vicon_client.latest_frame
    if frame is None or frame.canvas is None or not frame.canvas.is_valid():
        print("  [FAIL] Canvas not visible.")
        return False

    c      = frame.canvas
    center = np.mean(np.array(c.corners), axis=0)
    pos_mm = center + (BRUSH_LENGTH_MM + PEN_UP_MM) * c.normal
    x, y, z = pos_mm / 1000.0

    _arm_odom(cmd_client, x, y, z, _DOWN_QUAT, dur_s=3.0)
    if not _ctrl.sleep(3.5):
        return False

    print(f"  Draw pose reached  ({x:.3f}, {y:.3f}, {z:.3f}) m")
    return True
