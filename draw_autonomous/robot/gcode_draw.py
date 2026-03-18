"""
draw_autonomous.robot.gcode_draw
=================================
Gcode parsing (G00/G01/G02/G03) and force-compliant arm execution via
ArmSurfaceContact.  The canvas TL corner and orientation axes from Vicon
replace the touch-to-find-ground step in the original gcode.py.

Public API
----------
  build_canvas_origin(frame)            -> SE3Pose   (odom frame)
  draw_gcode(vicon_client, asc_client, cmd_client, state_client,
             gcode_path, scale, tool_length)          -> None
  draw_gcode_vicon(vicon_client, asc_client, cmd_client, state_client,
                   gcode_path, tool_length, margin)   -> None
  draw_gcode_manual(asc_client, cmd_client, state_client,
                    gcode_path, scale, tool_length, origin_x/y/z, ...) -> None

GcodeReader, move_arm, and get_transforms are inlined — no spot-sdk path dependency.
"""

import math
import time

import numpy as np
from google.protobuf import duration_pb2, wrappers_pb2

from bosdyn.api import (arm_surface_contact_pb2, arm_surface_contact_service_pb2,
                        geometry_pb2, trajectory_pb2)
from bosdyn.client.frame_helpers import (
    ODOM_FRAME_NAME, VISION_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME,
    BODY_FRAME_NAME, HAND_FRAME_NAME, WR1_FRAME_NAME, get_a_tform_b,
)
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.robot_command import RobotCommandBuilder, block_until_arm_arrives

import draw_autonomous._ctrl as _ctrl

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

VELOCITY            = 0.25    # arm speed m/s
PRESS_FORCE_PERCENT = -20.0   # negative = press down
BELOW_Z_ADMITTANCE  = 0.0     # gcode z below this (m) → force-controlled
TRAVEL_Z            = 0.03    # gcode z for travel moves (m)
MIN_DIST_TO_GOAL    = 0.02    # m — how close arm must be before advancing

# ---------------------------------------------------------------------------
# Gcode helpers
# ---------------------------------------------------------------------------

def _make_orthogonal(primary, secondary):
    p = primary / np.linalg.norm(primary)
    s = secondary / np.linalg.norm(secondary)
    u = s - np.dot(p, s) / np.dot(s, s) * p
    return u / np.linalg.norm(u)


class GcodeReader:
    def __init__(self, file_path, tool_length, scale, logger,
                 below_z_is_admittance, travel_z, draw_on_wall,
                 gcode_start_x=0, gcode_start_y=0):
        self.file = open(file_path, 'r')
        self.tool_length = tool_length
        self.scale = scale
        self.logger = logger
        self.below_z_is_admittance = below_z_is_admittance
        self.travel_z = travel_z
        self.draw_on_wall = draw_on_wall
        self.gcode_start_x = gcode_start_x
        self.gcode_start_y = gcode_start_y
        self.current_origin_T_goals = None
        self.last_x = self.last_y = self.last_z = 0

    def set_origin(self, vision_T_origin, vision_T_admittance_frame):
        if not self.draw_on_wall:
            zhat = [0.0, 0.0, 1.0]
            x1, x2, x3 = vision_T_origin.rot.transform_point(-1.0, 0.0, 0.0)
            xhat = _make_orthogonal(zhat, [x1, x2, x3])
            yhat = np.cross(zhat, xhat)
            mat = np.array([xhat, yhat, zhat]).T
            self.vision_T_origin = SE3Pose(vision_T_origin.x, vision_T_origin.y,
                                           vision_T_origin.z, Quat.from_matrix(mat))
        else:
            x1, x2, x3 = vision_T_admittance_frame.rot.transform_point(0, -1, 0)
            y1, y2, y3 = vision_T_admittance_frame.rot.transform_point(1, 0, 0)
            z1, z2, z3 = vision_T_admittance_frame.rot.transform_point(0, 0, 1)
            mat = np.array([[x1, y1, z1], [x2, y2, z2], [x3, y3, z3]]).T
            self.vision_T_origin = SE3Pose(vision_T_origin.x, vision_T_origin.y,
                                           vision_T_origin.z, Quat.from_matrix(mat))

    def _origin_Q_goal(self):
        mat = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]], dtype=float).T
        return Quat.from_matrix(mat)

    def _convert_line(self, line):
        for ch in ('(', '%', ';'):
            idx = line.find(ch)
            if idx >= 0:
                line = line[:idx]
        parts = line.split()
        if not parts:
            return None
        cmd = parts[0]
        if cmd in ('G00', 'G0', 'G01', 'G1'):
            x, y, z = self.last_x, self.last_y, self.last_z
            for p in parts[1:]:
                if p[0] == 'X':
                    x = (float(p[1:]) - self.gcode_start_x) * self.scale
                elif p[0] == 'Y':
                    y = (float(p[1:]) - self.gcode_start_y) * self.scale
                elif p[0] == 'Z':
                    z = float(p[1:]) * self.scale
            self.last_x, self.last_y, self.last_z = x, y, z
            return [SE3Pose(x, y, z, self._origin_Q_goal())]
        elif cmd in ('G02', 'G2', 'G03', 'G3'):
            x, y, z = self.last_x, self.last_y, self.last_z
            i_val = j_val = k_val = 0.0
            for p in parts[1:]:
                if p[0] == 'X':
                    x = (float(p[1:]) - self.gcode_start_x) * self.scale
                elif p[0] == 'Y':
                    y = (float(p[1:]) - self.gcode_start_y) * self.scale
                elif p[0] == 'Z':
                    z = float(p[1:]) * self.scale
                elif p[0] == 'I':
                    i_val = float(p[1:]) * self.scale
                elif p[0] == 'J':
                    j_val = float(p[1:]) * self.scale
                elif p[0] == 'K':
                    k_val = float(p[1:]) * self.scale
            clockwise = cmd in ('G02', 'G2')
            if i_val != 0 or j_val != 0:
                lp = [self.last_x, self.last_y]; ep = [x, y]
                cp = [self.last_x + i_val, self.last_y + j_val]
                def make_pt(px, py): return SE3Pose(px, py, self.last_z, self._origin_Q_goal())
            elif i_val != 0 or k_val != 0:
                lp = [self.last_x, self.last_z]; ep = [x, z]
                cp = [self.last_x + i_val, self.last_z + k_val]
                def make_pt(px, py): return SE3Pose(px, self.last_y, py, self._origin_Q_goal())
            else:
                lp = [self.last_y, self.last_z]; ep = [y, z]
                cp = [self.last_y + j_val, self.last_z + k_val]
                def make_pt(px, py): return SE3Pose(self.last_x, px, py, self._origin_Q_goal())
            lr = np.array(lp) - np.array(cp)
            er = np.array(ep) - np.array(cp)
            lt = math.atan2(lr[1], lr[0]); et = math.atan2(er[1], er[0])
            if clockwise:
                if lt < et: lt += 2 * math.pi
            else:
                if lt > et: et += 2 * math.pi
            n = max(int(abs(et - lt) / 0.01), 1)
            r = math.sqrt(lr[0]**2 + lr[1]**2)
            goals = []
            for i in range(n - 1):
                th = lt - i * 0.01 if clockwise else lt + i * 0.01
                goals.append(make_pt(cp[0] + r * math.cos(th), cp[1] + r * math.sin(th)))
            goals.append(make_pt(ep[0], ep[1]))
            self.last_x, self.last_y, self.last_z = goals[-1].x, goals[-1].y, goals[-1].z
            return goals
        else:
            return None

    def _get_vision_T_goal(self, origin_T_goal, ground_plane_rt_vision):
        vision_T_goal = self.vision_T_origin * origin_T_goal
        if not self.is_admittance():
            vision_T_goal.z = self.travel_z + ground_plane_rt_vision[2]
        else:
            vision_T_goal.z = ground_plane_rt_vision[2]
        return vision_T_goal

    def is_admittance(self):
        return self.current_origin_T_goals[0].z < self.below_z_is_admittance

    def get_next_vision_T_goals(self, ground_plane_rt_vision, read_new_line=True):
        origin_T_goals = None
        while not origin_T_goals:
            if read_new_line:
                self.last_line = self.file.readline()
            if not self.last_line:
                return (False, None, False)
            if self.last_line.strip() == 'M0':
                return (False, None, True)
            origin_T_goals = self._convert_line(self.last_line)
        self.current_origin_T_goals = origin_T_goals
        vision_T_goals = [self._get_vision_T_goal(g, ground_plane_rt_vision)
                          for g in origin_T_goals]
        return (self.is_admittance(), vision_T_goals, False)


# ---------------------------------------------------------------------------
# Robot state helpers
# ---------------------------------------------------------------------------

def get_transforms(use_vision_frame, robot_state):
    world = VISION_FRAME_NAME if use_vision_frame else ODOM_FRAME_NAME
    snap  = robot_state.kinematic_state.transforms_snapshot
    world_T_body      = get_a_tform_b(snap, world,           BODY_FRAME_NAME)
    world_T_flat_body = get_a_tform_b(snap, world,           GRAV_ALIGNED_BODY_FRAME_NAME)
    body_T_hand       = get_a_tform_b(snap, BODY_FRAME_NAME, HAND_FRAME_NAME)
    body_T_wrist      = get_a_tform_b(snap, BODY_FRAME_NAME, WR1_FRAME_NAME)
    world_T_hand      = world_T_body * body_T_hand
    odom_T_body       = get_a_tform_b(snap, ODOM_FRAME_NAME, BODY_FRAME_NAME)
    return (world_T_body, world_T_flat_body, body_T_hand, world_T_hand, body_T_wrist, odom_T_body)


def move_arm(robot_state, is_admittance, vision_T_goals, asc_client, velocity,
             allow_walking, vision_T_admittance, press_force_percentage,
             api_send_frame, use_xy_to_z_cross_term, bias_force_x):
    task_T_vision = SE3Pose.from_proto(vision_T_admittance).inverse()
    points = []
    last = None
    t = 0.0
    for goal in vision_T_goals:
        if last is None:
            seconds, nanos = 0, 0
        else:
            dt = math.sqrt((last.x - goal.x)**2 + (last.y - goal.y)**2 +
                           (last.z - goal.z)**2) / velocity
            t += dt
            seconds, nanos = int(t), int((t - int(t)) * 1e9)
        tg = task_T_vision * goal
        points.append(trajectory_pb2.SE3TrajectoryPoint(
            pose=geometry_pb2.SE3Pose(
                position=geometry_pb2.Vec3(x=tg.x, y=tg.y, z=tg.z),
                rotation=geometry_pb2.Quaternion(
                    w=tg.rot.w, x=tg.rot.x, y=tg.rot.y, z=tg.rot.z),
            ),
            time_since_reference=duration_pb2.Duration(seconds=seconds, nanos=nanos),
        ))
        last = goal

    traj  = trajectory_pb2.SE3Trajectory(points=points)
    press = geometry_pb2.Vec3(x=0, y=0, z=press_force_percentage)
    cmd   = arm_surface_contact_pb2.ArmSurfaceContact.Request(
        pose_trajectory_in_task=traj,
        root_frame_name=api_send_frame,
        root_tform_task=vision_T_admittance,
        press_force_percentage=press,
        x_axis=arm_surface_contact_pb2.ArmSurfaceContact.Request.AXIS_MODE_POSITION,
        y_axis=arm_surface_contact_pb2.ArmSurfaceContact.Request.AXIS_MODE_POSITION,
        z_axis=arm_surface_contact_pb2.ArmSurfaceContact.Request.AXIS_MODE_POSITION,
        max_linear_velocity=wrappers_pb2.DoubleValue(value=velocity),
    )
    if is_admittance:
        cmd.z_axis       = arm_surface_contact_pb2.ArmSurfaceContact.Request.AXIS_MODE_FORCE
        cmd.press_force_percentage.z = press_force_percentage
        cmd.xy_admittance = arm_surface_contact_pb2.ArmSurfaceContact.Request.ADMITTANCE_SETTING_OFF
        cmd.z_admittance  = arm_surface_contact_pb2.ArmSurfaceContact.Request.ADMITTANCE_SETTING_LOOSE
        cmd.xy_to_z_cross_term_admittance = (
            arm_surface_contact_pb2.ArmSurfaceContact.Request.ADMITTANCE_SETTING_VERY_STIFF
            if use_xy_to_z_cross_term else
            arm_surface_contact_pb2.ArmSurfaceContact.Request.ADMITTANCE_SETTING_OFF
        )
        cmd.bias_force_ewrt_body.CopyFrom(geometry_pb2.Vec3(x=bias_force_x, y=0, z=0))
    else:
        cmd.bias_force_ewrt_body.CopyFrom(geometry_pb2.Vec3(x=0, y=0, z=0))

    gripper = RobotCommandBuilder.claw_gripper_open_fraction_command(0)
    cmd.gripper_command.CopyFrom(
        gripper.synchronized_command.gripper_command.claw_gripper_command)
    cmd.is_robot_following_hand = allow_walking
    asc_client.arm_surface_contact_command(
        arm_surface_contact_service_pb2.ArmSurfaceContactCommand(request=cmd))


# ---------------------------------------------------------------------------
# Gcode bounds pre-flight scan
# ---------------------------------------------------------------------------

def scan_gcode_bounds(file_path: str, scale: float):
    """
    Scan a gcode file and return the X/Y extents (in metres) without executing.
    Returns (x_min, x_max, y_min, y_max) in metres.
    Accounts for G02/G03 arc centres (I/J offsets) as conservative extra points.
    """
    x_vals, y_vals = [0.0], [0.0]
    cur_x = cur_y = 0.0
    with open(file_path) as f:
        for line in f:
            for ch in ('(', '%', ';'):
                idx = line.find(ch)
                if idx >= 0:
                    line = line[:idx]
            parts = line.split()
            if not parts:
                continue
            cmd = parts[0]
            if cmd not in ('G00','G0','G01','G1','G02','G2','G03','G3'):
                continue
            x, y, i_val, j_val = cur_x, cur_y, 0.0, 0.0
            for p in parts[1:]:
                if not p:
                    continue
                if p[0] == 'X':
                    x = float(p[1:]) * scale
                elif p[0] == 'Y':
                    y = float(p[1:]) * scale
                elif p[0] == 'I':
                    i_val = float(p[1:]) * scale
                elif p[0] == 'J':
                    j_val = float(p[1:]) * scale
            x_vals.append(x)
            y_vals.append(y)
            if cmd in ('G02','G2','G03','G3') and (i_val or j_val):
                # arc centre as a conservative bound point
                x_vals.append(cur_x + i_val)
                y_vals.append(cur_y + j_val)
            cur_x, cur_y = x, y
    return min(x_vals), max(x_vals), min(y_vals), max(y_vals)


def print_bounds_check(file_path: str, scale: float, canvas) -> bool:
    """
    Print gcode extents vs canvas size.
    Returns True if gcode fits within canvas, False if it overflows (still continues).
    """
    x_min, x_max, y_min, y_max = scan_gcode_bounds(file_path, scale)
    # convert metres → mm for display
    gw = (x_max - x_min) * 1000
    gh = (y_max - y_min) * 1000
    gx0, gy0 = x_min * 1000, y_min * 1000

    print(f"  Gcode extents  X: {gx0:.1f} … {x_max*1000:.1f} mm  ({gw:.1f} mm wide)")
    print(f"                 Y: {gy0:.1f} … {y_max*1000:.1f} mm  ({gh:.1f} mm tall)")
    print(f"  Canvas size    {canvas.width_mm:.1f} mm × {canvas.height_mm:.1f} mm")

    fits = True
    if x_min < 0 or y_min < 0:
        print(f"  \033[93m[WARN] Gcode has negative coordinates"
              f" (X_min={gx0:.1f} mm, Y_min={gy0:.1f} mm)"
              f" — will draw outside TL corner\033[0m")
        fits = False
    if x_max * 1000 > canvas.width_mm:
        print(f"  \033[93m[WARN] Gcode X max ({x_max*1000:.1f} mm)"
              f" exceeds canvas width ({canvas.width_mm:.1f} mm)"
              f" — overflow by {x_max*1000 - canvas.width_mm:.1f} mm\033[0m")
        fits = False
    if y_max * 1000 > canvas.height_mm:
        print(f"  \033[93m[WARN] Gcode Y max ({y_max*1000:.1f} mm)"
              f" exceeds canvas height ({canvas.height_mm:.1f} mm)"
              f" — overflow by {y_max*1000 - canvas.height_mm:.1f} mm\033[0m")
        fits = False
    if fits:
        print(f"  \033[92m[OK]   Gcode fits within canvas.\033[0m")
    return fits


# ---------------------------------------------------------------------------
# Canvas origin from Vicon
# ---------------------------------------------------------------------------

def build_canvas_origin(frame) -> SE3Pose:
    """
    Compute SE3Pose (odom frame) for the gcode coordinate origin.
      origin  = canvas TL corner (mm → m)
      +X axis = canvas x_axis (TL → TR)
      +Y axis = canvas y_axis (TL → BL)
      +Z axis = canvas normal (upward from surface)
    """
    c      = frame.canvas
    tl     = c.corners[0]
    mat    = np.column_stack([c.x_axis, c.y_axis, c.normal])
    return SE3Pose(
        x=float(tl[0]) / 1000.0,
        y=float(tl[1]) / 1000.0,
        z=float(tl[2]) / 1000.0,
        rot=Quat.from_matrix(mat),
    )


# ---------------------------------------------------------------------------
# Shared execution core (Vicon-independent)
# ---------------------------------------------------------------------------

def _draw_gcode_impl(asc_client, cmd_client, state_client,
                     gcode_path: str, scale: float, tool_length: float,
                     odom_T_origin: SE3Pose, canvas_z: float,
                     canvas_width_mm: float = None,
                     canvas_height_mm: float = None) -> None:
    """
    Core drawing loop shared by draw_gcode and draw_gcode_manual.

    Parameters
    ----------
    odom_T_origin     SE3Pose for gcode (0, 0) in the odom frame.
                      Position in metres; rotation encodes canvas orientation
                      (+X = gcode X direction on the floor plane).
    canvas_z          Drawing surface height in odom frame (metres).
    canvas_width_mm / canvas_height_mm
                      Optional: only used for the pre-flight bounds check print.
                      Pass None to skip the check.
    """
    import logging
    logger = logging.getLogger("draw_autonomous.gcode_draw")

    print(f"  Canvas origin  odom ({odom_T_origin.x:.3f},"
          f" {odom_T_origin.y:.3f}, {odom_T_origin.z:.3f}) m"
          f"  |  canvas_z {canvas_z:.3f} m  |  scale {scale}")

    if canvas_width_mm is not None and canvas_height_mm is not None:

        class _FakeCanvas:
            width_mm  = canvas_width_mm
            height_mm = canvas_height_mm

        print_bounds_check(gcode_path, scale, _FakeCanvas())
    print()

    # ── Identity admittance frame (flat canvas) ───────────────────────────
    vision_T_admittance = geometry_pb2.SE3Pose(
        position=geometry_pb2.Vec3(x=0, y=0, z=0),
        rotation=geometry_pb2.Quaternion(w=1, x=0, y=0, z=0),
    )

    # ── Build GcodeReader ─────────────────────────────────────────────────
    gcode = GcodeReader(
        file_path=gcode_path,
        tool_length=tool_length,
        scale=scale,
        logger=logger,
        below_z_is_admittance=BELOW_Z_ADMITTANCE,
        travel_z=TRAVEL_Z,
        draw_on_wall=False,
    )
    gcode.set_origin(odom_T_origin, SE3Pose.from_proto(vision_T_admittance))
    print("  Gcode origin set.\n")

    ground_plane = [0.0, 0.0, canvas_z]

    # ── Move arm to starting position above canvas ────────────────────────
    robot_state = state_client.get_robot_state()
    (_, odom_T_flat_body, _, _, _, _) = get_transforms(False, robot_state)

    start_z_odom      = canvas_z + 0.30   # 30 cm above canvas surface
    body_z_odom       = odom_T_flat_body.z
    flat_body_T_hand  = SE3Pose(0.60, 0, start_z_odom - body_z_odom,
                                Quat(w=0.707, x=0, y=0.707, z=0))
    odom_T_hand_start = odom_T_flat_body * flat_body_T_hand
    p                 = odom_T_hand_start.to_proto()

    arm_cmd = RobotCommandBuilder.arm_pose_command(
        p.position.x, p.position.y, p.position.z,
        p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z,
        ODOM_FRAME_NAME, 0.000001,
    )
    cmd_id = cmd_client.robot_command(
        RobotCommandBuilder.build_synchro_command(arm_cmd))
    block_until_arm_arrives(cmd_client, cmd_id)
    print("  Arm at start position.")

    if not _ctrl.check():
        return

    # ── Execute gcode ─────────────────────────────────────────────────────
    print("  Executing gcode…\n")

    def _wait_m0():
        """Block on M0 pause until SPACE or estop."""
        print("  [M0] Gcode pause — press SPACE to continue", flush=True)
        while True:
            if not _ctrl.check():
                return False
            time.sleep(0.1)
            nonlocal vision_T_goals, is_admittance, is_pause
            (is_admittance, vision_T_goals, is_pause) = \
                gcode.get_next_vision_T_goals(ground_plane)
            if not is_pause:
                return True

    (is_admittance, vision_T_goals, is_pause) = \
        gcode.get_next_vision_T_goals(ground_plane)
    if is_pause:
        if not _wait_m0():
            return
    if vision_T_goals is None:
        print("  Gcode file is empty.")
        return

    robot_state = state_client.get_robot_state()
    move_arm(robot_state, is_admittance, vision_T_goals,
             asc_client, VELOCITY, False,
             vision_T_admittance, PRESS_FORCE_PERCENT,
             ODOM_FRAME_NAME, False, 0.0)

    odom_T_hand_goal = vision_T_goals[-1]
    last_admittance  = is_admittance

    while True:
        if not _ctrl.check():
            break

        robot_state = state_client.get_robot_state()
        (_, _, _, odom_T_hand, _, _) = get_transforms(False, robot_state)

        if is_admittance:
            dist = math.sqrt((odom_T_hand.x - odom_T_hand_goal.x)**2 +
                             (odom_T_hand.y - odom_T_hand_goal.y)**2)
        else:
            dist = math.sqrt((odom_T_hand.x - odom_T_hand_goal.x)**2 +
                             (odom_T_hand.y - odom_T_hand_goal.y)**2 +
                             (odom_T_hand.z - odom_T_hand_goal.z)**2)

        if dist < MIN_DIST_TO_GOAL:
            (is_admittance, vision_T_goals, is_pause) = \
                gcode.get_next_vision_T_goals(ground_plane)
            if is_pause:
                if not _wait_m0():
                    break
            if vision_T_goals is None:
                print("  Gcode complete.")
                break

            move_arm(robot_state, is_admittance, vision_T_goals,
                     asc_client, VELOCITY, False,
                     vision_T_admittance, PRESS_FORCE_PERCENT,
                     ODOM_FRAME_NAME, False, 0.0)
            odom_T_hand_goal = vision_T_goals[-1]

            if is_admittance != last_admittance:
                time.sleep(3.0 if is_admittance else 1.0)
            last_admittance = is_admittance
        elif not is_admittance:
            # Travel move: keep updating ground plane reference
            (is_admittance, vision_T_goals, is_pause) = \
                gcode.get_next_vision_T_goals(ground_plane, read_new_line=False)

        time.sleep(0.05)


# ---------------------------------------------------------------------------
# Main draw functions
# ---------------------------------------------------------------------------

def draw_gcode(vicon_client, asc_client, cmd_client, state_client,
               gcode_path: str, scale: float, tool_length: float) -> None:
    """
    Execute a gcode file using the Vicon canvas as the coordinate origin.

    - Reads the current canvas geometry from Vicon.
    - Builds a GcodeReader with the canvas TL corner as (0, 0).
    - Moves arm to start position above canvas.
    - Executes G00/G01/G02/G03 commands using ArmSurfaceContact.
    - Pauses on M0 (SPACE to continue).
    - Honours _ctrl.check() for SPACE/RETURN at every tick.
    """
    print("  Reading canvas origin from Vicon…")
    deadline = time.time() + 5.0
    frame = None
    while time.time() < deadline:
        frame = vicon_client.latest_frame
        if frame and frame.canvas and frame.canvas.is_valid():
            break
        time.sleep(0.1)
    if frame is None or frame.canvas is None or not frame.canvas.is_valid():
        raise RuntimeError("Canvas not visible in Vicon — cannot set gcode origin.")

    odom_T_origin = build_canvas_origin(frame)
    c = frame.canvas
    canvas_z = float(np.mean([c.corners[i][2] for i in range(4)])) / 1000.0

    _draw_gcode_impl(
        asc_client, cmd_client, state_client,
        gcode_path, scale, tool_length,
        odom_T_origin, canvas_z,
        canvas_width_mm=c.width_mm,
        canvas_height_mm=c.height_mm,
    )


def draw_gcode_vicon(vicon_client, asc_client, cmd_client, state_client,
                     gcode_path: str, tool_length: float,
                     margin: float = 0.90) -> None:
    """
    Execute a gcode file bounded and centered within the Vicon canvas.

    The 4-corner canvas geometry from Vicon defines the drawing space.
    The gcode is auto-scaled (aspect-ratio-preserved) to fill *margin* fraction
    of the canvas, then centered within it.  No explicit scale is needed.

    Parameters
    ----------
    vicon_client    Active ViconClient with a valid canvas in latest_frame.
    margin          Canvas fill fraction (0–1).  Default 0.90 leaves a 5% border.
    """
    print("  Reading canvas corners from Vicon…")
    deadline = time.time() + 5.0
    frame = None
    while time.time() < deadline:
        frame = vicon_client.latest_frame
        if frame and frame.canvas and frame.canvas.is_valid():
            break
        time.sleep(0.1)
    if frame is None or frame.canvas is None or not frame.canvas.is_valid():
        raise RuntimeError("Canvas not visible in Vicon — cannot bound gcode.")

    c = frame.canvas
    canvas_z = float(np.mean([c.corners[i][2] for i in range(4)])) / 1000.0

    # ── Auto-scale: fit gcode within canvas (margin applied) ─────────────
    x_min, x_max, y_min, y_max = scan_gcode_bounds(gcode_path, scale=1.0)
    gw = x_max - x_min
    gh = y_max - y_min

    if gw <= 0 or gh <= 0:
        print("  [WARN] Could not determine gcode extents — using default scale 0.001")
        scale = 0.001
    else:
        canvas_w_m = (c.width_mm / 1000.0) * margin
        canvas_h_m = (c.height_mm / 1000.0) * margin
        scale = min(canvas_w_m / gw, canvas_h_m / gh)

        fitted_w_mm = gw * scale * 1000
        fitted_h_mm = gh * scale * 1000
        print(f"  Gcode extents   {gw:.2f} x {gh:.2f} raw units")
        print(f"  Canvas          {c.width_mm:.0f} x {c.height_mm:.0f} mm"
              f"  (margin {margin*100:.0f}%)")
        print(f"  Auto-scale      {scale:.6f} m/unit"
              f"  ->  drawing {fitted_w_mm:.0f} x {fitted_h_mm:.0f} mm on canvas")

    # ── Center the drawing within the canvas ──────────────────────────────
    # After scaling, the drawing spans (x_min*scale … x_max*scale) in gcode
    # local coords.  We shift the origin so the drawing is centered on the canvas.
    drawing_w_m = gw * scale
    drawing_h_m = gh * scale
    # How much blank space to leave on each side
    pad_x = (c.width_mm  / 1000.0 - drawing_w_m) / 2.0   # along x_axis (TL→TR)
    pad_y = (c.height_mm / 1000.0 - drawing_h_m) / 2.0   # along y_axis (TL→BL)

    # gcode(0,0) maps to odom_T_origin.
    # We want gcode(x_min*scale) to land at  TL + x_axis * pad_x,
    # so gcode(0)  →  TL + x_axis * (pad_x − x_min*scale)
    #                    + y_axis * (pad_y − y_min*scale)
    tl_m  = c.corners[0] / 1000.0     # mm → m
    x_ax  = c.x_axis                  # unit vector
    y_ax  = c.y_axis                  # unit vector

    origin_m = (tl_m
                + x_ax * (pad_x - x_min * scale)
                + y_ax * (pad_y - y_min * scale))

    mat = np.column_stack([x_ax, y_ax, c.normal])
    odom_T_origin = SE3Pose(
        x=float(origin_m[0]),
        y=float(origin_m[1]),
        z=float(origin_m[2]),
        rot=Quat.from_matrix(mat),
    )

    print(f"  Canvas TL       ({tl_m[0]*1000:+.0f}, {tl_m[1]*1000:+.0f},"
          f" {tl_m[2]*1000:+.0f}) mm")
    print(f"  Gcode origin    ({origin_m[0]*1000:+.0f}, {origin_m[1]*1000:+.0f},"
          f" {origin_m[2]*1000:+.0f}) mm"
          f"  (centering pads {pad_x*1000:.0f} × {pad_y*1000:.0f} mm)")

    _draw_gcode_impl(
        asc_client, cmd_client, state_client,
        gcode_path, scale, tool_length,
        odom_T_origin, canvas_z,
        canvas_width_mm=c.width_mm,
        canvas_height_mm=c.height_mm,
    )


def draw_gcode_manual(asc_client, cmd_client, state_client,
                      gcode_path: str, scale: float, tool_length: float,
                      origin_x: float, origin_y: float, origin_z: float,
                      x_axis=(1.0, 0.0, 0.0),
                      canvas_z: float = None,
                      canvas_width_mm: float = None,
                      canvas_height_mm: float = None) -> None:
    """
    Execute a gcode file without Vicon, using manually supplied canvas parameters.

    Parameters
    ----------
    origin_x/y/z   Position of gcode (0, 0) in the odom frame, in metres.
                   Typically the top-left corner of the drawing area.
    x_axis         Unit vector (odom frame) for the gcode +X direction on the
                   floor plane.  Defaults to odom +X = (1, 0, 0).
    canvas_z       Drawing surface height in the odom frame (metres).
                   Defaults to origin_z if not supplied.
    canvas_width_mm / canvas_height_mm
                   Optional canvas dimensions in mm — only used for the
                   pre-flight bounds check print.  Pass None to skip.

    Example
    -------
    # Canvas TL corner 1 m ahead of robot, gcode X points to robot's right,
    # drawing surface is at floor level (z = 0):
    draw_gcode_manual(
        asc_client, cmd_client, state_client,
        "drawing.gcode", scale=0.001, tool_length=0.0,
        origin_x=1.0, origin_y=0.0, origin_z=0.0,
        x_axis=(0.0, -1.0, 0.0),
        canvas_z=0.0,
        canvas_width_mm=300, canvas_height_mm=200,
    )
    """
    xa = np.array(x_axis, dtype=float)
    xa /= np.linalg.norm(xa)
    za = np.array([0.0, 0.0, 1.0])
    ya = np.cross(za, xa)
    mat = np.column_stack([xa, ya, za])
    odom_T_origin = SE3Pose(
        x=float(origin_x),
        y=float(origin_y),
        z=float(origin_z),
        rot=Quat.from_matrix(mat),
    )

    _draw_gcode_impl(
        asc_client, cmd_client, state_client,
        gcode_path, scale, tool_length,
        odom_T_origin,
        canvas_z=float(canvas_z if canvas_z is not None else origin_z),
        canvas_width_mm=canvas_width_mm,
        canvas_height_mm=canvas_height_mm,
    )
