"""
draw_autonomous.robot.navigator
================================
Autonomous navigation, brush pickup, and draw-pose positioning.

Public API
----------
  walk_to(cmd_client, vicon_client, target_fn, stop_mm, label) -> bool
  pick_brush(cmd_client, vicon_client)                         -> bool
  move_to_draw_pose(cmd_client, vicon_client)                  -> bool

All functions return False immediately if the emergency stop fires.
They honour pause via _ctrl.check() / _ctrl.sleep().
"""

import math
import time
from typing import Callable, Optional

import numpy as np
from google.protobuf import duration_pb2

from bosdyn.api import robot_command_pb2
from bosdyn.api.geometry_pb2 import Quaternion
from bosdyn.client.robot_command import RobotCommandBuilder

import draw_autonomous._ctrl as _ctrl

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

BRUSH_APPROACH_MM  = 500    # stop walking when this close to brush (mm)
CANVAS_STANDOFF_MM = 700    # stop walking when this close to canvas centre (mm)
BRUSH_LENGTH_MM    = 150.0  # brush extends below gripper tip (mm)
PEN_UP_MM          = 60.0   # clearance above canvas when pen is raised (mm)
ARM_REACH_MAX_M    = 0.85
ARM_REACH_MIN_M    = 0.15
BASE_CMD_DUR       = 0.6    # duration passed to velocity commands (s)
NAV_SPEED          = 0.35   # m/s top navigation speed

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


def _reachable(pos_body_m: np.ndarray) -> bool:
    d = float(np.linalg.norm(pos_body_m))
    return ARM_REACH_MIN_M <= d <= ARM_REACH_MAX_M and pos_body_m[0] >= 0.0


# ---------------------------------------------------------------------------
# Arm helpers
# ---------------------------------------------------------------------------

# Gripper pointing straight down (pitch = 90°)
_DOWN_QUAT = Quaternion(w=0.7071068, x=0.0, y=0.7071068, z=0.0)


def _arm_odom(cmd_client, x: float, y: float, z: float,
              quat: Quaternion, dur_s: float = 2.0) -> None:
    """Send an arm Cartesian command in the odom frame."""
    cmd = robot_command_pb2.RobotCommand()
    ac  = cmd.synchronized_command.arm_command.arm_cartesian_command
    ac.root_frame_name = "odom"
    pt  = ac.pose_trajectory_in_task.points.add()
    pt.pose.position.x = x
    pt.pose.position.y = y
    pt.pose.position.z = z
    pt.pose.rotation.CopyFrom(quat)
    pt.time_since_reference.CopyFrom(_dur(dur_s))
    cmd_client.robot_command(cmd)


# ---------------------------------------------------------------------------
# Navigation
# ---------------------------------------------------------------------------

def _nav_step(cmd_client, body, target_mm: np.ndarray, stop_mm: float) -> bool:
    """
    One navigation tick toward *target_mm* (world XY).
    Returns True when Spot is within *stop_mm*.
    Uses a proportional yaw controller to face the target before moving forward.
    """
    pos   = np.array(body.position[:2])
    tgt   = np.array(target_mm[:2])
    delta = tgt - pos
    dist  = float(np.linalg.norm(delta))

    if dist < stop_mm:
        return True

    R        = _rotation_matrix(body.rotation_quat)[:2, :2]
    dir_body = R.T @ (delta / dist)

    yaw_err = math.atan2(float(dir_body[1]), float(dir_body[0]))
    from config.robot_config import BASE_ROTATION_MAX
    v_rot   = float(np.clip(yaw_err * 2.0, -BASE_ROTATION_MAX, BASE_ROTATION_MAX))
    fwd     = float(np.clip(dir_body[0], 0.0, 1.0)) * NAV_SPEED

    cmd_client.robot_command(
        RobotCommandBuilder.synchro_velocity_command(v_x=fwd, v_y=0, v_rot=v_rot),
        end_time_secs=time.time() + BASE_CMD_DUR,
    )
    return False


def walk_to(cmd_client, vicon_client,
            target_fn: Callable, stop_mm: float, label: str) -> bool:
    """
    Walk until target_fn(frame) returns a world position within *stop_mm*.
    *target_fn* receives the latest ViconFrame and returns a (3,) mm ndarray or None.
    Returns False if emergency stop triggered.
    """
    print(f"  Walking to {label}…")
    while True:
        if not _ctrl.check():
            return False
        frame = vicon_client.latest_frame
        if frame is None or frame.spot_body is None or frame.spot_body.occluded:
            time.sleep(0.05)
            continue
        target = target_fn(frame)
        if target is None:
            print(f"  [WARN] {label} not visible in Vicon — waiting…", flush=True)
            if not _ctrl.sleep(0.5):
                return False
            continue
        if _nav_step(cmd_client, frame.spot_body, target, stop_mm):
            cmd_client.robot_command(
                RobotCommandBuilder.synchro_velocity_command(0, 0, 0),
                end_time_secs=time.time() + 0.2,
            )
            print(f"  Arrived at {label}.")
            return True
        time.sleep(0.05)


# ---------------------------------------------------------------------------
# Brush pickup
# ---------------------------------------------------------------------------

def pick_brush(cmd_client, vicon_client) -> bool:
    """
    Autonomously pick up the brush using the Vicon BrushTip marker.

    Sequence:
      1. Open gripper
      2. Move arm 200 mm above BrushTip
      3. Lower to 20 mm above tip
      4. Close gripper
      5. Lift 100 mm

    Returns False if stopped or BrushTip not visible.
    """
    frame = vicon_client.latest_frame
    if frame is None or frame.brush_tip is None or frame.brush_tip.occluded:
        print("  [PICK] BrushTip marker not visible — cannot pick brush.")
        return False

    print("  [PICK] Opening gripper…")
    cmd_client.robot_command(RobotCommandBuilder.claw_gripper_open_command())
    if not _ctrl.sleep(1.0):
        return False

    # Refresh position
    frame = vicon_client.latest_frame
    if frame is None or frame.brush_tip is None or frame.brush_tip.occluded:
        print("  [PICK] BrushTip lost — aborting pick.")
        return False
    bp = np.array(frame.brush_tip.position, dtype=float)  # mm

    print("  [PICK] Moving arm above brush…")
    ax, ay, az = (bp + np.array([0.0, 0.0, 200.0])) / 1000.0
    _arm_odom(cmd_client, ax, ay, az, _DOWN_QUAT, dur_s=2.5)
    if not _ctrl.sleep(3.0):
        return False

    print("  [PICK] Lowering to brush…")
    frame = vicon_client.latest_frame
    if frame and frame.brush_tip and not frame.brush_tip.occluded:
        bp = np.array(frame.brush_tip.position, dtype=float)
    bx, by, bz = (bp + np.array([0.0, 0.0, 20.0])) / 1000.0
    _arm_odom(cmd_client, bx, by, bz, _DOWN_QUAT, dur_s=1.5)
    if not _ctrl.sleep(2.0):
        return False

    print("  [PICK] Closing gripper…")
    cmd_client.robot_command(RobotCommandBuilder.claw_gripper_close_command())
    if not _ctrl.sleep(1.5):
        return False

    print("  [PICK] Lifting…")
    _arm_odom(cmd_client, ax, ay, az + 0.1, _DOWN_QUAT, dur_s=1.5)
    if not _ctrl.sleep(2.0):
        return False

    print("  [PICK] Brush acquired.")
    return True


# ---------------------------------------------------------------------------
# Draw pose
# ---------------------------------------------------------------------------

def move_to_draw_pose(cmd_client, vicon_client) -> bool:
    """
    Move arm above canvas centre, brush pointing straight down.
    Uses Vicon canvas geometry to compute the position in odom frame.
    Returns False if canvas not visible or stopped.
    """
    print("  Moving to draw pose…")
    frame = vicon_client.latest_frame
    if frame is None or frame.canvas is None or not frame.canvas.is_valid():
        print("  [WARN] Canvas not visible — cannot position for drawing.")
        return False

    c      = frame.canvas
    center = np.mean(np.array(c.corners), axis=0)
    pos_mm = center + (BRUSH_LENGTH_MM + PEN_UP_MM) * c.normal
    x, y, z = pos_mm / 1000.0

    cmd = robot_command_pb2.RobotCommand()
    ac  = cmd.synchronized_command.arm_command.arm_cartesian_command
    ac.root_frame_name = "odom"
    pt  = ac.pose_trajectory_in_task.points.add()
    pt.pose.position.x = x
    pt.pose.position.y = y
    pt.pose.position.z = z
    pt.pose.rotation.CopyFrom(_DOWN_QUAT)
    pt.time_since_reference.CopyFrom(_dur(3.0))
    cmd_client.robot_command(cmd)

    if not _ctrl.sleep(3.5):
        return False
    print(f"  Draw pose reached  ({x:.3f}, {y:.3f}, {z:.3f}) m")
    return True
