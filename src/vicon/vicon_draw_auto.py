"""
Spot Draw — Autonomous
======================
Fully autonomous drawing pipeline:

  1. Walk to brush  (Vicon BrushTip marker, or fallback position in JSON)
  2. Pick up brush  (Vicon-guided arm → close gripper)
  3. Walk to canvas (Vicon canvas corners → stop at standoff distance)
  4. Position arm   (draw pose above canvas center, brush pointing down)
  5. Draw pattern   (execute strokes from JSON file at capped speed)
  6. Repeat         (--repeats N, default 1)

Controls (terminal, always active):
  SPACE    — pause / resume  (base and arm stop in place)
  RETURN   — emergency stop  (stow arm → sit)

Pattern JSON format  (see patterns/square.json for a live example):
  {
    "brush_world_mm": [x, y, z],        // optional: fallback brush position
    "strokes": [
      [[u1,v1], [u2,v2], [u3,v3]],     // pen-down stroke (list of UV points)
      [[u4,v4], [u5,v5]]               // another stroke; pen lifts between
    ]
  }
  u, v are normalised canvas coordinates in [0, 1] (0,0 = TL corner).

Run:
    python -m src.vicon.vicon_draw_auto patterns/square.json
    python -m src.vicon.vicon_draw_auto patterns/square.json --repeats 3
    python -m src.vicon.vicon_draw_auto patterns/square.json --mock
"""

import argparse
import json
import math
import os
import select
import sys
import termios
import threading
import time
import tty
from typing import Callable, Optional

import numpy as np
from google.protobuf import duration_pb2

import bosdyn.client
import bosdyn.client.util
from bosdyn.api import robot_command_pb2
from bosdyn.api.geometry_pb2 import Quaternion
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import BODY_FRAME_NAME
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand

sys.path.insert(0, __file__.split("/src/")[0])
from config.robot_config import (
    BASE_ROTATION_MAX, BASE_VELOCITY_MAX,
    PASSWORD, ROBOT_IP, USERNAME, VICON_ADDRESS,
)
from draw.vicon.client import MockViconClient, ViconClient
from draw.vicon.transform import canvas_to_world

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

BRUSH_APPROACH_MM  = 500    # stop walking when brush is this close (mm)
CANVAS_STANDOFF_MM = 700    # stop walking when canvas center is this close (mm)
BRUSH_LENGTH_MM    = 150.0  # brush extends below gripper tip
PEN_UP_MM          = 60.0   # extra z clearance when pen is raised
ARM_REACH_MAX_M    = 0.85
ARM_REACH_MIN_M    = 0.15
BASE_CMD_DUR       = 0.6    # duration passed to velocity commands (s)
NAV_SPEED          = 0.35   # m/s top navigation speed
DRAW_HZ            = 20     # arm command rate while drawing
MAX_UV_STEP        = 0.018  # UV per tick at DRAW_HZ ≈ 0.36 UV/s on canvas
VICON_CONNECT_TO   = 5.0    # seconds to wait for first Vicon frame

# ---------------------------------------------------------------------------
# Pause / E-Stop state  (module-level so helpers can access them)
# ---------------------------------------------------------------------------

_pause = threading.Event()
_pause.set()   # "set" means NOT paused (clear = paused)
_estop = threading.Event()


# ---------------------------------------------------------------------------
# Key listener
# ---------------------------------------------------------------------------

def _get_key(timeout: float = 0.1) -> Optional[str]:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _key_listener_thread() -> None:
    """Background thread: SPACE = pause/resume, RETURN = emergency stop."""
    while not _estop.is_set():
        key = _get_key(timeout=0.15)
        if key == " ":
            if _pause.is_set():
                _pause.clear()
                print("\n  [PAUSED]   press SPACE to resume", flush=True)
            else:
                _pause.set()
                print("\n  [RESUMED]", flush=True)
        elif key in ("\r", "\n"):
            _estop.set()
            _pause.set()   # unblock any _check() waiters
            print("\n  [EMERGENCY STOP]", flush=True)


def _check() -> bool:
    """Block while paused. Return False if emergency stop was requested."""
    _pause.wait()
    return not _estop.is_set()


def _sleep(secs: float) -> bool:
    """Sleep for secs, honouring pause/estop every 50 ms. Returns False if stopped."""
    end = time.time() + secs
    while time.time() < end:
        if not _check():
            return False
        time.sleep(0.05)
    return True


# ---------------------------------------------------------------------------
# Math / geometry helpers
# ---------------------------------------------------------------------------

def _dur(s: float):
    return duration_pb2.Duration(seconds=int(s), nanos=int((s - int(s)) * 1e9))


def _rpy_to_quat(roll, pitch, yaw) -> Quaternion:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(
        w=cr*cp*cy + sr*sp*sy, x=sr*cp*cy - cr*sp*sy,
        y=cr*sp*cy + sr*cp*sy, z=cr*cp*sy - sr*sp*cy,
    )


def _rotation_matrix(q) -> np.ndarray:
    qx, qy, qz, qw = q
    return np.array([
        [1-2*(qy**2+qz**2),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [  2*(qx*qy+qz*qw), 1-2*(qx**2+qz**2),   2*(qy*qz-qx*qw)],
        [  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx**2+qy**2)],
    ], dtype=float)


def _world_to_body(target_mm, body_pos_mm, body_quat) -> np.ndarray:
    R = _rotation_matrix(body_quat)
    return R.T @ (np.array(target_mm) - np.array(body_pos_mm)) / 1000.0


def _reachable(pos_body_m) -> bool:
    d = float(np.linalg.norm(pos_body_m))
    return ARM_REACH_MIN_M <= d <= ARM_REACH_MAX_M and pos_body_m[0] >= 0.0


# ---------------------------------------------------------------------------
# Arm command helpers
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


def _send_draw_command(cmd_client, frame, u: float, v: float,
                       pen_down: bool) -> bool:
    """UV → odom arm command. Returns True if sent, False if out of reach."""
    if frame is None or frame.canvas is None or not frame.canvas.is_valid():
        return False
    if frame.spot_body is None or frame.spot_body.occluded:
        return False
    z_off     = BRUSH_LENGTH_MM + (0.0 if pen_down else PEN_UP_MM)
    target_mm = canvas_to_world(u, v, frame.canvas, z_offset_mm=z_off)
    pos_body  = _world_to_body(target_mm, frame.spot_body.position,
                               frame.spot_body.rotation_quat)
    if not _reachable(pos_body):
        return False
    x, y, z = target_mm / 1000.0
    cmd = robot_command_pb2.RobotCommand()
    ac  = cmd.synchronized_command.arm_command.arm_cartesian_command
    ac.root_frame_name = "odom"
    pt  = ac.pose_trajectory_in_task.points.add()
    pt.pose.position.x = x
    pt.pose.position.y = y
    pt.pose.position.z = z
    pt.pose.rotation.CopyFrom(_DOWN_QUAT)
    pt.time_since_reference.CopyFrom(_dur(0.15))
    cmd_client.robot_command(cmd)
    return True


# ---------------------------------------------------------------------------
# Pattern loading
# ---------------------------------------------------------------------------

def load_pattern(path: str) -> dict:
    with open(path) as f:
        data = json.load(f)
    if "strokes" not in data:
        raise ValueError(f"{path}: missing 'strokes' key")
    return data


# ---------------------------------------------------------------------------
# Navigation
# ---------------------------------------------------------------------------

def _nav_step(cmd_client, body, target_mm, stop_mm: float) -> bool:
    """
    One navigation tick toward target_mm (world XY).
    Returns True when Spot is within stop_mm.
    Includes proportional yaw control to face the target.
    """
    pos   = np.array(body.position[:2])
    tgt   = np.array(target_mm[:2])
    delta = tgt - pos
    dist  = float(np.linalg.norm(delta))

    if dist < stop_mm:
        return True

    # Direction in body frame
    R        = _rotation_matrix(body.rotation_quat)[:2, :2]
    dir_body = R.T @ (delta / dist)

    # Yaw proportional controller: turn to face target
    yaw_err = math.atan2(float(dir_body[1]), float(dir_body[0]))
    v_rot   = float(np.clip(yaw_err * 2.0, -BASE_ROTATION_MAX, BASE_ROTATION_MAX))

    # Forward only when roughly aligned (within ~45°) to avoid sidestep wobble
    fwd = float(np.clip(dir_body[0], 0.0, 1.0)) * NAV_SPEED

    cmd_client.robot_command(
        RobotCommandBuilder.synchro_velocity_command(v_x=fwd, v_y=0, v_rot=v_rot),
        end_time_secs=time.time() + BASE_CMD_DUR,
    )
    return False


def walk_to(cmd_client, vicon_client,
            target_fn: Callable, stop_mm: float, label: str) -> bool:
    """
    Walk until target_fn(frame) returns a world position within stop_mm.
    target_fn should return a (3,) ndarray or None if not yet visible.
    Returns False if emergency stop triggered.
    """
    print(f"  Walking to {label}…")
    while True:
        if not _check():
            return False
        frame = vicon_client.latest_frame
        if frame is None or frame.spot_body is None or frame.spot_body.occluded:
            time.sleep(0.05)
            continue
        target = target_fn(frame)
        if target is None:
            print(f"  [WARN] {label} not visible in Vicon — waiting…", flush=True)
            if not _sleep(0.5):
                return False
            continue
        if _nav_step(cmd_client, frame.spot_body, target, stop_mm):
            # Send a zero-velocity command to stop the base
            cmd_client.robot_command(
                RobotCommandBuilder.synchro_velocity_command(0, 0, 0),
                end_time_secs=time.time() + 0.2,
            )
            print(f"  Arrived at {label}.")
            return True
        time.sleep(0.05)


# ---------------------------------------------------------------------------
# Pick brush
# ---------------------------------------------------------------------------

def pick_brush(cmd_client, vicon_client) -> bool:
    """
    Autonomously pick up the brush using the Vicon BrushTip marker.
    Sequence: open gripper → arm above brush → lower → close → lift.
    Returns False if stopped or marker not visible.
    """
    frame = vicon_client.latest_frame
    if frame is None or frame.brush_tip is None or frame.brush_tip.occluded:
        print("  [PICK] BrushTip marker not visible — cannot pick brush.")
        return False

    print("  [PICK] Opening gripper…")
    cmd_client.robot_command(RobotCommandBuilder.claw_gripper_open_command())
    if not _sleep(1.0):
        return False

    # Refresh brush position
    frame = vicon_client.latest_frame
    if frame is None or frame.brush_tip is None or frame.brush_tip.occluded:
        print("  [PICK] BrushTip lost — aborting pick.")
        return False
    bp = np.array(frame.brush_tip.position, dtype=float)  # mm in world/odom frame

    print("  [PICK] Moving arm above brush…")
    ax, ay, az = (bp + np.array([0.0, 0.0, 200.0])) / 1000.0   # 200 mm above
    _arm_odom(cmd_client, ax, ay, az, _DOWN_QUAT, dur_s=2.5)
    if not _sleep(3.0):
        return False

    print("  [PICK] Lowering to brush…")
    # Re-read position one more time for accuracy
    frame = vicon_client.latest_frame
    if frame and frame.brush_tip and not frame.brush_tip.occluded:
        bp = np.array(frame.brush_tip.position, dtype=float)
    bx, by, bz = (bp + np.array([0.0, 0.0, 20.0])) / 1000.0    # 20 mm above tip
    _arm_odom(cmd_client, bx, by, bz, _DOWN_QUAT, dur_s=1.5)
    if not _sleep(2.0):
        return False

    print("  [PICK] Closing gripper…")
    cmd_client.robot_command(RobotCommandBuilder.claw_gripper_close_command())
    if not _sleep(1.5):
        return False

    print("  [PICK] Lifting…")
    _arm_odom(cmd_client, ax, ay, az + 0.1, _DOWN_QUAT, dur_s=1.5)  # extra 100 mm up
    if not _sleep(2.0):
        return False

    print("  [PICK] Brush acquired.")
    return True


# ---------------------------------------------------------------------------
# Draw pose
# ---------------------------------------------------------------------------

def move_to_draw_pose(cmd_client, vicon_client) -> bool:
    """
    Move arm to above canvas center, brush pointing straight down.
    Uses Vicon canvas geometry; returns False if canvas not visible or stopped.
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

    if not _sleep(3.5):
        return False
    print(f"  Draw pose reached  ({x:.3f}, {y:.3f}, {z:.3f}) m")
    return True


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------

def draw_strokes(cmd_client, vicon_client, strokes: list) -> bool:
    """
    Execute all strokes from the pattern.
    Speed-limited to MAX_UV_STEP per tick at DRAW_HZ.
    Returns False if stopped mid-draw.
    """
    tick = 1.0 / DRAW_HZ
    cur_u, cur_v = 0.5, 0.5

    for s_idx, stroke in enumerate(strokes):
        if not stroke:
            continue
        print(f"  Stroke {s_idx + 1}/{len(strokes)}…")

        # Pen-up move to the start of this stroke
        su, sv   = stroke[0]
        cur_u, cur_v = su, sv
        frame = vicon_client.latest_frame
        _send_draw_command(cmd_client, frame, su, sv, pen_down=False)
        if not _sleep(0.6):
            return False

        # Execute stroke: smooth interpolation between each waypoint
        for tu, tv in stroke:
            while True:
                if not _check():
                    return False
                du   = tu - cur_u
                dv   = tv - cur_v
                dist = math.sqrt(du * du + dv * dv)
                if dist < 0.001:
                    break
                step   = min(dist, MAX_UV_STEP)
                cur_u += (du / dist) * step
                cur_v += (dv / dist) * step
                frame  = vicon_client.latest_frame
                ok     = _send_draw_command(cmd_client, frame, cur_u, cur_v, pen_down=True)
                if not ok:
                    print(f"  [WARN] ({cur_u:.3f},{cur_v:.3f}) out of reach — skipping",
                          flush=True)
                time.sleep(tick)

    # Final pen-up
    frame = vicon_client.latest_frame
    _send_draw_command(cmd_client, frame, cur_u, cur_v, pen_down=False)
    print("  Pattern complete.")
    return True


# ---------------------------------------------------------------------------
# Main run loop
# ---------------------------------------------------------------------------

def run(vicon_client, robot, pattern: dict, repeats: int) -> None:
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    cmd_client   = robot.ensure_client(RobotCommandClient.default_service_name)
    estop_client = robot.ensure_client(EstopClient.default_service_name)

    estop_ep = EstopEndpoint(estop_client, name="spot_auto_draw_estop", estop_timeout=9.0)
    estop_ep.force_simple_setup()

    with (
        EstopKeepAlive(estop_ep),
        LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True),
    ):
        robot.power_on(timeout_sec=20)
        blocking_stand(cmd_client, timeout_sec=10)
        print("Robot ready.\n")
        print("  SPACE = pause / resume     RETURN = emergency stop\n")

        # Start key listener thread
        kt = threading.Thread(target=_key_listener_thread, daemon=True, name="KeyListener")
        kt.start()

        strokes          = pattern["strokes"]
        brush_fallback   = pattern.get("brush_world_mm")  # optional ndarray

        for rep in range(repeats):
            if _estop.is_set():
                break

            print(f"\n{'━' * 52}")
            print(f"  REPETITION {rep + 1} / {repeats}")
            print(f"{'━' * 52}")

            # ── 1. Walk to brush ────────────────────────────────────────
            def brush_target(frame):
                if frame.brush_tip and not frame.brush_tip.occluded:
                    return frame.brush_tip.position
                if brush_fallback:
                    return np.array(brush_fallback, dtype=float)
                return None

            if not walk_to(cmd_client, vicon_client,
                           brush_target, BRUSH_APPROACH_MM, "brush"):
                break
            if not _check():
                break

            # ── 2. Pick brush ───────────────────────────────────────────
            if not pick_brush(cmd_client, vicon_client):
                break
            if not _check():
                break

            # ── 3. Walk to canvas ───────────────────────────────────────
            def canvas_target(frame):
                if frame.canvas and frame.canvas.is_valid():
                    return np.mean(np.array(frame.canvas.corners), axis=0)
                return None

            if not walk_to(cmd_client, vicon_client,
                           canvas_target, CANVAS_STANDOFF_MM, "canvas"):
                break
            if not _check():
                break

            # ── 4. Draw pose ────────────────────────────────────────────
            if not move_to_draw_pose(cmd_client, vicon_client):
                break
            if not _check():
                break

            # ── 5. Draw pattern ─────────────────────────────────────────
            if not draw_strokes(cmd_client, vicon_client, strokes):
                break

        # ── Clean exit / emergency cleanup ──────────────────────────────
        print("\nCleaning up — stowing arm and sitting…")
        try:
            cmd_client.robot_command(RobotCommandBuilder.arm_stow_command())
            time.sleep(2)
            cmd_client.robot_command(RobotCommandBuilder.synchro_sit_command())
            time.sleep(2)
        except Exception:
            pass
        print("Done.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description="Spot Draw — Autonomous")
    ap.add_argument("pattern",            help="Path to pattern JSON file")
    ap.add_argument("--repeats", type=int, default=1,
                    help="Number of times to execute the full draw cycle (default 1)")
    ap.add_argument("--vicon", metavar="HOST:PORT",
                    help=f"Vicon address (default: VICON_ADDRESS from .env = {VICON_ADDRESS})")
    ap.add_argument("--mock", action="store_true",
                    help="Use simulated Vicon client (no hardware)")
    args = ap.parse_args()

    pattern = load_pattern(args.pattern)
    print(f"Pattern : {args.pattern}  "
          f"({len(pattern['strokes'])} stroke(s), {args.repeats}x repeat)\n")

    # ── Vicon ────────────────────────────────────────────────────────────
    vicon_addr = args.vicon or VICON_ADDRESS
    if args.mock:
        print("Using mock Vicon client.")
        vicon = MockViconClient()
    elif vicon_addr:
        print(f"Connecting to Vicon at {vicon_addr}…")
        vicon = ViconClient(host=vicon_addr)
    else:
        raise SystemExit("No VICON_HOST set and --mock not specified.")

    vicon.start()
    deadline = time.time() + VICON_CONNECT_TO
    while vicon.latest_frame is None and time.time() < deadline:
        time.sleep(0.05)
    if vicon.latest_frame is None:
        raise SystemExit(f"No Vicon frames after {VICON_CONNECT_TO:.0f} s — check connection.")

    frame = vicon.latest_frame
    print(f"Vicon connected.")
    print(f"  Spot body : {'OK' if (frame.spot_body and not frame.spot_body.occluded) else 'no data'}")
    print(f"  BrushTip  : {'OK' if (frame.brush_tip and not frame.brush_tip.occluded) else 'no data'}")
    print(f"  Canvas    : {'OK' if (frame.canvas and frame.canvas.is_valid()) else 'no data'}\n")

    # ── Spot ─────────────────────────────────────────────────────────────
    print("Connecting to Spot…")
    sdk   = bosdyn.client.create_standard_sdk("SpotDrawAuto")
    robot = sdk.create_robot(ROBOT_IP)
    robot.authenticate(USERNAME, PASSWORD)
    robot.time_sync.wait_for_sync()
    print("Spot connected.\n")

    try:
        run(vicon, robot, pattern, args.repeats)
    finally:
        vicon.stop()


if __name__ == "__main__":
    main()
