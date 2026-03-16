"""
Test: Validate canvas drawing from current Spot position
=========================================================
Assumes Spot is already positioned at the canvas with a brush in its gripper.
Moves the arm to draw pose, then executes a pattern JSON — no navigation.

Used to validate:
  - Arm reaches correct canvas positions from current Spot stance
  - IK is reachable for the whole pattern
  - Speed limiting and pen up/down transitions work correctly

Usage:
    python -m tests.test_draw patterns/square.json
    python -m tests.test_draw patterns/spiral.json --speed 0.5
    python -m tests.test_draw patterns/square.json --mock     # mock Vicon

Controls:
    SPACE    — pause / resume
    RETURN   — emergency stop (stow arm + sit)
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

import numpy as np
from google.protobuf import duration_pb2

import bosdyn.client
import bosdyn.client.util
from bosdyn.api import robot_command_pb2
from bosdyn.api.geometry_pb2 import Quaternion
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config.robot_config import PASSWORD, ROBOT_IP, USERNAME, VICON_ADDRESS
from draw.vicon.client import MockViconClient, ViconClient
from draw.vicon.transform import canvas_to_world

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

BRUSH_LENGTH_MM = 150.0
PEN_UP_MM       = 60.0
ARM_REACH_MAX_M = 0.85
ARM_REACH_MIN_M = 0.15
DRAW_HZ         = 20
DEFAULT_SPEED   = 1.0   # UV/s (1.0 = fast, 0.3 = cautious)

# ---------------------------------------------------------------------------
# Pause / E-Stop
# ---------------------------------------------------------------------------

_pause = threading.Event()
_pause.set()
_estop = threading.Event()


def _get_key(timeout=0.1):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _key_listener():
    while not _estop.is_set():
        key = _get_key(0.15)
        if key == " ":
            if _pause.is_set():
                _pause.clear()
                print("\n  [PAUSED]  press SPACE to resume", flush=True)
            else:
                _pause.set()
                print("\n  [RESUMED]", flush=True)
        elif key in ("\r", "\n"):
            _estop.set()
            _pause.set()
            print("\n  [EMERGENCY STOP]", flush=True)


def _check():
    _pause.wait()
    return not _estop.is_set()


def _sleep(secs):
    end = time.time() + secs
    while time.time() < end:
        if not _check():
            return False
        time.sleep(0.05)
    return True


# ---------------------------------------------------------------------------
# Arm helpers
# ---------------------------------------------------------------------------

def _dur(s):
    return duration_pb2.Duration(seconds=int(s), nanos=int((s - int(s)) * 1e9))


def _rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return Quaternion(
        w=cr*cp*cy+sr*sp*sy, x=sr*cp*cy-cr*sp*sy,
        y=cr*sp*cy+sr*cp*sy, z=cr*cp*sy-sr*sp*cy,
    )


def _rotation_matrix(q):
    qx, qy, qz, qw = q
    return np.array([
        [1-2*(qy**2+qz**2),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [  2*(qx*qy+qz*qw), 1-2*(qx**2+qz**2),   2*(qy*qz-qx*qw)],
        [  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx**2+qy**2)],
    ], dtype=float)


_DOWN_QUAT = Quaternion(w=0.7071068, x=0.0, y=0.7071068, z=0.0)


def _world_to_body(target_mm, body_pos_mm, body_quat):
    R = _rotation_matrix(body_quat)
    return R.T @ (np.array(target_mm) - np.array(body_pos_mm)) / 1000.0


def _reachable(pos_body_m):
    d = float(np.linalg.norm(pos_body_m))
    return ARM_REACH_MIN_M <= d <= ARM_REACH_MAX_M and pos_body_m[0] >= 0.0


def _send_draw_command(cmd_client, frame, u, v, pen_down):
    if frame is None or frame.canvas is None or not frame.canvas.is_valid():
        return False, "no canvas"
    if frame.spot_body is None or frame.spot_body.occluded:
        return False, "no body"
    z_off     = BRUSH_LENGTH_MM + (0.0 if pen_down else PEN_UP_MM)
    target_mm = canvas_to_world(u, v, frame.canvas, z_offset_mm=z_off)
    pos_body  = _world_to_body(target_mm, frame.spot_body.position,
                               frame.spot_body.rotation_quat)
    if not _reachable(pos_body):
        return False, f"out of reach ({np.linalg.norm(pos_body):.3f} m)"
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
    return True, "ok"


def move_to_draw_pose(cmd_client, vicon_client):
    """Position arm above canvas centre."""
    print("  Moving to draw pose…")
    frame = vicon_client.latest_frame
    if frame is None or frame.canvas is None or not frame.canvas.is_valid():
        print("  [WARN] No canvas — using fixed body-frame fallback.")
        cmd = robot_command_pb2.RobotCommand()
        ac  = cmd.synchronized_command.arm_command.arm_cartesian_command
        ac.root_frame_name = "body"
        pt  = ac.pose_trajectory_in_task.points.add()
        pt.pose.position.x = 0.6
        pt.pose.position.y = 0.0
        pt.pose.position.z = -0.1
        pt.pose.rotation.CopyFrom(_rpy_to_quat(0, math.pi/2, 0))
        pt.time_since_reference.CopyFrom(_dur(3.0))
        cmd_client.robot_command(cmd)
        return _sleep(3.5)

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
    print(f"  Draw pose: ({x:.3f}, {y:.3f}, {z:.3f}) m in odom")
    return _sleep(3.5)


# ---------------------------------------------------------------------------
# Draw
# ---------------------------------------------------------------------------

def draw_strokes(cmd_client, vicon_client, strokes, speed_uv_per_s):
    """Execute pattern. Returns False if stopped."""
    tick     = 1.0 / DRAW_HZ
    max_step = speed_uv_per_s * tick
    cur_u, cur_v = 0.5, 0.5
    total_pts = sum(len(s) for s in strokes)
    done_pts  = 0
    skipped   = 0

    for s_idx, stroke in enumerate(strokes):
        if not stroke:
            continue
        su, sv = stroke[0]
        cur_u, cur_v = su, sv

        # Pen-up move to stroke start
        frame = vicon_client.latest_frame
        ok, reason = _send_draw_command(cmd_client, frame, su, sv, pen_down=False)
        if not ok:
            print(f"\n  [WARN] pen-up to start of stroke {s_idx+1}: {reason}")
        if not _sleep(0.6):
            return False

        print(f"  Stroke {s_idx+1}/{len(strokes)}  ({len(stroke)} pts)", flush=True)

        for tu, tv in stroke:
            while True:
                if not _check():
                    return False
                du = tu - cur_u
                dv = tv - cur_v
                dist = math.sqrt(du*du + dv*dv)
                if dist < 0.001:
                    break
                step  = min(dist, max_step)
                cur_u += (du / dist) * step
                cur_v += (dv / dist) * step
                frame  = vicon_client.latest_frame
                ok, reason = _send_draw_command(cmd_client, frame, cur_u, cur_v, pen_down=True)
                if not ok:
                    skipped += 1
                    print(f"\n  [SKIP] ({cur_u:.3f},{cur_v:.3f}): {reason}", flush=True)
                else:
                    done_pts += 1
                time.sleep(tick)

    # Final pen-up
    frame = vicon_client.latest_frame
    _send_draw_command(cmd_client, frame, cur_u, cur_v, pen_down=False)
    print(f"\n  Done.  {done_pts}/{total_pts} points sent  |  {skipped} skipped (out of reach)")
    return True


# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------

def run(vicon_client, robot, strokes, speed):
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    cmd_client   = robot.ensure_client(RobotCommandClient.default_service_name)
    estop_client = robot.ensure_client(EstopClient.default_service_name)

    estop_ep = EstopEndpoint(estop_client, name="test_draw_estop", estop_timeout=9.0)
    estop_ep.force_simple_setup()

    with (
        EstopKeepAlive(estop_ep),
        LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True),
    ):
        robot.power_on(timeout_sec=20)
        blocking_stand(cmd_client, timeout_sec=10)
        print("  Robot standing.\n")
        print("  SPACE = pause / resume     RETURN = emergency stop\n")

        kt = threading.Thread(target=_key_listener, daemon=True, name="KeyListener")
        kt.start()

        if not move_to_draw_pose(cmd_client, vicon_client):
            print("  Stopped before drawing.")
        elif not draw_strokes(cmd_client, vicon_client, strokes, speed):
            print("  Stopped mid-draw.")

        print("\n  Stowing arm and sitting…")
        try:
            cmd_client.robot_command(RobotCommandBuilder.arm_stow_command())
            time.sleep(2)
            cmd_client.robot_command(RobotCommandBuilder.synchro_sit_command())
            time.sleep(2)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Test: validate canvas drawing")
    ap.add_argument("pattern", help="Path to pattern JSON  (strokes format)")
    ap.add_argument("--speed", type=float, default=DEFAULT_SPEED,
                    help=f"Drawing speed in canvas UV/s (default {DEFAULT_SPEED})")
    ap.add_argument("--vicon", metavar="HOST:PORT",
                    help=f"Vicon address (default: from .env = {VICON_ADDRESS})")
    ap.add_argument("--mock", action="store_true", help="Mock Vicon client")
    args = ap.parse_args()

    with open(args.pattern) as f:
        pattern = json.load(f)
    strokes = pattern["strokes"]
    print(f"Pattern : {args.pattern}  ({len(strokes)} strokes)  speed={args.speed} UV/s\n")

    vicon_addr = args.vicon or VICON_ADDRESS
    if args.mock:
        vicon = MockViconClient()
    elif vicon_addr:
        vicon = ViconClient(host=vicon_addr)
    else:
        raise SystemExit("No VICON_HOST set and --mock not specified.")

    vicon.start()
    deadline = time.time() + 5.0
    while vicon.latest_frame is None and time.time() < deadline:
        time.sleep(0.05)
    if vicon.latest_frame is None:
        raise SystemExit("No Vicon frames received.")

    f = vicon.latest_frame
    print(f"Vicon OK  |  body {'OK' if (f.spot_body and not f.spot_body.occluded) else 'no data'}"
          f"  |  canvas {'OK' if (f.canvas and f.canvas.is_valid()) else 'no data'}")

    # Pre-validate: check every UV point for reachability and report
    if f.spot_body and not f.spot_body.occluded and f.canvas and f.canvas.is_valid():
        out = 0
        total = sum(len(s) for s in strokes)
        for stroke in strokes:
            for u, v in stroke:
                z_off     = BRUSH_LENGTH_MM
                target_mm = canvas_to_world(u, v, f.canvas, z_offset_mm=z_off)
                R         = _rotation_matrix(f.spot_body.rotation_quat)
                pos_body  = R.T @ (target_mm - np.array(f.spot_body.position)) / 1000.0
                if not _reachable(pos_body):
                    out += 1
        print(f"IK check : {total - out}/{total} waypoints reachable"
              + (f"  ({out} out of reach — will be skipped)" if out else "  ✓"))
    print()

    sdk   = bosdyn.client.create_standard_sdk("TestDraw")
    robot = sdk.create_robot(ROBOT_IP)
    robot.authenticate(USERNAME, PASSWORD)
    robot.time_sync.wait_for_sync()

    try:
        run(vicon, robot, strokes, args.speed)
    finally:
        vicon.stop()


if __name__ == "__main__":
    main()
