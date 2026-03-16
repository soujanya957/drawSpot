"""
Test: Autonomous navigation to brush or canvas
===============================================
Walks Spot from its current position to a Vicon-tracked target and stops.
Used to validate that the nav controller reaches the right standoff distance
before attempting the full autonomous draw pipeline.

Usage:
    python -m tests.test_nav --target brush
    python -m tests.test_nav --target canvas
    python -m tests.test_nav --target 500,200,0    # world XYZ in mm

Controls:
    SPACE    — pause / resume
    RETURN   — emergency stop (stow arm + sit)
"""

import argparse
import math
import os
import select
import sys
import termios
import threading
import time
import tty

import numpy as np

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config.robot_config import (
    BASE_ROTATION_MAX, BASE_VELOCITY_MAX,
    PASSWORD, ROBOT_IP, USERNAME, VICON_ADDRESS,
)
from draw.vicon.client import MockViconClient, ViconClient

# ---------------------------------------------------------------------------
# Tuning
# ---------------------------------------------------------------------------

BRUSH_STOP_MM  = 500   # stop when brush is this close
CANVAS_STOP_MM = 700   # stop when canvas centre is this close
NAV_SPEED      = 0.35  # m/s
BASE_CMD_DUR   = 0.6   # s, velocity command horizon

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


# ---------------------------------------------------------------------------
# Math
# ---------------------------------------------------------------------------

def _rotation_matrix(q):
    qx, qy, qz, qw = q
    return np.array([
        [1-2*(qy**2+qz**2),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [  2*(qx*qy+qz*qw), 1-2*(qx**2+qz**2),   2*(qy*qz-qx*qw)],
        [  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx**2+qy**2)],
    ], dtype=float)


# ---------------------------------------------------------------------------
# Navigation
# ---------------------------------------------------------------------------

def _nav_step(cmd_client, body, target_mm, stop_mm):
    """One nav tick. Returns True when within stop_mm."""
    pos   = np.array(body.position[:2])
    tgt   = np.array(target_mm[:2])
    delta = tgt - pos
    dist  = float(np.linalg.norm(delta))
    if dist < stop_mm:
        return True, dist

    R        = _rotation_matrix(body.rotation_quat)[:2, :2]
    dir_body = R.T @ (delta / dist)

    yaw_err = math.atan2(float(dir_body[1]), float(dir_body[0]))
    v_rot   = float(np.clip(yaw_err * 2.0, -BASE_ROTATION_MAX, BASE_ROTATION_MAX))
    fwd     = float(np.clip(dir_body[0], 0.0, 1.0)) * NAV_SPEED

    cmd_client.robot_command(
        RobotCommandBuilder.synchro_velocity_command(v_x=fwd, v_y=0, v_rot=v_rot),
        end_time_secs=time.time() + BASE_CMD_DUR,
    )
    return False, dist


def walk_to(cmd_client, vicon_client, target_fn, stop_mm, label):
    """Walk to target until within stop_mm. Returns False if stopped."""
    print(f"\n  Walking to {label}  (stop dist = {stop_mm:.0f} mm)…")
    while True:
        if not _check():
            return False
        frame = vicon_client.latest_frame
        if frame is None or frame.spot_body is None or frame.spot_body.occluded:
            time.sleep(0.05)
            continue

        target = target_fn(frame)
        if target is None:
            print(f"  [WARN] {label} not visible — waiting…", flush=True)
            time.sleep(0.5)
            continue

        arrived, dist_mm = _nav_step(cmd_client, frame.spot_body, target, stop_mm)
        spot_xy = frame.spot_body.position[:2]
        print(f"  Spot ({spot_xy[0]:+7.0f}, {spot_xy[1]:+7.0f}) mm  →  "
              f"{label} dist {dist_mm:6.0f} mm", end="\r", flush=True)

        if arrived:
            # Send zero-velocity to stop cleanly
            cmd_client.robot_command(
                RobotCommandBuilder.synchro_velocity_command(0, 0, 0),
                end_time_secs=time.time() + 0.2,
            )
            print(f"\n  Arrived at {label}  (final dist = {dist_mm:.0f} mm)")
            return True
        time.sleep(0.05)


# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------

def run(vicon_client, robot, target_arg):
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    cmd_client   = robot.ensure_client(RobotCommandClient.default_service_name)
    estop_client = robot.ensure_client(EstopClient.default_service_name)

    estop_ep = EstopEndpoint(estop_client, name="test_nav_estop", estop_timeout=9.0)
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

        # Resolve target
        if target_arg == "brush":
            def target_fn(frame):
                if frame.brush_tip and not frame.brush_tip.occluded:
                    return frame.brush_tip.position
                return None
            label, stop_mm = "brush", BRUSH_STOP_MM

        elif target_arg == "canvas":
            def target_fn(frame):
                if frame.canvas and frame.canvas.is_valid():
                    return np.mean(np.array(frame.canvas.corners), axis=0)
                return None
            label, stop_mm = "canvas", CANVAS_STOP_MM

        else:
            # Custom world position: "x,y,z" in mm
            coords = np.array([float(v) for v in target_arg.split(",")])
            def target_fn(frame): return coords
            label, stop_mm = f"({target_arg}) mm", 200.0

        walk_to(cmd_client, vicon_client, target_fn, stop_mm, label)

        print("\n  Done — stowing arm and sitting…")
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
    ap = argparse.ArgumentParser(description="Test: walk to brush or canvas")
    ap.add_argument("--target", default="canvas",
                    help="brush | canvas | 'x,y,z' in world mm (default: canvas)")
    ap.add_argument("--vicon", metavar="HOST:PORT",
                    help=f"Vicon address (default: from .env = {VICON_ADDRESS})")
    ap.add_argument("--mock", action="store_true", help="Mock Vicon client")
    args = ap.parse_args()

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
    print(f"Vicon OK  |  Spot {'OK' if (f.spot_body and not f.spot_body.occluded) else 'no data'}"
          f"  |  BrushTip {'OK' if (f.brush_tip and not f.brush_tip.occluded) else 'no data'}"
          f"  |  Canvas {'OK' if (f.canvas and f.canvas.is_valid()) else 'no data'}")

    sdk   = bosdyn.client.create_standard_sdk("TestNav")
    robot = sdk.create_robot(ROBOT_IP)
    robot.authenticate(USERNAME, PASSWORD)
    robot.time_sync.wait_for_sync()

    try:
        run(vicon, robot, args.target)
    finally:
        vicon.stop()


if __name__ == "__main__":
    main()
