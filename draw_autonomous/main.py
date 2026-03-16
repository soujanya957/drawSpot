"""
draw_autonomous/main.py — Fully Autonomous Spot Gcode Drawing
=============================================================
Pipeline (repeated --repeats times):

  1. Walk to brush   — Vicon BrushTip marker guides navigation
  2. Pick up brush   — Vicon-guided arm, gripper close + lift
  3. Walk to canvas  — Vicon canvas corners as target
  4. Position arm    — brush pointing down above canvas centre
  5. Draw            — execute .gcode file via ArmSurfaceContact

Controls (terminal):
  SPACE    — pause / resume  (arm and base stop in place)
  RETURN   — emergency stop  (stow arm + sit)

Usage:
    python -m draw_autonomous.main my_drawing.gcode
    python -m draw_autonomous.main my_drawing.gcode --scale 0.001  # gcode in mm (default)
    python -m draw_autonomous.main my_drawing.gcode --repeats 3
    python -m draw_autonomous.main my_drawing.gcode --mock

Generate gcode from Inkscape:
  Extensions → Gcodetools → Path to Gcode
  Or any CAM tool that outputs G00/G01/G02/G03.
"""

import argparse
import os
import sys
import time

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.arm_surface_contact import ArmSurfaceContactClient
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.client.robot_state import RobotStateClient

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config.robot_config import PASSWORD, ROBOT_IP, USERNAME, VICON_ADDRESS  # noqa: E402
from vicon.client import MockViconClient, ViconClient                        # noqa: E402

import draw_autonomous._ctrl as _ctrl
from draw_autonomous.robot.navigator import (
    BRUSH_APPROACH_MM, CANVAS_STANDOFF_MM, walk_to, pick_brush, move_to_draw_pose,
)
from draw_autonomous.robot.gcode_draw import draw_gcode

VICON_CONNECT_TO = 5.0


# ---------------------------------------------------------------------------
# Full autonomous pipeline
# ---------------------------------------------------------------------------

def run(vicon_client, robot, gcode_path: str, scale: float,
        tool_length: float, repeats: int) -> None:
    lease_client  = robot.ensure_client(LeaseClient.default_service_name)
    cmd_client    = robot.ensure_client(RobotCommandClient.default_service_name)
    state_client  = robot.ensure_client(RobotStateClient.default_service_name)
    estop_client  = robot.ensure_client(EstopClient.default_service_name)
    asc_client    = robot.ensure_client(ArmSurfaceContactClient.default_service_name)

    estop_ep = EstopEndpoint(estop_client, name="draw_auto_estop", estop_timeout=9.0)
    estop_ep.force_simple_setup()

    with (
        EstopKeepAlive(estop_ep),
        LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True),
    ):
        robot.power_on(timeout_sec=20)
        blocking_stand(cmd_client, timeout_sec=10)
        print("Robot ready.\n")
        print("  SPACE = pause / resume     RETURN = emergency stop\n")

        _ctrl.start_key_listener()

        for rep in range(repeats):
            if _ctrl.estop.is_set():
                break

            print(f"\n{'━' * 52}")
            print(f"  REPETITION {rep + 1} / {repeats}")
            print(f"{'━' * 52}\n")

            # ── 1. Walk to brush ─────────────────────────────────────────
            def brush_target(frame):
                if frame.brush_tip and not frame.brush_tip.occluded:
                    return frame.brush_tip.position
                return None

            if not walk_to(cmd_client, vicon_client,
                           brush_target, BRUSH_APPROACH_MM, "brush"):
                break
            if not _ctrl.check():
                break

            # ── 2. Pick brush ────────────────────────────────────────────
            if not pick_brush(cmd_client, vicon_client):
                break
            if not _ctrl.check():
                break

            # ── 3. Walk to canvas ────────────────────────────────────────
            def canvas_target(frame):
                if frame.canvas and frame.canvas.is_valid():
                    return np.mean(np.array(frame.canvas.corners), axis=0)
                return None

            if not walk_to(cmd_client, vicon_client,
                           canvas_target, CANVAS_STANDOFF_MM, "canvas"):
                break
            if not _ctrl.check():
                break

            # ── 4. Move to draw pose ─────────────────────────────────────
            if not move_to_draw_pose(cmd_client, vicon_client):
                break
            if not _ctrl.check():
                break

            # ── 5. Draw gcode ────────────────────────────────────────────
            try:
                draw_gcode(vicon_client, asc_client, cmd_client, state_client,
                           gcode_path, scale, tool_length)
            except RuntimeError as e:
                print(f"  [ERROR] {e}")
                break

        # ── Clean exit ───────────────────────────────────────────────────
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
    ap = argparse.ArgumentParser(
        description="draw_autonomous — fully autonomous Spot gcode drawing")
    ap.add_argument("gcode",
                    help="Path to .gcode file (G00/G01/G02/G03)")
    ap.add_argument("--scale", type=float, default=0.001,
                    help="Metres per gcode unit (default 0.001 = gcode in mm)")
    ap.add_argument("--tool-length", type=float, default=0.0,
                    help="Brush length beyond wrist (m, default 0.0)")
    ap.add_argument("--repeats", type=int, default=1,
                    help="Number of full draw cycles (default 1)")
    ap.add_argument("--vicon", metavar="HOST:PORT",
                    help=f"Vicon address (default: from .env = {VICON_ADDRESS})")
    ap.add_argument("--mock", action="store_true",
                    help="Use simulated Vicon client (no hardware)")
    args = ap.parse_args()

    if not os.path.exists(args.gcode):
        raise SystemExit(f"Gcode file not found: {args.gcode}")

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
    print("Vicon connected.")
    print(f"  Spot body : {'OK' if (frame.spot_body and not frame.spot_body.occluded) else 'no data'}")
    print(f"  BrushTip  : {'OK' if (frame.brush_tip and not frame.brush_tip.occluded) else 'no data'}")
    print(f"  Canvas    : {'OK' if (frame.canvas and frame.canvas.is_valid()) else 'no data'}")

    if frame.canvas is None or not frame.canvas.is_valid():
        raise SystemExit("Canvas markers not visible — required to set gcode origin.")
    print()

    # ── Spot ─────────────────────────────────────────────────────────────
    print("Connecting to Spot…")
    bosdyn.client.util.setup_logging(verbose=False)
    sdk   = bosdyn.client.create_standard_sdk("DrawAutonomous")
    robot = sdk.create_robot(ROBOT_IP)
    robot.authenticate(USERNAME, PASSWORD)
    robot.time_sync.wait_for_sync()
    print("Spot connected.\n")

    try:
        run(vicon, robot, args.gcode, args.scale, args.tool_length, args.repeats)
    finally:
        vicon.stop()


if __name__ == "__main__":
    main()
