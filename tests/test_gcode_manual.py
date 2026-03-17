"""
Test: draw a gcode file without Vicon
======================================
Stand Spot in front of your paper, measure where the paper's top-left corner
is relative to Spot's starting position, then run this script.

The odom frame origin is roughly where Spot's centre was when it powered on,
at floor level (z ≈ 0).  A piece of paper flat on the floor 80 cm ahead of
Spot would be --origin 0.8 0.0 0.0.

Usage
-----
    python -m tests.test_gcode_manual my_drawing.gcode \\
        --origin 0.8 0.0 0.0 \\
        --x-axis 1.0 0.0 0.0 \\
        --canvas-z 0.0 \\
        --canvas-size 200 150

Arguments
---------
    gcode           Path to .gcode file (G00/G01/G02/G03)
    --origin X Y Z  Odom-frame position of gcode (0,0) in metres.
                    Put the paper's top-left corner here.  Default: 0.8 0.0 0.0
    --x-axis X Y Z  Unit vector for gcode +X direction (odom frame).
                    Default: 1.0 0.0 0.0  (pointing straight ahead of Spot)
    --canvas-z Z    Drawing surface height in odom frame (m).  Default: 0.0
    --canvas-size W H  Canvas width and height in mm — only used for the
                    pre-flight bounds check.  Optional.
    --scale S       Metres per gcode unit.  Default 0.001 (gcode in mm).
    --tool-length L Marker/tool length beyond wrist (m).  Default 0.0.

Controls
--------
    SPACE   — pause / resume
    RETURN  — emergency stop (stow arm + sit)

Tips for placing the origin without Vicon
------------------------------------------
1. Power on Spot and let it stand.  The odom frame origin is now at Spot's
   starting position on the floor.
2. Measure (tape measure) how far ahead and to the side your paper's top-left
   corner is from where Spot is standing.
3. Pass those values as --origin <forward> <left_positive> 0.0
   (odom +X = Spot's forward at power-on, +Y = Spot's left).
4. If gcode X should point to Spot's right, use --x-axis 0 -1 0.
   If gcode X points straight ahead, leave --x-axis as default.
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

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config.robot_config import PASSWORD, ROBOT_IP, USERNAME  # noqa: E402

import draw_autonomous._ctrl as _ctrl
from draw_autonomous.robot.gcode_draw import draw_gcode_manual


def run(robot, args):
    lease_client  = robot.ensure_client(LeaseClient.default_service_name)
    cmd_client    = robot.ensure_client(RobotCommandClient.default_service_name)
    state_client  = robot.ensure_client(RobotStateClient.default_service_name)
    estop_client  = robot.ensure_client(EstopClient.default_service_name)
    asc_client    = robot.ensure_client(ArmSurfaceContactClient.default_service_name)

    estop_ep = EstopEndpoint(estop_client, name="test_gcode_manual_estop", estop_timeout=9.0)
    estop_ep.force_simple_setup()

    with (
        EstopKeepAlive(estop_ep),
        LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True),
    ):
        robot.power_on(timeout_sec=20)
        blocking_stand(cmd_client, timeout_sec=10)
        print("Robot standing.\n")
        print("  SPACE = pause / resume     RETURN = emergency stop\n")

        _ctrl.start_key_listener()

        try:
            draw_gcode_manual(
                asc_client, cmd_client, state_client,
                gcode_path=args.gcode,
                scale=args.scale,
                tool_length=args.tool_length,
                origin_x=args.origin[0],
                origin_y=args.origin[1],
                origin_z=args.origin[2],
                x_axis=args.x_axis,
                canvas_z=args.canvas_z if args.canvas_z is not None else args.origin[2],
                canvas_width_mm=args.canvas_size[0] if args.canvas_size else None,
                canvas_height_mm=args.canvas_size[1] if args.canvas_size else None,
            )
        except Exception as e:
            print(f"\n  [ERROR] {e}")

        print("\nStowing arm and sitting…")
        try:
            cmd_client.robot_command(RobotCommandBuilder.arm_stow_command())
            time.sleep(2)
            cmd_client.robot_command(RobotCommandBuilder.synchro_sit_command())
            time.sleep(2)
        except Exception:
            pass
        print("Done.")


def main():
    ap = argparse.ArgumentParser(
        description="Draw a gcode file without Vicon using manually measured canvas position")
    ap.add_argument("gcode", help="Path to .gcode file")
    ap.add_argument("--origin", type=float, nargs=3, metavar=("X", "Y", "Z"),
                    default=[0.8, 0.0, 0.0],
                    help="Odom-frame position of gcode (0,0) in metres (default: 0.8 0.0 0.0)")
    ap.add_argument("--x-axis", type=float, nargs=3, metavar=("X", "Y", "Z"),
                    default=[1.0, 0.0, 0.0],
                    help="Gcode +X direction in odom frame (default: 1.0 0.0 0.0)")
    ap.add_argument("--canvas-z", type=float, default=None,
                    help="Drawing surface height in odom frame (m). Defaults to origin Z.")
    ap.add_argument("--canvas-size", type=float, nargs=2, metavar=("W_MM", "H_MM"),
                    default=None,
                    help="Canvas width and height in mm for bounds check (optional)")
    ap.add_argument("--scale", type=float, default=0.001,
                    help="Metres per gcode unit (default 0.001 = gcode in mm)")
    ap.add_argument("--tool-length", type=float, default=0.0,
                    help="Marker/tool length beyond wrist in metres (default 0.0)")
    args = ap.parse_args()

    if not os.path.exists(args.gcode):
        raise SystemExit(f"Gcode file not found: {args.gcode}")

    print(f"Gcode     : {args.gcode}")
    print(f"Origin    : {args.origin} m  (gcode 0,0 in odom frame)")
    print(f"X-axis    : {args.x_axis}")
    print(f"Canvas Z  : {args.canvas_z if args.canvas_z is not None else args.origin[2]} m")
    print(f"Scale     : {args.scale}  ({1/args.scale:.0f} gcode units per metre)")
    if args.canvas_size:
        print(f"Canvas    : {args.canvas_size[0]:.0f} x {args.canvas_size[1]:.0f} mm")
    print()

    bosdyn.client.util.setup_logging(verbose=False)
    sdk   = bosdyn.client.create_standard_sdk("TestGcodeManual")
    robot = sdk.create_robot(ROBOT_IP)
    robot.authenticate(USERNAME, PASSWORD)
    robot.time_sync.wait_for_sync()
    print("Spot connected.\n")

    run(robot, args)


if __name__ == "__main__":
    main()
