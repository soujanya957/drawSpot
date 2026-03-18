"""
Test: draw_gcode pipeline — layered validation
===============================================
Run stages in order to validate each piece before the full pipeline.

Stages
------
  vicon    Verify Vicon data for all subjects — no Spot required
  nav      Walk Spot to canvas standoff and face canvas
  marker   Open gripper, wait for marker placement, close gripper
  draw     Execute a gcode file (arm assumed already near canvas — skips nav)
  full     Complete pipeline: nav → marker → draw

Usage
-----
    python -m tests.test_draw_gcode --stage vicon  --vicon 169.254.217.218:801
    python -m tests.test_draw_gcode --stage nav    --vicon 169.254.217.218:801
    python -m tests.test_draw_gcode --stage marker --vicon 169.254.217.218:801
    python -m tests.test_draw_gcode --stage draw   --vicon 169.254.217.218:801
    python -m tests.test_draw_gcode --stage full   --vicon 169.254.217.218:801

Controls during robot stages
-----------------------------
    SPACE   — emergency stop (stow + sit immediately)
    ENTER   — confirm marker placement
    x       — sit and exit after a stage completes
"""

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config.robot_config import PASSWORD, ROBOT_IP, USERNAME, VICON_ADDRESS  # noqa: E402
from vicon.client import MockViconClient, ViconClient                        # noqa: E402
from draw_gcode.main import G_CODES_DIR, select_gcode                       # noqa: E402

VICON_CONNECT_TO = 6.0
STAGES = ("vicon", "nav", "marker", "draw", "full")

SEP  = "─" * 52
SEP2 = "━" * 52


# ---------------------------------------------------------------------------
# Stage 0 — Vicon only
# ---------------------------------------------------------------------------

def stage_vicon(vicon_client) -> None:
    frame = vicon_client.latest_frame
    if frame is None:
        print("  FAIL — no Vicon frame received")
        return

    print(f"  timestamp  {frame.timestamp:.3f}")
    print()

    # spot_base
    b = frame.spot_body
    if b is None or b.occluded:
        print("  spot_base  NO DATA")
    else:
        px, py, pz = b.position
        print(f"  spot_base  pos  ({px:+8.1f}, {py:+8.1f}, {pz:+8.1f}) mm")
        qx, qy, qz, qw = b.rotation_quat
        print(f"             rot  qx={qx:.3f}  qy={qy:.3f}  qz={qz:.3f}  qw={qw:.3f}")
        print(f"             markers  {len(b.markers)}")
    print()

    # spot_ee
    e = frame.spot_ee
    if e is None or e.occluded:
        print("  spot_ee    NO DATA")
    else:
        px, py, pz = e.position
        print(f"  spot_ee    pos  ({px:+8.1f}, {py:+8.1f}, {pz:+8.1f}) mm")
    print()

    # canvas
    c = frame.canvas
    if c is None or not c.is_valid():
        print("  canvas     NO DATA  (check test_canvas markers in Vicon Tracker)")
    else:
        print(f"  canvas     {c.width_mm:.1f} mm × {c.height_mm:.1f} mm")
        labels = ("TL", "TR", "BR", "BL")
        for label, corner in zip(labels, c.corners):
            cx, cy, cz = corner
            print(f"             {label}  ({cx:+8.1f}, {cy:+8.1f}, {cz:+8.1f}) mm")
        xx, xy, xz = c.x_axis
        yx, yy, yz = c.y_axis
        nx, ny, nz = c.normal
        print(f"             x_axis  ({xx:.3f}, {xy:.3f}, {xz:.3f})")
        print(f"             y_axis  ({yx:.3f}, {yy:.3f}, {yz:.3f})")
        print(f"             normal  ({nx:.3f}, {ny:.3f}, {nz:.3f})")
        dot = float(np.dot(c.x_axis, c.y_axis))
        ok  = "\033[92mOK\033[0m" if abs(dot) < 0.01 else "\033[91mWARN\033[0m"
        print(f"             axes orthogonal?  dot(x,y) = {dot:.4f}  {ok}")
    print()


# ---------------------------------------------------------------------------
# Spot helpers
# ---------------------------------------------------------------------------

def _connect_spot():
    import bosdyn.client
    import bosdyn.client.util
    bosdyn.client.util.setup_logging(verbose=False)
    sdk   = bosdyn.client.create_standard_sdk("TestDrawGcode")
    robot = sdk.create_robot(ROBOT_IP)
    robot.authenticate(USERNAME, PASSWORD)
    robot.time_sync.wait_for_sync()
    return robot


def _get_clients(robot):
    from bosdyn.client.arm_surface_contact import ArmSurfaceContactClient
    from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
    from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
    from bosdyn.client.robot_command import RobotCommandClient
    from bosdyn.client.robot_state import RobotStateClient

    lc  = robot.ensure_client(LeaseClient.default_service_name)
    cc  = robot.ensure_client(RobotCommandClient.default_service_name)
    sc  = robot.ensure_client(RobotStateClient.default_service_name)
    ac  = robot.ensure_client(ArmSurfaceContactClient.default_service_name)
    ec  = robot.ensure_client(EstopClient.default_service_name)

    ep = EstopEndpoint(ec, name="test_draw_gcode_estop", estop_timeout=9.0)
    ep.force_simple_setup()
    return lc, cc, sc, ac, EstopKeepAlive(ep), LeaseKeepAlive(lc, must_acquire=True, return_at_exit=True)


def _stand(robot, cmd_client) -> None:
    from bosdyn.client.robot_command import blocking_stand
    import draw_gcode._ctrl as _ctrl
    robot.power_on(timeout_sec=20)
    blocking_stand(cmd_client, timeout_sec=10)
    print("  Robot standing.")
    print()
    print("  SPACE = emergency stop  |  ENTER = confirm  |  x = sit and exit")
    print()
    _ctrl.start_key_listener()


def _stow(cmd_client) -> None:
    """Stow arm only — robot stays standing."""
    from bosdyn.client.robot_command import RobotCommandBuilder
    print()
    print("  Stowing arm…")
    try:
        cmd_client.robot_command(RobotCommandBuilder.arm_stow_command())
        time.sleep(4)   # stow from extended position takes ~3-4 s
    except Exception:
        pass
    print("  Arm stowed. Robot still standing.")


def _sit_and_exit(cmd_client) -> None:
    """Sit the robot and exit."""
    from bosdyn.client.robot_command import RobotCommandBuilder
    print("  Sitting…")
    try:
        cmd_client.robot_command(RobotCommandBuilder.synchro_sit_command())
        time.sleep(2)
    except Exception:
        pass
    print("  Done.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(
        description="Layered tests for draw_gcode pipeline",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--stage", choices=STAGES, required=True,
                    help="Pipeline stage to test")
    ap.add_argument("--vicon", metavar="HOST:PORT",
                    help=f"Vicon address (default from .env: {VICON_ADDRESS})")
    ap.add_argument("--mock", action="store_true",
                    help="Use simulated Vicon client (no hardware)")
    ap.add_argument("--margin", type=float, default=0.90,
                    help="Canvas fill fraction for auto-scale")
    args = ap.parse_args()

    # ── Vicon ─────────────────────────────────────────────────────────────
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
        raise SystemExit(f"No Vicon frames after {VICON_CONNECT_TO:.0f} s.")

    print("Vicon connected.")
    print()

    # ── Stage: vicon ──────────────────────────────────────────────────────
    if args.stage == "vicon":
        print(SEP)
        print("  Vicon data snapshot")
        print(SEP)
        stage_vicon(vicon)
        vicon.stop()
        return

    # ── All other stages need Spot ─────────────────────────────────────────
    print("Connecting to Spot…")
    robot = _connect_spot()
    print("Spot connected.")
    print()

    import draw_gcode._ctrl as _ctrl
    from draw_gcode.robot.navigator import walk_to_canvas, open_for_marker, move_to_draw_pose
    from draw_gcode.main import _draw_file, compute_scale, _load_cfg

    # Select gcode before _stand() so input() works (key listener not yet running)
    gcode_path = None
    scale = None
    if args.stage in ("draw", "full"):
        frame = vicon.latest_frame
        if frame is None or frame.canvas is None or not frame.canvas.is_valid():
            vicon.stop()
            raise SystemExit("FAIL — canvas not visible in Vicon")
        c = frame.canvas
        print(f"  Canvas  {c.width_mm:.0f} mm × {c.height_mm:.0f} mm")
        os.makedirs(G_CODES_DIR, exist_ok=True)
        gcode_path = select_gcode()
        print("  Computing auto-scale…")
        scale = compute_scale(gcode_path, c, margin=args.margin)
        print()

    lc, cmd_client, sc, asc_client, estop_ka, lease_ka = _get_clients(robot)

    try:
        with estop_ka, lease_ka:
            _stand(robot, cmd_client)

            print(SEP)
            print(f"  Stage: {args.stage}")
            print(SEP)

            if args.stage == "nav":
                ok = walk_to_canvas(cmd_client, vicon)
                print()
                print(f"  Result: {'OK — at canvas standoff' if ok else 'STOPPED'}")

            elif args.stage == "marker":
                ok = open_for_marker(cmd_client)
                print()
                print(f"  Result: {'OK — marker acquired' if ok else 'STOPPED'}")

            elif args.stage in ("draw", "full"):
                cfg = _load_cfg()
                if args.stage == "full":
                    ok = walk_to_canvas(cmd_client, vicon)
                    if ok and _ctrl.check():
                        ok = open_for_marker(cmd_client)
                    if ok and _ctrl.check():
                        _draw_file(gcode_path, scale, vicon, sc, cmd_client, asc_client,
                                   robot.logger, cfg)
                else:
                    # draw only — arm assumed already at canvas
                    _draw_file(gcode_path, scale, vicon, sc, cmd_client, asc_client,
                               robot.logger, cfg)

            # ── Stage done: stow arm, wait for x to sit ────────────────────
            _stow(cmd_client)
            _ctrl.wait_finish("Stage complete. Press x to sit and exit.")
            _sit_and_exit(cmd_client)

    finally:
        vicon.stop()


if __name__ == "__main__":
    main()
