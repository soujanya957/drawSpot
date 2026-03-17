"""
draw_gcode/main.py — Autonomous walk + manual marker + gcode drawing
=====================================================================
Pipeline
--------
  1. Connect Vicon  — verify canvas is visible
  2. Select gcode   — numbered menu from draw_gcode/g_codes/
  3. Auto-scale     — fit drawing within canvas (90% fill, aspect-ratio preserved)
  4. Connect Spot   — authenticate + time-sync
  5. Stand up       — power on, stand
  6. Walk to canvas — approach TL/TR edge, rotate to face canvas (+X toward canvas)
  7. Place marker   — extend arm, open gripper, wait for ENTER, close gripper
  8. Draw pose      — raise arm above canvas centre
  9. Draw gcode     — execute file via ArmSurfaceContact with Vicon canvas origin

Controls (during robot operation)
----------------------------------
  SPACE   — emergency stop  (stows arm + sits immediately)
  ENTER   — confirm marker placement (step 7 only)

Usage
-----
    python -m draw_gcode.main --vicon 169.254.217.218:801
    python -m draw_gcode.main --vicon 169.254.217.218:801 --repeats 2
    python -m draw_gcode.main --vicon 169.254.217.218:801 --tool-length 0.15
"""

import argparse
import glob
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
from config.robot_config import PASSWORD, ROBOT_IP, USERNAME, VICON_ADDRESS  # noqa: E402
from vicon.client import ViconClient                                          # noqa: E402

import draw_gcode._ctrl as _ctrl
from draw_gcode.robot.navigator import walk_to_canvas, open_for_marker, move_to_draw_pose
from draw_gcode.robot.gcode_exec import execute as gcode_execute, compute_canvas_scale

VICON_CONNECT_TO = 6.0
G_CODES_DIR = os.path.join(os.path.dirname(__file__), "g_codes")


# ---------------------------------------------------------------------------
# Gcode selection
# ---------------------------------------------------------------------------

def select_gcode() -> str:
    """
    Present a numbered menu of .gcode / .nc files from draw_gcode/g_codes/.
    Uses normal input() — call this BEFORE starting the key listener.
    Returns the selected file path.
    """
    patterns = ("*.gcode", "*.nc", "*.ngc")
    files = []
    for pat in patterns:
        files.extend(glob.glob(os.path.join(G_CODES_DIR, pat)))
    files = sorted(set(files))

    if not files:
        raise SystemExit(
            f"\nNo gcode files found in  {os.path.relpath(G_CODES_DIR)}/\n"
            f"Add .gcode / .nc files there and re-run.\n"
        )

    print(f"\nGcode files available in  {os.path.relpath(G_CODES_DIR)}/")
    print(f"{'─' * 50}")
    for i, f in enumerate(files, 1):
        name = os.path.basename(f)
        kb   = os.path.getsize(f) / 1024
        print(f"  [{i}]  {name}  ({kb:.1f} KB)")
    print(f"{'─' * 50}")

    while True:
        try:
            raw = input(f"Select file [1–{len(files)}]: ").strip()
            idx = int(raw) - 1
            if 0 <= idx < len(files):
                chosen = files[idx]
                print(f"  → {os.path.basename(chosen)}\n")
                return chosen
        except (ValueError, EOFError):
            pass
        print(f"  Enter a number between 1 and {len(files)}.")


# ---------------------------------------------------------------------------
# Robot pipeline
# ---------------------------------------------------------------------------

def run(vicon_client, robot, gcode_path: str, tool_length: float,
        repeats: int, precomputed_scale: float) -> None:

    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    cmd_client   = robot.ensure_client(RobotCommandClient.default_service_name)
    state_client = robot.ensure_client(RobotStateClient.default_service_name)
    estop_client = robot.ensure_client(EstopClient.default_service_name)
    asc_client   = robot.ensure_client(ArmSurfaceContactClient.default_service_name)

    estop_ep = EstopEndpoint(estop_client, name="draw_gcode_estop", estop_timeout=9.0)
    estop_ep.force_simple_setup()

    with (
        EstopKeepAlive(estop_ep),
        LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True),
    ):
        # ── 1. Stand up ───────────────────────────────────────────────────
        print("Standing up…")
        robot.power_on(timeout_sec=20)
        blocking_stand(cmd_client, timeout_sec=10)
        print("Robot standing.\n")
        print(f"  SPACE = emergency stop (stow + sit)")
        print(f"  ENTER = confirm marker placement\n")

        _ctrl.start_key_listener()

        for rep in range(repeats):
            if not _ctrl.check():
                break

            if repeats > 1:
                print(f"\n{'━' * 52}")
                print(f"  Repetition {rep + 1} / {repeats}")
                print(f"{'━' * 52}\n")

            # ── 2. Walk to canvas ─────────────────────────────────────────
            print("─── Step 1 / 4  Walk to canvas")
            if not walk_to_canvas(cmd_client, vicon_client):
                break
            if not _ctrl.check():
                break

            # ── 3. Place marker ───────────────────────────────────────────
            print("\n─── Step 2 / 4  Marker placement")
            if not open_for_marker(cmd_client):
                break
            if not _ctrl.check():
                break

            # ── 4. Draw pose ──────────────────────────────────────────────
            print("\n─── Step 3 / 4  Draw pose")
            if not move_to_draw_pose(cmd_client, vicon_client):
                break
            if not _ctrl.check():
                break

            # ── 5. Draw ───────────────────────────────────────────────────
            print("\n─── Step 4 / 4  Drawing")
            try:
                gcode_execute(
                    vicon_client, asc_client, cmd_client, state_client,
                    gcode_path,
                    scale=precomputed_scale,
                    tool_length=tool_length,
                )
            except RuntimeError as exc:
                print(f"\n  [ERROR] {exc}")
                break

        # ── Clean exit ────────────────────────────────────────────────────
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
        description="draw_gcode — autonomous walk + manual marker + gcode drawing",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--vicon", metavar="HOST:PORT",
                    help=f"Vicon address (default: from .env = {VICON_ADDRESS})")
    ap.add_argument("--tool-length", type=float, default=0.0,
                    help="Marker/brush length beyond the wrist (metres)")
    ap.add_argument("--repeats", type=int, default=1,
                    help="Number of full draw cycles")
    ap.add_argument("--margin", type=float, default=0.90,
                    help="Canvas fill fraction for auto-scale (0–1)")
    args = ap.parse_args()

    vicon_addr = args.vicon or VICON_ADDRESS
    if not vicon_addr:
        raise SystemExit("No VICON_HOST set and --vicon not specified.")

    # ── Connect Vicon first (needed to preview auto-scale) ────────────────
    print(f"Connecting to Vicon at {vicon_addr}…")
    vicon = ViconClient(host=vicon_addr)
    vicon.start()
    deadline = time.time() + VICON_CONNECT_TO
    while vicon.latest_frame is None and time.time() < deadline:
        time.sleep(0.05)
    if vicon.latest_frame is None:
        raise SystemExit(f"No Vicon frames after {VICON_CONNECT_TO:.0f} s.")

    frame = vicon.latest_frame
    canvas_ok = frame.canvas is not None and frame.canvas.is_valid()
    spot_ok   = frame.spot_body is not None and not frame.spot_body.occluded

    print(f"Vicon OK  |  canvas: {'OK' if canvas_ok else 'NO DATA'}"
          f"  |  spot_base: {'OK' if spot_ok else 'NO DATA'}")

    if not canvas_ok:
        vicon.stop()
        raise SystemExit("Canvas not visible — check test_canvas markers in Vicon Tracker.")
    if not spot_ok:
        print("[WARN] spot_base not visible — navigation will wait for it.\n")

    # ── Select gcode file (uses normal input(), before key listener) ───────
    os.makedirs(G_CODES_DIR, exist_ok=True)
    gcode_path = select_gcode()

    # ── Pre-compute scale from canvas now (shows preview before Spot connect) ─
    print("Computing auto-scale…")
    scale = compute_canvas_scale(gcode_path, frame.canvas, margin=args.margin)
    print()

    # ── Connect Spot ──────────────────────────────────────────────────────
    print("Connecting to Spot…")
    bosdyn.client.util.setup_logging(verbose=False)
    sdk   = bosdyn.client.create_standard_sdk("DrawGcode")
    robot = sdk.create_robot(ROBOT_IP)
    robot.authenticate(USERNAME, PASSWORD)
    robot.time_sync.wait_for_sync()
    print("Spot connected.\n")

    try:
        run(vicon, robot, gcode_path, args.tool_length, args.repeats, scale)
    finally:
        vicon.stop()


if __name__ == "__main__":
    main()
