"""
Test: Vicon connection and live data display
============================================
Connects to Vicon and prints a live-refreshing status for all tracked
subjects.  Run this before anything else to verify Vicon is working and
all subjects are named correctly in Tracker.

Subjects expected on the Vicon server:
    "Spot"      → Spot body       (rigid body, segment = "Spot")
    "SpotEE"    → end-effector    (rigid body, segment = "SpotEE")
    "BrushTip"  → brush tip       (single marker, subject = "BrushTip")
    "Canvas"    → canvas plane    (4 markers: TL, TR, BR, BL)

Usage:
    python -m tests.test_vicon                          # all subjects, live
    python -m tests.test_vicon --target spot_body       # Spot body only
    python -m tests.test_vicon --target spot_ee         # end-effector only
    python -m tests.test_vicon --target brush_tip       # brush marker only
    python -m tests.test_vicon --target canvas          # canvas corners + axes
    python -m tests.test_vicon --mock                   # no hardware (simulated)
    python -m tests.test_vicon --once                   # print one frame and exit
    python -m tests.test_vicon --rate 2                 # refresh every 2 s (default 1)

Exit:  Ctrl+C
"""

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config.robot_config import VICON_ADDRESS          # noqa: E402
from vicon.client import MockViconClient, ViconClient  # noqa: E402

VICON_CONNECT_TO = 5.0
TARGETS = ("all", "spot_body", "spot_ee", "brush_tip", "canvas")

# ---------------------------------------------------------------------------
# Formatting helpers
# ---------------------------------------------------------------------------

def _pos(p) -> str:
    return f"({p[0]:9.2f}, {p[1]:9.2f}, {p[2]:9.2f}) mm"

def _vec(v) -> str:
    return f"({v[0]:6.3f}, {v[1]:6.3f}, {v[2]:6.3f})"

def _quat(q) -> str:
    return f"qx={q[0]:6.3f}  qy={q[1]:6.3f}  qz={q[2]:6.3f}  qw={q[3]:6.3f}"

def _occ(rb) -> str:
    return "\033[91mOCCLUDED\033[0m" if rb.occluded else "\033[92mvisible\033[0m"

def _missing(subject_name) -> str:
    return f"  \033[90mNO DATA  — subject '{subject_name}' not seen by Vicon\033[0m"


# ---------------------------------------------------------------------------
# Per-subject printers
# ---------------------------------------------------------------------------

def _block_spot_body(frame) -> list:
    lines = ["\033[1m── Spot body  (subject: 'Spot')\033[0m"]
    b = frame.spot_body
    if b is None:
        lines.append(_missing("Spot"))
    else:
        lines.append(f"  pos    {_pos(b.position)}   [{_occ(b)}]")
        lines.append(f"  rot    {_quat(b.rotation_quat)}")
        if b.markers:
            lines.append(f"  markers ({len(b.markers)}):")
            for m in b.markers:
                occ = " \033[91m[occ]\033[0m" if m.occluded else ""
                lines.append(f"    {m.name:<6}  {_pos(m.position)}{occ}")
    return lines


def _block_spot_ee(frame) -> list:
    lines = ["\033[1m── SpotEE  (subject: 'SpotEE')\033[0m"]
    e = frame.spot_ee
    if e is None:
        lines.append(_missing("SpotEE"))
    else:
        lines.append(f"  pos    {_pos(e.position)}   [{_occ(e)}]")
        lines.append(f"  rot    {_quat(e.rotation_quat)}")
    return lines


def _block_brush_tip(frame) -> list:
    lines = ["\033[1m── BrushTip  (subject: 'BrushTip', single marker)\033[0m"]
    t = frame.brush_tip
    if t is None:
        lines.append(_missing("BrushTip"))
    else:
        occ = f"   [{_occ(t)}]"
        lines.append(f"  pos    {_pos(t.position)}{occ}")
    return lines


def _block_canvas(frame) -> list:
    lines = ["\033[1m── Canvas  (subject: 'Canvas', markers: TL TR BR BL)\033[0m"]
    c = frame.canvas
    if c is None:
        lines.append(_missing("Canvas"))
        lines.append("  \033[90m  (ensure TL, TR, BR, BL markers are labelled in Vicon Tracker)\033[0m")
    else:
        valid = "\033[92mYES\033[0m" if c.is_valid() else "\033[91mNO\033[0m"
        lines.append(f"  valid   {valid}")
        lines.append(f"  size    {c.width_mm:.1f} mm  ×  {c.height_mm:.1f} mm")
        lines.append("  corners:")
        for label, corner in zip(("TL", "TR", "BR", "BL"), c.corners):
            lines.append(f"    {label}   {_pos(corner)}")
        lines.append(f"  x_axis  {_vec(c.x_axis)}  (TL → TR)")
        lines.append(f"  y_axis  {_vec(c.y_axis)}  (TL → BL)")
        lines.append(f"  normal  {_vec(c.normal)}  (out of canvas surface)")
        # Orthogonality check
        dot_xy = float(np.dot(c.x_axis, c.y_axis))
        dot_xn = float(np.dot(c.x_axis, c.normal))
        status = "\033[92mOK\033[0m" if abs(dot_xy) < 0.01 else "\033[91mWARN\033[0m"
        lines.append(f"  axes orthogonal? dot(x,y)={dot_xy:.4f}  {status}")
    return lines


# ---------------------------------------------------------------------------
# Composite frame printer
# ---------------------------------------------------------------------------

BLOCK_FNS = {
    "spot_body": _block_spot_body,
    "spot_ee":   _block_spot_ee,
    "brush_tip": _block_brush_tip,
    "canvas":    _block_canvas,
}

def print_frame(frame, target: str, frame_count: int) -> None:
    print("\033[2J\033[H", end="")   # clear screen, cursor home
    print(f"  \033[1mVicon live data\033[0m"
          f"  —  frame #{frame_count}"
          f"  —  ts {frame.timestamp:.3f}"
          f"  —  Ctrl+C to exit\n")

    subjects = list(BLOCK_FNS.keys()) if target == "all" else [target]
    for subject in subjects:
        for line in BLOCK_FNS[subject](frame):
            print(line)
        print()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description="Vicon connection and live data test")
    ap.add_argument("--target", choices=TARGETS, default="all",
                    help="Which subject to display (default: all)")
    ap.add_argument("--mock",  action="store_true",
                    help="Use simulated Vicon client (no hardware)")
    ap.add_argument("--once",  action="store_true",
                    help="Print one frame and exit")
    ap.add_argument("--rate",  type=float, default=1.0,
                    help="Refresh rate in seconds (default 1.0)")
    ap.add_argument("--vicon", metavar="HOST:PORT",
                    help=f"Vicon address (default: from .env = {VICON_ADDRESS})")
    args = ap.parse_args()

    # ── Connect ──────────────────────────────────────────────────────────
    vicon_addr = args.vicon or VICON_ADDRESS
    if args.mock:
        print("Using mock Vicon client (no hardware).")
        vicon = MockViconClient()
    elif vicon_addr:
        print(f"Connecting to Vicon at {vicon_addr}…")
        vicon = ViconClient(host=vicon_addr)
    else:
        raise SystemExit("No VICON_HOST set and --mock not specified.\n"
                         "Set VICON_HOST in .env or pass --vicon HOST:PORT")

    vicon.start()

    # Wait for first frame
    deadline = time.time() + VICON_CONNECT_TO
    while vicon.latest_frame is None and time.time() < deadline:
        time.sleep(0.05)

    if vicon.latest_frame is None:
        vicon.stop()
        raise SystemExit(f"No Vicon frames received after {VICON_CONNECT_TO:.0f} s.\n"
                         "Check Vicon Tracker is running, streaming is enabled, "
                         "and VICON_HOST is correct.")

    print(f"Connected.  Showing: {args.target}\n")

    # ── Display loop ─────────────────────────────────────────────────────
    frame_count = 0
    try:
        while True:
            frame = vicon.latest_frame
            if frame:
                frame_count += 1
                print_frame(frame, args.target, frame_count)
                if args.once:
                    break
            time.sleep(args.rate)
    except KeyboardInterrupt:
        pass
    finally:
        vicon.stop()
        print("\nStopped.")


if __name__ == "__main__":
    main()
