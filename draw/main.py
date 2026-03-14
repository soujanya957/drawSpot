"""
draw/main.py — Spot Vicon Canvas Draw
======================================
Entry point that wires together the three independent components:

  1. Vicon client   — streams 6-DoF pose + marker data from the capture system.
  2. Robot controller — translates canvas UV commands into Spot arm motions.
  3. Web UI server  — serves the real-time canvas interface on port 8080.

Usage
-----
Run in simulation mode (no hardware required):
    python -m draw.main

Connect to a real Vicon system:
    python -m draw.main --vicon 192.168.1.100:801

Also control a real Spot robot:
    python -m draw.main --vicon 192.168.1.100:801 --real-robot

Additional options:
    --port 8080          Web UI port (default: 8080)
    --host 0.0.0.0       Web UI bind address
    --autonomous         Start in AUTONOMOUS mode immediately (default: IDLE)
    --log-level debug    Logging verbosity

Keyboard shortcut in the browser:
    Space     — E-Stop
    t         — Takeover (AUTONOMOUS → TELEOP)
    r         — Release  (TELEOP → AUTONOMOUS)
    Escape    — Reset E-Stop → IDLE
    c         — Clear canvas
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Spot Vicon Canvas Draw",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "--vicon", metavar="HOST:PORT", default=None,
        help="Vicon DataStream host:port.  Omit to run the built-in simulator.",
    )
    p.add_argument(
        "--real-robot", action="store_true",
        help="Connect to a real Spot robot (requires bosdyn SDK and config/robot_config.py).",
    )
    p.add_argument("--host",       default="0.0.0.0",     help="Web UI bind address.")
    p.add_argument("--port",       default=8080, type=int, help="Web UI port.")
    p.add_argument("--autonomous", action="store_true",    help="Start in AUTONOMOUS mode.")
    p.add_argument(
        "--log-level", default="info",
        choices=["debug", "info", "warning", "error"],
        help="Logging verbosity.",
    )
    return p


def main() -> None:
    args = _build_parser().parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s  %(levelname)-8s  %(name)s — %(message)s",
        datefmt="%H:%M:%S",
    )
    logger = logging.getLogger("draw.main")

    # ------------------------------------------------------------------
    # 1. Vicon client
    # ------------------------------------------------------------------
    if args.vicon:
        from draw.vicon.client import ViconClient
        vicon = ViconClient(host=args.vicon)
        logger.info("Using real Vicon client → %s", args.vicon)
    else:
        from draw.vicon.client import MockViconClient
        vicon = MockViconClient()
        logger.info("Using mock Vicon client (simulation mode).")

    # ------------------------------------------------------------------
    # 2. Robot controller
    # ------------------------------------------------------------------
    from draw.robot.controller import SpotController
    controller = SpotController(simulation=not args.real_robot)

    if args.real_robot:
        logger.info("Real robot mode — will connect to Spot.")
    else:
        logger.info("Simulation mode — no Spot hardware needed.")

    # ------------------------------------------------------------------
    # 3. Web UI server
    # ------------------------------------------------------------------
    from draw.ui.server import UIServer
    ui = UIServer(controller, host=args.host, port=args.port)

    # ------------------------------------------------------------------
    # Wire Vicon frames → controller
    # ------------------------------------------------------------------
    import threading

    def vicon_dispatch_loop() -> None:
        """Forward every new Vicon frame to the controller."""
        last_ts: float = 0.0
        while _running:
            frame = vicon.latest_frame
            if frame is not None and frame.timestamp != last_ts:
                last_ts = frame.timestamp
                controller.update_vicon(frame)
            time.sleep(0.005)   # poll at 200 Hz max (Vicon typically 100 Hz)

    # ------------------------------------------------------------------
    # Start everything
    # ------------------------------------------------------------------
    _running = True

    vicon.start()
    ui.start_background()

    dispatch_thread = threading.Thread(
        target=vicon_dispatch_loop, daemon=True, name="ViconDispatch"
    )
    dispatch_thread.start()

    if args.autonomous:
        ok = controller.start_autonomous()
        logger.info("Started in AUTONOMOUS mode: %s", ok)

    logger.info("━" * 60)
    logger.info("  Web UI:  http://localhost:%d", args.port)
    logger.info("  Mode:    %s", controller.mode.name)
    logger.info("  Press Ctrl-C to quit.")
    logger.info("━" * 60)

    # ------------------------------------------------------------------
    # Wait for Ctrl-C / SIGTERM
    # ------------------------------------------------------------------
    def _shutdown(sig, frame) -> None:
        nonlocal _running
        logger.info("\n[main] Shutdown signal received.")
        _running = False

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        while _running:
            time.sleep(0.2)
    finally:
        logger.info("[main] Stopping Vicon client…")
        vicon.stop()
        logger.info("[main] Done. Goodbye.")


if __name__ == "__main__":
    main()
