"""
Spot Robot Controller
======================
Translates high-level canvas UV commands into Spot arm motions.

Operates in two modes:
  simulation=True  (default) — no real robot required.  All commands are
                               logged; internal state is updated directly.
  simulation=False           — connects to a real Spot using the credentials
                               in config/robot_config.py and sends arm
                               Cartesian commands in the "odom" frame.

Public API
----------
  move_brush(u, v, pen_down)   — move brush to canvas UV position [0..1]²
  emergency_stop()             — activate E-Stop
  emergency_takeover()         — hand control to operator (AUTONOMOUS → TELEOP)
  release_teleop()             — return from TELEOP to AUTONOMOUS
  reset()                      — clear E-Stop → IDLE
  update_vicon(frame)          — called by the Vicon thread with each new frame
  brush_uv                     — (u, v) property: current brush position
  pen_down                     — bool property: brush touching canvas?
  mode                         — RobotMode property: current state machine mode
  pop_new_strokes()            — consume buffered stroke events for the web UI
  clear_canvas()               — empty the stroke buffer and fire a clear event

Thread safety
-------------
All public methods are safe to call from any thread.
"""

from __future__ import annotations

import logging
import math
import sys
import threading
from typing import List, Optional, Tuple

import numpy as np

from .state import RobotMode, RobotStateMachine
from ..vicon.types import ViconFrame
from ..vicon.transform import canvas_to_world, clamp_uv, world_to_canvas

logger = logging.getLogger(__name__)


class SpotController:
    # Height of brush tip above the canvas surface (mm).
    DRAW_HEIGHT_MM = 2.0  # pen touching
    LIFT_HEIGHT_MM = 30.0  # pen raised

    # Only re-send an arm command if the UV position changed by this much.
    _MOVE_THRESHOLD_UV = 0.002  # ~0.8 mm on a 400 mm canvas

    def __init__(self, simulation: bool = True) -> None:
        self.simulation = simulation

        self.state = RobotStateMachine()

        # Current brush position & pen state (guarded by _brush_lock)
        self._brush_uv: Tuple[float, float] = (0.5, 0.5)
        self._pen_down: bool = False
        self._brush_lock = threading.Lock()

        # Latest Vicon frame (guarded by _frame_lock)
        self._latest_frame: Optional[ViconFrame] = None
        self._frame_lock = threading.Lock()

        # Stroke event buffer for the web UI (guarded by _strokes_lock)
        # Each item: {"u": float, "v": float, "pen_down": bool}
        self._strokes: List[dict] = []
        # Clear events are stored as {"type": "clear"}
        self._events: List[dict] = []
        self._events_lock = threading.Lock()

        # Real robot handles (None in simulation mode)
        self._robot = None
        self._cmd_client = None
        self._lease_keepalive = None
        self._last_sent_uv: Optional[Tuple[float, float]] = None

        if not simulation:
            self._connect_spot()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def brush_uv(self) -> Tuple[float, float]:
        with self._brush_lock:
            return self._brush_uv

    @property
    def pen_down(self) -> bool:
        with self._brush_lock:
            return self._pen_down

    @property
    def mode(self) -> RobotMode:
        return self.state.mode

    def move_brush(self, u: float, v: float, pen_down: bool = True) -> None:
        """
        Move the brush to canvas UV position (u, v) ∈ [0,1]².
        Silently ignored while in ESTOP mode.
        """
        if self.state.mode == RobotMode.ESTOP:
            return

        u, v = clamp_uv(u, v)

        with self._brush_lock:
            self._brush_uv = (u, v)
            self._pen_down = pen_down

        # Buffer stroke for web UI
        with self._events_lock:
            self._events.append(
                {"type": "stroke", "u": u, "v": v, "pen_down": pen_down}
            )

        if self.simulation:
            logger.debug("[SIM] brush → (%.3f, %.3f) pen=%s", u, v, pen_down)
            return

        # Throttle real arm commands
        if self._last_sent_uv is not None:
            du = u - self._last_sent_uv[0]
            dv = v - self._last_sent_uv[1]
            if math.hypot(du, dv) < self._MOVE_THRESHOLD_UV:
                return

        self._send_arm_command(u, v, pen_down)
        self._last_sent_uv = (u, v)

    def emergency_stop(self) -> None:
        """Activate E-Stop.  Freezes all motion immediately."""
        self.state.emergency_stop()
        if not self.simulation and self._cmd_client is not None:
            try:
                from bosdyn.client.robot_command import RobotCommandBuilder

                self._cmd_client.robot_command(RobotCommandBuilder.stop_command())
                logger.warning("[ESTOP] Stop command sent to Spot.")
            except Exception as exc:
                logger.error("[ESTOP] Could not send stop command: %s", exc)

    def emergency_takeover(self) -> bool:
        """
        Hand control from AUTONOMOUS to TELEOP (operator override).
        Returns True if the transition succeeded.
        """
        return self.state.emergency_takeover()

    def release_teleop(self) -> bool:
        """Return from TELEOP to AUTONOMOUS. Returns True on success."""
        return self.state.release_teleop()

    def start_autonomous(self) -> bool:
        return self.state.start_autonomous()

    def start_teleop(self) -> bool:
        return self.state.start_teleop()

    def reset(self) -> None:
        """Clear E-Stop and return to IDLE."""
        self.state.reset()

    def update_vicon(self, frame: ViconFrame) -> None:
        """
        Called by the Vicon thread for every new frame.
        In AUTONOMOUS mode, uses the Vicon brush-tip position to move the arm.
        """
        with self._frame_lock:
            self._latest_frame = frame

        if self.state.mode == RobotMode.AUTONOMOUS:
            self._autonomous_tick(frame)

    def pop_new_events(self) -> List[dict]:
        """
        Return and clear all buffered stroke / clear events.
        Called by the web UI push loop.
        """
        with self._events_lock:
            events = list(self._events)
            self._events.clear()
        return events

    def clear_canvas(self) -> None:
        """Clear the stroke buffer and fire a clear event for the web UI."""
        with self._events_lock:
            self._events.clear()
            self._events.append({"type": "clear"})

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _autonomous_tick(self, frame: ViconFrame) -> None:
        """Drive the brush from Vicon brush-tip position."""
        if frame.canvas is None or not frame.canvas.is_valid():
            return
        if frame.brush_tip is None or frame.brush_tip.occluded:
            return
        u, v = world_to_canvas(frame.brush_tip.position, frame.canvas)
        self.move_brush(u, v, pen_down=True)

    def _connect_spot(self) -> None:
        import bosdyn.client
        import bosdyn.client.util
        from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
        from bosdyn.client.robot_command import RobotCommandClient, blocking_stand

        # Locate repo root to import config
        repo_root = __file__
        for _ in range(5):
            repo_root = str(__import__("pathlib").Path(repo_root).parent)
            if __import__("os").path.isfile(f"{repo_root}/config/robot_config.py"):
                break
        sys.path.insert(0, repo_root)
        from config.robot_config import ROBOT_IP, USERNAME, PASSWORD  # type: ignore[import]

        sdk = bosdyn.client.create_standard_sdk("ViconDraw")
        robot = sdk.create_robot(ROBOT_IP)
        robot.authenticate(USERNAME, PASSWORD)
        robot.time_sync.wait_for_sync()
        robot.power_on(timeout_sec=20)

        lease_client = robot.ensure_client(LeaseClient.default_service_name)
        self._lease_keepalive = LeaseKeepAlive(
            lease_client, must_acquire=True, return_at_exit=True
        )
        self._cmd_client = robot.ensure_client(RobotCommandClient.default_service_name)
        blocking_stand(self._cmd_client, timeout_sec=10)
        self._robot = robot
        logger.info("[Spot] Connected and standing.")

    def _send_arm_command(self, u: float, v: float, pen_down: bool) -> None:
        """Send a Cartesian arm command to the real Spot in the odom frame."""
        try:
            import bosdyn.client.math_helpers as math_helpers
            from bosdyn.api import (
                arm_command_pb2,
                robot_command_pb2,
                synchronized_command_pb2,
            )
            from bosdyn.api.geometry_pb2 import SE3Pose, Vec3, Quaternion
            from google.protobuf import duration_pb2
        except ImportError:
            logger.error("[Spot] bosdyn SDK not available.")
            return

        with self._frame_lock:
            frame = self._latest_frame
        if frame is None or frame.canvas is None:
            return

        z_mm = self.DRAW_HEIGHT_MM if pen_down else self.LIFT_HEIGHT_MM
        pos_mm = canvas_to_world(u, v, frame.canvas, z_offset_mm=z_mm)
        x, y, z = pos_mm / 1000.0  # mm → m

        # pitch = pi/2 quaternion: w=cos(pi/4), y=sin(pi/4), x=z=0
        rot = Quaternion(w=0.7071068, x=0.0, y=0.7071068, z=0.0)
        duration = duration_pb2.Duration(seconds=0, nanos=int(0.15 * 1e9))

        cmd = robot_command_pb2.RobotCommand()
        arm_cart = cmd.synchronized_command.arm_command.arm_cartesian_command
        arm_cart.root_frame_name = "odom"
        point = arm_cart.pose_trajectory_in_task.points.add()
        point.pose.position.x = x
        point.pose.position.y = y
        point.pose.position.z = z
        point.pose.rotation.CopyFrom(rot)
        point.time_since_reference.CopyFrom(duration)
        self._cmd_client.robot_command(cmd)
