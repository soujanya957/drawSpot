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

Smooth movement
---------------
For the real robot, arm commands are NOT sent immediately on every move_brush
call.  Instead a background motion thread runs at 20 Hz and steps the
commanded position toward the target by at most MAX_UV_STEP per tick.
This prevents jerky motion when the mouse moves fast and keeps the arm
within safe acceleration limits.

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
  pop_new_events()             — consume buffered stroke events for the web UI
  clear_canvas()               — empty the stroke buffer and fire a clear event
  get_ik_grid(n)               — NxN reachability booleans for the web overlay
  get_canvas_dims()            — physical canvas size in mm

Thread safety
-------------
All public methods are safe to call from any thread.
"""

from __future__ import annotations

import logging
import math
import sys
import threading
import time
from typing import List, Optional, Tuple

import numpy as np

from .state import RobotMode, RobotStateMachine
from ..vicon.types import ViconFrame
from ..vicon.transform import canvas_to_world, clamp_uv, world_to_canvas

logger = logging.getLogger(__name__)

# Motion thread parameters
_MOTION_HZ    = 20                  # arm command rate (Hz)
_MOTION_TICK  = 1.0 / _MOTION_HZ
MAX_UV_STEP   = 0.008               # max UV distance per tick ≈ 64 mm/s on a 400 mm canvas

# IK workspace limits (conservative)
_ARM_REACH_MAX_M = 0.85
_ARM_REACH_MIN_M = 0.15


class SpotController:
    # Height of brush tip above the canvas surface (mm).
    DRAW_HEIGHT_MM = 2.0   # pen touching
    LIFT_HEIGHT_MM = 30.0  # pen raised

    # Minimum position change (UV) before sending a new arm command.
    _MOVE_THRESHOLD_UV = 0.002  # ~0.8 mm on a 400 mm canvas

    def __init__(self, simulation: bool = True) -> None:
        self.simulation = simulation

        self.state = RobotStateMachine()

        # Desired target position (set by move_brush, may jump around fast)
        self._target_uv:  Tuple[float, float] = (0.5, 0.5)
        self._target_pen: bool = False

        # Last position actually sent to the arm (interpolated slowly toward target)
        self._brush_uv:  Tuple[float, float] = (0.5, 0.5)
        self._pen_down:  bool = False
        self._brush_lock = threading.Lock()

        # Latest Vicon frame (guarded by _frame_lock)
        self._latest_frame: Optional[ViconFrame] = None
        self._frame_lock = threading.Lock()

        # Event buffer for the web UI (guarded by _events_lock)
        self._events: List[dict] = []
        self._events_lock = threading.Lock()

        # Real robot handles (None in simulation mode)
        self._robot = None
        self._cmd_client = None
        self._lease_keepalive = None

        if not simulation:
            self._connect_spot()
            self._start_motion_thread()

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

        In simulation the position updates immediately.
        On the real robot the motion thread smoothly interpolates toward the
        new target at MAX_UV_STEP per tick.
        """
        if self.state.mode == RobotMode.ESTOP:
            return

        u, v = clamp_uv(u, v)

        # Set target for the motion thread (real robot) or update directly (sim)
        with self._brush_lock:
            self._target_uv  = (u, v)
            self._target_pen = pen_down
            if self.simulation:
                self._brush_uv = (u, v)
                self._pen_down = pen_down

        # Buffer stroke for web UI — immediate visual feedback regardless of mode
        with self._events_lock:
            self._events.append({"type": "stroke", "u": u, "v": v, "pen_down": pen_down})

        if self.simulation:
            logger.debug("[SIM] brush → (%.3f, %.3f) pen=%s", u, v, pen_down)

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
        """Hand control from AUTONOMOUS to TELEOP (operator override)."""
        return self.state.emergency_takeover()

    def release_teleop(self) -> bool:
        """Return from TELEOP to AUTONOMOUS."""
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
        """Return and clear all buffered stroke / clear events for the web UI."""
        with self._events_lock:
            events = list(self._events)
            self._events.clear()
        return events

    def clear_canvas(self) -> None:
        """Clear the stroke buffer and fire a clear event for the web UI."""
        with self._events_lock:
            self._events.clear()
            self._events.append({"type": "clear"})

    def get_ik_grid(self, n: int = 16) -> Optional[List[bool]]:
        """
        Sample an n×n UV grid and return which points are within the arm's
        workspace given the current Spot body position and canvas geometry.

        Returns a flat list of n*n booleans (row-major, u varies fastest),
        or None if no Vicon data is available.
        """
        with self._frame_lock:
            frame = self._latest_frame
        if frame is None or frame.canvas is None or not frame.canvas.is_valid():
            return None
        if frame.spot_body is None or frame.spot_body.occluded:
            return None

        R = frame.spot_body.rotation_matrix
        body_pos = frame.spot_body.position
        grid: List[bool] = []

        for row in range(n):
            for col in range(n):
                u = col / (n - 1)
                v = row / (n - 1)
                target_mm = canvas_to_world(u, v, frame.canvas,
                                            z_offset_mm=self.DRAW_HEIGHT_MM)
                pos_body_m = R.T @ (target_mm - body_pos) / 1000.0
                dist = float(np.linalg.norm(pos_body_m))
                reachable = (
                    _ARM_REACH_MIN_M <= dist <= _ARM_REACH_MAX_M
                    and pos_body_m[0] >= 0.0
                )
                grid.append(reachable)

        return grid

    def get_canvas_dims(self) -> dict:
        """
        Return physical canvas dimensions in mm.
        Uses Vicon canvas geometry if available, otherwise falls back to config.
        """
        with self._frame_lock:
            frame = self._latest_frame
        if frame is not None and frame.canvas is not None and frame.canvas.is_valid():
            return {
                "width_mm":  frame.canvas.width_mm,
                "height_mm": frame.canvas.height_mm,
            }
        # Fall back to config values
        try:
            repo_root = __file__
            for _ in range(5):
                repo_root = str(__import__("pathlib").Path(repo_root).parent)
                if __import__("os").path.isfile(f"{repo_root}/config/robot_config.py"):
                    break
            sys.path.insert(0, repo_root)
            from config.robot_config import CANVAS_WIDTH_M, CANVAS_HEIGHT_M  # type: ignore
            return {"width_mm": CANVAS_WIDTH_M * 1000, "height_mm": CANVAS_HEIGHT_M * 1000}
        except Exception:
            return {"width_mm": 400.0, "height_mm": 400.0}

    # ------------------------------------------------------------------
    # Internal — motion thread
    # ------------------------------------------------------------------

    def _start_motion_thread(self) -> None:
        t = threading.Thread(target=self._motion_loop, daemon=True, name="ArmMotion")
        t.start()

    def _motion_loop(self) -> None:
        """
        Runs at _MOTION_HZ.  Steps the commanded arm position toward the
        target by at most MAX_UV_STEP per tick, then sends the arm command.
        """
        while True:
            time.sleep(_MOTION_TICK)

            if self.state.mode == RobotMode.ESTOP:
                continue

            with self._brush_lock:
                cu, cv   = self._brush_uv
                tu, tv   = self._target_uv
                pen      = self._target_pen

            du, dv = tu - cu, tv - cv
            dist = math.hypot(du, dv)

            if dist < self._MOVE_THRESHOLD_UV:
                continue

            # Clamp step
            if dist > MAX_UV_STEP:
                scale = MAX_UV_STEP / dist
                du, dv = du * scale, dv * scale

            nu, nv = clamp_uv(cu + du, cv + dv)

            with self._brush_lock:
                self._brush_uv = (nu, nv)
                self._pen_down = pen

            self._send_arm_command(nu, nv, pen)

    # ------------------------------------------------------------------
    # Internal — autonomous
    # ------------------------------------------------------------------

    def _autonomous_tick(self, frame: ViconFrame) -> None:
        """Drive the brush from Vicon brush-tip position."""
        if frame.canvas is None or not frame.canvas.is_valid():
            return
        if frame.brush_tip is None or frame.brush_tip.occluded:
            return
        u, v = world_to_canvas(frame.brush_tip.position, frame.canvas)
        self.move_brush(u, v, pen_down=True)

    # ------------------------------------------------------------------
    # Internal — Spot connection
    # ------------------------------------------------------------------

    def _connect_spot(self) -> None:
        import bosdyn.client
        import bosdyn.client.util
        from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
        from bosdyn.client.robot_command import RobotCommandClient, blocking_stand

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
            from bosdyn.api import robot_command_pb2
            from bosdyn.api.geometry_pb2 import Quaternion
            from google.protobuf import duration_pb2
        except ImportError:
            logger.error("[Spot] bosdyn SDK not available.")
            return

        with self._frame_lock:
            frame = self._latest_frame
        if frame is None or frame.canvas is None:
            return

        z_mm  = self.DRAW_HEIGHT_MM if pen_down else self.LIFT_HEIGHT_MM
        pos_mm = canvas_to_world(u, v, frame.canvas, z_offset_mm=z_mm)
        x, y, z = pos_mm / 1000.0

        rot      = Quaternion(w=0.7071068, x=0.0, y=0.7071068, z=0.0)
        duration = duration_pb2.Duration(seconds=0, nanos=int(0.15 * 1e9))

        cmd      = robot_command_pb2.RobotCommand()
        arm_cart = cmd.synchronized_command.arm_command.arm_cartesian_command
        arm_cart.root_frame_name = "odom"
        point    = arm_cart.pose_trajectory_in_task.points.add()
        point.pose.position.x = x
        point.pose.position.y = y
        point.pose.position.z = z
        point.pose.rotation.CopyFrom(rot)
        point.time_since_reference.CopyFrom(duration)
        self._cmd_client.robot_command(cmd)
