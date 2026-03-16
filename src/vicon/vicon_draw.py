"""
Spot Draw
=========
Two-phase unified controller.

Phase 1 — TELEOP  (terminal)
  Walk the robot to the brush, pick it up, walk to canvas.
  Same controls as full_control.py.

  Key       Action
  ──────────────────────────────────────────
  w/s/a/d   base forward / back / strafe
  q/e       base yaw left / right
  SPACE     stop base
  TAB       toggle arm mode (IJKL UO RPY)
  [ / ]     gripper open / close
  z         stow arm
  n         pick object from hand camera
  d         enter draw pose → launch web UI
  ESC       emergency stop (stop + stow + sit)
  x         exit

Phase 2 — DRAW  (web UI at http://localhost:8080)
  Arm moves to the draw pose above the canvas center.
  Browser canvas controls the brush:
    mouse click + hold  → pen down, arm follows in 2D
    mouse release       → pen up
    IK overlay          → green = reachable, red = out of range
  Terminal: press ESC to exit draw mode and return to teleop.

Run:
    python -m src.vicon.vicon_draw                  # uses VICON_ADDRESS from .env
    python -m src.vicon.vicon_draw --vicon HOST:PORT
    python -m src.vicon.vicon_draw --mock
"""

import argparse
import asyncio
import json
import math
import os
import select
import sys
import termios
import threading
import time
import tty
from contextlib import asynccontextmanager
from typing import Optional, Set

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from google.protobuf import duration_pb2

import bosdyn.client
import bosdyn.client.util
from bosdyn.api import geometry_pb2, manipulation_api_pb2, robot_command_pb2
from bosdyn.api.geometry_pb2 import Quaternion
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import BODY_FRAME_NAME
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand

sys.path.insert(0, __file__.split("/src/")[0])
from config.robot_config import (
    BASE_ROTATION_MAX, BASE_VELOCITY_MAX,
    PASSWORD, ROBOT_IP, USERNAME, VICON_ADDRESS,
)
from vicon.client import MockViconClient, ViconClient
from vicon.transform import canvas_to_world, clamp_uv

_REPO_ROOT  = __file__.split("/src/")[0]
_STATIC_DIR = os.path.join(_REPO_ROOT, "draw", "ui", "static")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

BRUSH_LENGTH_MM   = 150.0   # brush extends this far below gripper tip
PEN_UP_EXTRA_MM   =  60.0   # extra clearance when pen is raised
ARM_REACH_MAX_M   = 0.85
ARM_REACH_MIN_M   = 0.15
BASE_CMD_DUR      = 0.5
ARM_STEP_POS      = 0.02
ARM_STEP_ROT      = math.radians(3)
HAND_COLOR_SOURCE = "hand_color_image"
STATUS_INTERVAL   = 0.5
WEB_PORT          = 8080
IK_GRID_N         = 16
MOTION_HZ         = 20      # arm command rate
MAX_ARM_UV_SPEED  = 0.20    # canvas UV/s ≈ 80 mm/s on a 400 mm canvas

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _dur(secs):
    return duration_pb2.Duration(seconds=int(secs), nanos=int((secs - int(secs)) * 1e9))


def _rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(
        w=cr*cp*cy + sr*sp*sy, x=sr*cp*cy - cr*sp*sy,
        y=cr*sp*cy + sr*cp*sy, z=cr*cp*sy - sr*sp*cy,
    )


def _rotation_matrix(q):
    qx, qy, qz, qw = q
    return np.array([
        [1-2*(qy**2+qz**2),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [  2*(qx*qy+qz*qw), 1-2*(qx**2+qz**2),   2*(qy*qz-qx*qw)],
        [  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx**2+qy**2)],
    ], dtype=float)


def _world_to_body(target_mm, body_pos_mm, body_quat):
    R = _rotation_matrix(body_quat)
    return R.T @ (target_mm - body_pos_mm) / 1000.0


def _reachable(pos_body_m):
    d = float(np.linalg.norm(pos_body_m))
    return ARM_REACH_MIN_M <= d <= ARM_REACH_MAX_M and pos_body_m[0] >= 0.0


def _get_key(timeout=0.05):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

# ---------------------------------------------------------------------------
# Vicon / IK helpers
# ---------------------------------------------------------------------------

def _compute_ik_grid(frame, n=IK_GRID_N):
    if frame is None or frame.canvas is None or not frame.canvas.is_valid():
        return None
    if frame.spot_body is None or frame.spot_body.occluded:
        return None
    R        = _rotation_matrix(frame.spot_body.rotation_quat)
    body_pos = frame.spot_body.position
    grid = []
    for row in range(n):
        for col in range(n):
            u, v = col / (n - 1), row / (n - 1)
            t = canvas_to_world(u, v, frame.canvas, z_offset_mm=BRUSH_LENGTH_MM)
            p = R.T @ (t - body_pos) / 1000.0
            grid.append(bool(_reachable(p)))
    return grid


def _compute_canvas_rotation(frame):
    """
    Returns the display rotation (0/90/180/270) so that the canvas edge
    closest to Spot's body appears at the bottom of the 2-D display.

      0   — Spot at south (v=1 edge)  — no rotation
      90  — Spot at west  (u=0 edge)  — 90° CCW: left edge → display bottom
      180 — Spot at north (v=0 edge)  — 180°:    top edge  → display bottom
      270 — Spot at east  (u=1 edge)  — 90° CW:  right edge → display bottom
    """
    if frame is None or frame.canvas is None or not frame.canvas.is_valid():
        return 0
    if frame.spot_body is None or frame.spot_body.occluded:
        return 0
    c = frame.canvas
    center = np.mean(np.array(c.corners), axis=0)
    toward_spot = np.array(frame.spot_body.position) - center
    dot_u = float(np.dot(toward_spot, c.x_axis))   # +u = east (u=1 side)
    dot_v = float(np.dot(toward_spot, c.y_axis))   # +v = south (v=1 side)
    if abs(dot_v) >= abs(dot_u):
        return 0 if dot_v >= 0 else 180
    else:
        return 270 if dot_u >= 0 else 90


def _get_canvas_dims(frame):
    rotation = _compute_canvas_rotation(frame)
    if frame and frame.canvas and frame.canvas.is_valid():
        return {
            "width_mm":  frame.canvas.width_mm,
            "height_mm": frame.canvas.height_mm,
            "rotation":  rotation,
        }
    return {"width_mm": 400.0, "height_mm": 400.0, "rotation": 0}

# ---------------------------------------------------------------------------
# Vicon status display
# ---------------------------------------------------------------------------

def _fmt_rb(rb, label):
    if rb is None:       return f"no data from Vicon for {label}"
    if rb.occluded:      return f"occluded [{label}]"
    x, y, z = rb.position
    return f"({x:+8.1f}, {y:+8.1f}, {z:+8.1f}) mm"


def _print_status(frame, arm_mode, vicon_client=None):
    if vicon_client is None:
        print("  [Vicon] not connected — teleop only")
        print(f"  Mode      : {'ARM' if arm_mode else 'BASE'}")
        return
    if frame is None:
        print("  [Vicon] connected but waiting for first frame…")
        print(f"  Mode      : {'ARM' if arm_mode else 'BASE'}")
        return
    canvas_str = "not visible"
    if frame.canvas and frame.canvas.is_valid():
        c = frame.canvas
        center = np.mean(np.array(c.corners), axis=0)
        canvas_str = (f"({center[0]:+8.1f}, {center[1]:+8.1f}, {center[2]:+8.1f}) mm"
                      f"  [{c.width_mm:.0f}×{c.height_mm:.0f} mm]")
    print(
        f"  Spot body : {_fmt_rb(frame.spot_body, 'Spot')}\n"
        f"  Spot EE   : {_fmt_rb(frame.spot_ee,   'SpotEE')}\n"
        f"  Canvas    : {canvas_str}\n"
        f"  Mode      : {'ARM' if arm_mode else 'BASE'}"
    )

# ---------------------------------------------------------------------------
# Robot commands
# ---------------------------------------------------------------------------

def send_arm_cartesian(cmd_client, x, y, z, roll, pitch, yaw, frame_name=BODY_FRAME_NAME):
    cmd      = robot_command_pb2.RobotCommand()
    arm_cart = cmd.synchronized_command.arm_command.arm_cartesian_command
    arm_cart.root_frame_name = frame_name
    point    = arm_cart.pose_trajectory_in_task.points.add()
    point.pose.position.x = x
    point.pose.position.y = y
    point.pose.position.z = z
    point.pose.rotation.CopyFrom(_rpy_to_quat(roll, pitch, yaw))
    point.time_since_reference.CopyFrom(_dur(0.4))
    cmd_client.robot_command(cmd)


def _send_draw_command(cmd_client, frame, u, v, pen_down):
    """UV → odom arm command. Returns True if sent, False if out of reach."""
    if frame is None or frame.canvas is None or not frame.canvas.is_valid():
        return False
    if frame.spot_body is None or frame.spot_body.occluded:
        return False
    z_off     = BRUSH_LENGTH_MM + (0.0 if pen_down else PEN_UP_EXTRA_MM)
    target_mm = canvas_to_world(u, v, frame.canvas, z_offset_mm=z_off)
    pos_body  = _world_to_body(target_mm, frame.spot_body.position, frame.spot_body.rotation_quat)
    if not _reachable(pos_body):
        return False
    x, y, z  = target_mm / 1000.0
    rot       = Quaternion(w=0.7071068, x=0.0, y=0.7071068, z=0.0)
    cmd       = robot_command_pb2.RobotCommand()
    arm_cart  = cmd.synchronized_command.arm_command.arm_cartesian_command
    arm_cart.root_frame_name = "odom"
    point     = arm_cart.pose_trajectory_in_task.points.add()
    point.pose.position.x = x
    point.pose.position.y = y
    point.pose.position.z = z
    point.pose.rotation.CopyFrom(rot)
    point.time_since_reference.CopyFrom(_dur(0.15))
    cmd_client.robot_command(cmd)
    return True


def move_to_draw_pose(cmd_client, vicon_client):
    """Move arm to above canvas center, brush pointing straight down."""
    frame = _latest_frame(vicon_client) if vicon_client else None
    if frame and frame.canvas and frame.canvas.is_valid():
        c      = frame.canvas
        center = np.mean(np.array(c.corners), axis=0)
        pos_mm = center + (BRUSH_LENGTH_MM + PEN_UP_EXTRA_MM) * c.normal
        x, y, z = pos_mm / 1000.0
        rot    = Quaternion(w=0.7071068, x=0.0, y=0.7071068, z=0.0)
        cmd    = robot_command_pb2.RobotCommand()
        ac     = cmd.synchronized_command.arm_command.arm_cartesian_command
        ac.root_frame_name = "odom"
        pt     = ac.pose_trajectory_in_task.points.add()
        pt.pose.position.x = x
        pt.pose.position.y = y
        pt.pose.position.z = z
        pt.pose.rotation.CopyFrom(rot)
        pt.time_since_reference.CopyFrom(_dur(2.5))
        cmd_client.robot_command(cmd)
        print(f"  Draw pose: canvas center at odom ({x:.3f}, {y:.3f}, {z:.3f}) m")
    else:
        # Fallback: fixed body-frame pose (arm forward, pointing down)
        send_arm_cartesian(cmd_client, 0.6, 0.0, -0.1, 0.0, math.pi / 2, 0.0)
        print("  Draw pose: fixed body-frame fallback (no canvas visible)")


def stow_arm(cmd_client):
    cmd_client.robot_command(RobotCommandBuilder.arm_stow_command())
    print("  Arm stowed.")


def open_gripper(cmd_client):
    cmd_client.robot_command(RobotCommandBuilder.claw_gripper_open_command())
    print("  Gripper open.")


def close_gripper(cmd_client):
    cmd_client.robot_command(RobotCommandBuilder.claw_gripper_close_command())
    print("  Gripper closed.")


def emergency_stop(cmd_client):
    print("\n!! EMERGENCY STOP !!")
    cmd_client.robot_command(RobotCommandBuilder.stop_command())
    time.sleep(0.5)
    cmd_client.robot_command(RobotCommandBuilder.arm_stow_command())
    time.sleep(2)
    cmd_client.robot_command(RobotCommandBuilder.synchro_sit_command())
    time.sleep(2)
    print("Emergency stop complete.")

# ---------------------------------------------------------------------------
# Pick object (Phase 1 only)
# ---------------------------------------------------------------------------

def _select_pixel(img):
    h, w = img.shape[:2]
    cx, cy = w // 2, h // 2
    win = "Select target  |  arrows=1px  WASD=10px  ENTER=confirm  q=cancel"
    cv2.namedWindow(win)
    while True:
        display = img.copy()
        cv2.line(display, (0, cy), (w, cy), (0, 255, 0), 1)
        cv2.line(display, (cx, 0), (cx, h), (0, 255, 0), 1)
        cv2.circle(display, (cx, cy), 6, (0, 255, 0), 2)
        cv2.putText(display, f"({cx},{cy})", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        cv2.imshow(win, display)
        raw = cv2.waitKeyEx(0)
        key = raw & 0xFF
        if key in (13, ord(' ')):
            cv2.destroyWindow(win); return (cx, cy)
        if key in (ord('q'), 27):
            cv2.destroyWindow(win); return None
        if   raw == 2424832: cx = max(0,   cx - 1)
        elif raw == 2555904: cx = min(w-1, cx + 1)
        elif raw == 2490368: cy = max(0,   cy - 1)
        elif raw == 2621440: cy = min(h-1, cy + 1)
        elif key == ord('a'): cx = max(0,   cx - 10)
        elif key == ord('d'): cx = min(w-1, cx + 10)
        elif key == ord('w'): cy = max(0,   cy - 10)
        elif key == ord('s'): cy = min(h-1, cy + 10)


def do_pick(cmd_client, img_client, manip_client, arm_pose):
    print("  Moving arm to survey position…")
    ax, ay, az = 0.7, 0.0, 0.0
    aroll, apitch, ayaw = 0.0, math.pi / 2, 0.0
    send_arm_cartesian(cmd_client, ax, ay, az, aroll, apitch, ayaw)
    time.sleep(3)

    PICK_KEYS = {
        ord('i'): ("ax", +ARM_STEP_POS), ord('k'): ("ax", -ARM_STEP_POS),
        ord('j'): ("ay", +ARM_STEP_POS), ord('l'): ("ay", -ARM_STEP_POS),
        ord('u'): ("az", +ARM_STEP_POS), ord('o'): ("az", -ARM_STEP_POS),
        ord('r'): ("aroll",  +ARM_STEP_ROT), ord('f'): ("aroll",  -ARM_STEP_ROT),
        ord('t'): ("apitch", +ARM_STEP_ROT), ord('g'): ("apitch", -ARM_STEP_ROT),
        ord('y'): ("ayaw",   +ARM_STEP_ROT), ord('h'): ("ayaw",   -ARM_STEP_ROT),
    }

    win = "Hand Camera  |  ijkl/uo=xyz  rf/tg/yh=rot  SPACE=confirm  q=cancel"
    cv2.namedWindow(win)
    print("  Adjust arm then press SPACE.")

    while True:
        req  = build_image_request(HAND_COLOR_SOURCE, quality_percent=75)
        resp = img_client.get_image([req])[0]
        img  = cv2.imdecode(np.frombuffer(resp.shot.image.data, np.uint8), cv2.IMREAD_COLOR)
        img_response = resp
        cv2.imshow(win, img)
        key = cv2.waitKey(50) & 0xFF
        if key in (ord('q'), ord('Q')):
            cv2.destroyAllWindows(); arm_pose[:] = [ax,ay,az,aroll,apitch,ayaw]; print("  Cancelled."); return
        if key in (ord(' '), 13):
            break
        if key in PICK_KEYS:
            attr, delta = PICK_KEYS[key]
            if   attr == "ax":     ax     += delta
            elif attr == "ay":     ay     += delta
            elif attr == "az":     az     += delta
            elif attr == "aroll":  aroll  += delta
            elif attr == "apitch": apitch += delta
            elif attr == "ayaw":   ayaw   += delta
            send_arm_cartesian(cmd_client, ax, ay, az, aroll, apitch, ayaw)

    arm_pose[:] = [ax, ay, az, aroll, apitch, ayaw]
    cv2.destroyAllWindows()

    req  = build_image_request(HAND_COLOR_SOURCE, quality_percent=75)
    resp = img_client.get_image([req])[0]
    frozen = cv2.imdecode(np.frombuffer(resp.shot.image.data, np.uint8), cv2.IMREAD_COLOR)
    frozen_response = resp

    print("  Position crosshair, ENTER to confirm.")
    result = _select_pixel(frozen)
    if result is None:
        print("  Pick cancelled."); return

    px, py = result
    print(f"  Picking at pixel ({px}, {py})…")
    pick_req = manipulation_api_pb2.PickObjectInImage(
        pixel_xy=geometry_pb2.Vec2(x=float(px), y=float(py)),
        transforms_snapshot_for_camera=frozen_response.shot.transforms_snapshot,
        frame_name_image_sensor=frozen_response.shot.frame_name_image_sensor,
        camera_model=frozen_response.source.pinhole,
    )
    resp2 = manip_client.manipulation_api_command(
        manipulation_api_request=manipulation_api_pb2.ManipulationApiRequest(
            pick_object_in_image=pick_req
        )
    )
    deadline = time.time() + 15.0
    while time.time() < deadline:
        fb = manip_client.manipulation_api_feedback_command(
            manipulation_api_feedback_request=manipulation_api_pb2.ManipulationApiFeedbackRequest(
                manipulation_cmd_id=resp2.manipulation_cmd_id
            )
        )
        state = fb.current_state
        print(f"  Grasp: {manipulation_api_pb2.ManipulationFeedbackState.Name(state)}   ", end="\r")
        if state in (
            manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED,
            manipulation_api_pb2.MANIP_STATE_GRASP_FAILED,
            manipulation_api_pb2.MANIP_STATE_GRASP_PLANNING_NO_SOLUTION,
        ):
            print()
            if state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED:
                print("  Grasp SUCCEEDED!")
            else:
                print(f"  Grasp ended: {manipulation_api_pb2.ManipulationFeedbackState.Name(state)}")
            return
        time.sleep(0.3)
    print("\n  Grasp timed out.")

# ---------------------------------------------------------------------------
# Draw web server (Phase 2)
# ---------------------------------------------------------------------------

class DrawServer:
    """
    Minimal FastAPI/WebSocket server for Phase 2.
    Serves the existing draw/ui/static/ files and forwards
    mouse move commands directly to the robot arm.
    """

    def __init__(self, cmd_client, vicon_client):
        self._cmd    = cmd_client
        self._vicon  = vicon_client
        self._clients: Set[WebSocket] = set()
        # Latest target received from browser (canvas UV, updated by WS handler)
        self._target_uv:  Optional[tuple] = None
        self._target_pen: bool            = False
        self._app    = self._build_app()
        self._thread: Optional[threading.Thread] = None

    def _build_app(self) -> FastAPI:
        srv = self

        @asynccontextmanager
        async def lifespan(app: FastAPI):
            task = asyncio.create_task(srv._push_loop())
            yield
            task.cancel()

        app = FastAPI(lifespan=lifespan)
        app.mount("/static", StaticFiles(directory=_STATIC_DIR), name="static")

        @app.get("/")
        async def index():
            with open(os.path.join(_STATIC_DIR, "index.html")) as f:
                return HTMLResponse(f.read())

        @app.websocket("/ws")
        async def ws_endpoint(websocket: WebSocket):
            await websocket.accept()
            srv._clients.add(websocket)
            # Tell the UI we're in TELEOP so the canvas is immediately active
            await websocket.send_json({"type": "mode", "data": "TELEOP"})
            # Push current canvas dims right away
            frame = srv._vicon.latest_frame
            await websocket.send_json({"type": "canvas_dims", **_get_canvas_dims(frame)})
            try:
                async for raw in websocket.iter_text():
                    try:
                        await srv._handle(json.loads(raw))
                    except Exception:
                        pass
            except WebSocketDisconnect:
                pass
            finally:
                srv._clients.discard(websocket)

        return app

    async def _handle(self, msg: dict):
        t = msg.get("type")
        if t == "move":
            # Store target only — motion loop sends arm commands at controlled rate
            self._target_uv  = (float(msg.get("u", 0.5)), float(msg.get("v", 0.5)))
            self._target_pen = bool(msg.get("pen_down", True))
        elif t == "estop":
            emergency_stop(self._cmd)

    async def _push_loop(self):
        """
        Runs at MOTION_HZ (20 Hz).
        - Smoothly interpolates arm toward latest target UV at MAX_ARM_UV_SPEED.
        - Pushes IK grid + canvas dims to browsers once per second.
        """
        _tick     = 1.0 / MOTION_HZ
        _max_step = MAX_ARM_UV_SPEED * _tick

        last_ik = 0.0
        cur_u, cur_v = 0.5, 0.5

        ev_loop = asyncio.get_event_loop()
        while True:
            t0 = ev_loop.time()

            # ── Smooth arm motion ─────────────────────────────────────────
            if self._target_uv is not None:
                tu, tv = self._target_uv
                du, dv = tu - cur_u, tv - cur_v
                dist = math.sqrt(du * du + dv * dv)
                if dist > 1e-6:
                    step   = min(dist, _max_step)
                    cur_u += (du / dist) * step
                    cur_v += (dv / dist) * step
                frame = _latest_frame(self._vicon)
                _send_draw_command(self._cmd, frame, cur_u, cur_v, self._target_pen)

            # ── IK grid + canvas dims (every 1 s) ────────────────────────
            now = ev_loop.time()
            if now - last_ik >= 1.0:
                last_ik = now
                frame = _latest_frame(self._vicon)
                ik = _compute_ik_grid(frame)
                if ik:
                    await self._broadcast({"type": "ik_grid", "n": IK_GRID_N, "data": ik})
                await self._broadcast({"type": "canvas_dims", **_get_canvas_dims(frame)})

            elapsed = ev_loop.time() - t0
            await asyncio.sleep(max(0.001, _tick - elapsed))

    async def _broadcast(self, msg: dict):
        dead = set()
        for ws in self._clients:
            try:
                await ws.send_json(msg)
            except Exception:
                dead.add(ws)
        self._clients -= dead

    def start(self):
        def _run():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            config = uvicorn.Config(
                self._app, host="0.0.0.0", port=WEB_PORT, log_level="warning"
            )
            server = uvicorn.Server(config)
            loop.run_until_complete(server.serve())
        self._thread = threading.Thread(target=_run, daemon=True, name="DrawWebServer")
        self._thread.start()

# ---------------------------------------------------------------------------
# Main control loop
# ---------------------------------------------------------------------------

def _latest_frame(vicon_client):
    """Returns latest frame, or None if vicon_client is None."""
    return _latest_frame(vicon_client) if vicon_client is not None else None


def run(vicon_client, robot):
    print("[1/6] Acquiring clients…")
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    cmd_client   = robot.ensure_client(RobotCommandClient.default_service_name)
    img_client   = robot.ensure_client(ImageClient.default_service_name)
    manip_client = robot.ensure_client(ManipulationApiClient.default_service_name)
    estop_client = robot.ensure_client(EstopClient.default_service_name)

    print("[2/6] Setting up E-Stop…")
    estop_ep = EstopEndpoint(estop_client, name="spot_draw_estop", estop_timeout=9.0)
    estop_ep.force_simple_setup()
    print("[3/6] E-Stop configured. Acquiring lease…")

    with (
        EstopKeepAlive(estop_ep),
        LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True),
    ):
        print("[4/6] Lease acquired. Powering on…")
        robot.power_on(timeout_sec=20)
        print("[5/6] Standing up…")
        blocking_stand(cmd_client, timeout_sec=10)
        print("[6/6] Ready.\n")

        print("━" * 52)
        print("  PHASE 1 — TELEOP")
        print("  base   w/s/a/d/q/e  |  SPACE = stop")
        print("  arm    TAB to toggle  |  i/k j/l u/o = xyz")
        print("                        |  r/f t/g y/h = rpy")
        print("  grip   [ open   ] close")
        print("  z = stow   n = pick object")
        print("  d = draw pose + launch web UI")
        print("  ESC = emergency stop   x = exit")
        print("━" * 52 + "\n")

        ax, ay, az          = 0.6, 0.0, 0.3
        aroll, apitch, ayaw = 0.0, 0.0, 0.0
        arm_pose = [ax, ay, az, aroll, apitch, ayaw]
        arm_mode = False

        BASE_KEYS = {
            "w": ( BASE_VELOCITY_MAX, 0, 0),
            "s": (-BASE_VELOCITY_MAX, 0, 0),
            "a": (0,  BASE_VELOCITY_MAX, 0),
            "d": (0, -BASE_VELOCITY_MAX, 0),
            "q": (0, 0,  BASE_ROTATION_MAX),
            "e": (0, 0, -BASE_ROTATION_MAX),
            " ": (0, 0, 0),
        }

        ARM_KEYS = {
            "i": ("ax",     +ARM_STEP_POS), "k": ("ax",     -ARM_STEP_POS),
            "j": ("ay",     +ARM_STEP_POS), "l": ("ay",     -ARM_STEP_POS),
            "u": ("az",     +ARM_STEP_POS), "o": ("az",     -ARM_STEP_POS),
            "r": ("aroll",  +ARM_STEP_ROT), "f": ("aroll",  -ARM_STEP_ROT),
            "t": ("apitch", +ARM_STEP_ROT), "g": ("apitch", -ARM_STEP_ROT),
            "y": ("ayaw",   +ARM_STEP_ROT), "h": ("ayaw",   -ARM_STEP_ROT),
        }

        last_status_t = 0.0

        # ── Phase 1: teleop loop ─────────────────────────────────────────
        while True:
            now = time.time()
            if now - last_status_t >= STATUS_INTERVAL:
                print("─" * 52)
                _print_status(_latest_frame(vicon_client), arm_mode, vicon_client)
                last_status_t = now

            key = _get_key()
            if key is None:
                continue

            if key == "\x1b":
                emergency_stop(cmd_client)
                continue
            if key == "x":
                break

            if key == "z":
                stow_arm(cmd_client); continue
            if key == "n":
                do_pick(cmd_client, img_client, manip_client, arm_pose)
                ax, ay, az, aroll, apitch, ayaw = arm_pose
                continue
            if key == "p":
                print(f"  arm ({ax:.3f}, {ay:.3f}, {az:.3f})"
                      f"  rpy ({math.degrees(aroll):.1f}°,"
                      f" {math.degrees(apitch):.1f}°,"
                      f" {math.degrees(ayaw):.1f}°)")
                continue
            if key == "\t":
                arm_mode = not arm_mode
                print(f"  ARM mode: {'ON' if arm_mode else 'OFF'}")
                continue

            # ── Enter draw mode ──────────────────────────────────────────
            if key == "d":
                if vicon_client is None:
                    print("  Draw mode requires Vicon — not connected.")
                    continue
                print("\n  Moving to draw pose…")
                move_to_draw_pose(cmd_client, vicon_client)
                time.sleep(3)
                print("  Draw pose reached.")
                print("  Press ENTER to launch web UI, any other key to cancel.")
                confirm = _get_key(timeout=5.0)
                if confirm not in ("\r", "\n", " "):
                    print("  Cancelled.")
                    continue

                # ── Phase 2: web UI draw loop ────────────────────────────
                srv = DrawServer(cmd_client, vicon_client)
                srv.start()
                time.sleep(0.5)   # give uvicorn a moment to bind

                print(f"\n  Web UI at http://localhost:{WEB_PORT}")
                print("  Draw with mouse. Press ESC here to exit draw mode.\n")

                last_status_t = 0.0
                while True:
                    now = time.time()
                    if now - last_status_t >= STATUS_INTERVAL:
                        frame = _latest_frame(vicon_client)
                        canvas_ok = (frame and frame.canvas and frame.canvas.is_valid())
                        body_ok   = (frame and frame.spot_body and not frame.spot_body.occluded)
                        print(f"  [DRAW]  canvas={'OK' if canvas_ok else 'no data'}"
                              f"  body={'OK' if body_ok else 'no data'}"
                              f"  — ESC to exit")
                        last_status_t = now

                    k = _get_key(timeout=0.2)
                    if k == "\x1b":
                        # Lift pen before exiting
                        frame = _latest_frame(vicon_client)
                        if frame:
                            _send_draw_command(cmd_client, frame, 0.5, 0.5, pen_down=False)
                        print("\n  Exited draw mode.")
                        break

                last_status_t = 0.0
                continue   # back to Phase 1 teleop

            # ── Base ─────────────────────────────────────────────────────
            if key in BASE_KEYS:
                dvx, dvy, dvyaw = BASE_KEYS[key]
                cmd_client.robot_command(
                    command=RobotCommandBuilder.synchro_velocity_command(
                        v_x=dvx, v_y=dvy, v_rot=dvyaw,
                    ),
                    end_time_secs=time.time() + BASE_CMD_DUR,
                )
                continue

            # ── Gripper ──────────────────────────────────────────────────
            if key == "[": open_gripper(cmd_client);  continue
            if key == "]": close_gripper(cmd_client); continue

            # ── Arm ──────────────────────────────────────────────────────
            if arm_mode and key in ARM_KEYS:
                attr, delta = ARM_KEYS[key]
                if   attr == "ax":     ax     += delta
                elif attr == "ay":     ay     += delta
                elif attr == "az":     az     += delta
                elif attr == "aroll":  aroll  += delta
                elif attr == "apitch": apitch += delta
                elif attr == "ayaw":   ayaw   += delta
                arm_pose[:] = [ax, ay, az, aroll, apitch, ayaw]
                send_arm_cartesian(cmd_client, ax, ay, az, aroll, apitch, ayaw)

        # Clean exit
        print("\nExiting — stowing arm and sitting…")
        cmd_client.robot_command(RobotCommandBuilder.arm_stow_command())
        time.sleep(2)
        cmd_client.robot_command(RobotCommandBuilder.synchro_sit_command())
        time.sleep(2)
        print("Done.")

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

VICON_CONNECT_TIMEOUT = 5.0   # seconds to wait for first Vicon frame


def main():
    ap = argparse.ArgumentParser(description="Spot Draw — teleop + web UI canvas draw")
    ap.add_argument("--vicon", metavar="HOST:PORT",
                    help=f"Vicon address (default: VICON_ADDRESS from env = {VICON_ADDRESS})")
    ap.add_argument("--mock", action="store_true",
                    help="Use simulated Vicon (no hardware)")
    args = ap.parse_args()

    # ── Connect to Spot first ────────────────────────────────────────────────
    print("[1/2] Connecting to Spot…")
    sdk   = bosdyn.client.create_standard_sdk("SpotDraw")
    robot = sdk.create_robot(ROBOT_IP)
    robot.authenticate(USERNAME, PASSWORD)
    robot.time_sync.wait_for_sync()
    print("      Spot connected.\n")

    # ── Connect to Vicon (optional) ──────────────────────────────────────────
    vicon_addr = args.vicon or VICON_ADDRESS
    if args.mock:
        print("[2/2] Using mock Vicon client.")
        vicon = MockViconClient()
        vicon.start()
    elif vicon_addr is None:
        print("[2/2] No VICON_HOST set — running without Vicon (teleop only).")
        vicon = None
    else:
        print(f"[2/2] Connecting to Vicon at {vicon_addr}…")
        vicon = ViconClient(host=vicon_addr)
        vicon.start()
        deadline = time.time() + VICON_CONNECT_TIMEOUT
        while vicon.latest_frame is None and time.time() < deadline:
            time.sleep(0.05)
        if vicon.latest_frame is not None:
            f = vicon.latest_frame
            body_ok   = f.spot_body is not None and not f.spot_body.occluded
            ee_ok     = f.spot_ee   is not None and not f.spot_ee.occluded
            canvas_ok = f.canvas    is not None and f.canvas.is_valid()
            print(f"      Vicon connected.")
            print(f"      Spot body : {'OK' if body_ok   else 'no data'}")
            print(f"      Spot EE   : {'OK' if ee_ok     else 'no data'}")
            print(f"      Canvas    : {'OK' if canvas_ok else 'no data'}")
        else:
            print(f"      WARNING: no Vicon frames received after {VICON_CONNECT_TIMEOUT:.0f}s.")
            print("      Running in teleop-only mode (draw mode requires Vicon).\n")
    print()

    try:
        run(vicon, robot)
    finally:
        if vicon is not None:
            vicon.stop()


if __name__ == "__main__":
    main()
