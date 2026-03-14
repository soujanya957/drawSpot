"""
Vicon Draw
==========
Full teleop (base + arm + gripper + pick) with live Vicon position status
and a canvas draw mode where the arm follows a UV target on the physical canvas.

Modes
-----
  (default)  BASE only  — WASD/QE move the base
  TAB        ARM mode   — IJKL/UO/RTFGYH control arm in body frame
  b          DRAW mode  — IJKL move a UV target on canvas; arm follows

Draw mode
---------
  i / k      canvas -V / +V  (forward / back along canvas height axis)
  j / l      canvas -U / +U  (left / right along canvas width axis)
  [          pen DOWN  (gripper at BRUSH_LENGTH_MM above canvas surface)
  ]          pen UP    (gripper lifted an extra PEN_UP_EXTRA_MM above that)

  The arm is commanded in the odom frame using the Vicon canvas geometry.
  If the UV target is outside the arm's workspace the command is skipped
  and the reachable UV bounding box is printed instead.

Outside draw mode, [ / ] open / close the gripper as usual.

Run:
    python -m src.vicon.vicon_draw --vicon HOST:PORT
    python -m src.vicon.vicon_draw --mock          # no Vicon hardware
"""

import argparse
import math
import select
import sys
import termios
import time
import tty

import cv2
import numpy as np

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
from google.protobuf import duration_pb2

sys.path.insert(0, __file__.split("/src/")[0])
from config.robot_config import (
    BASE_ROTATION_MAX, BASE_VELOCITY_MAX,
    PASSWORD, ROBOT_IP, USERNAME,
    VICON_ADDRESS,
)
from draw.vicon.client import MockViconClient, ViconClient
from draw.vicon.transform import canvas_to_world, clamp_uv

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

BRUSH_LENGTH_MM  = 150.0        # brush extends this far below gripper tip
PEN_UP_EXTRA_MM  =  60.0        # extra lift when pen is raised
DRAW_STEP_UV     =   0.02       # UV step per keypress in draw mode

ARM_REACH_MAX_M  = 0.85
ARM_REACH_MIN_M  = 0.15

BASE_CMD_DUR     = 0.5          # seconds velocity command stays active
ARM_STEP_POS     = 0.02         # metres per arm key press
ARM_STEP_ROT     = math.radians(3)

HAND_COLOR_SOURCE = "hand_color_image"
STATUS_INTERVAL   = 0.5         # seconds between status prints

# ---------------------------------------------------------------------------
# Low-level helpers
# ---------------------------------------------------------------------------


def _dur(secs):
    return duration_pb2.Duration(seconds=int(secs), nanos=int((secs - int(secs)) * 1e9))


def _rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(
        w=cr * cp * cy + sr * sp * sy,
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
    )


def _rotation_matrix_from_quat(q):
    qx, qy, qz, qw = q
    return np.array([
        [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
    ], dtype=float)


def _world_to_body(target_mm, body_pos_mm, body_quat):
    """Vicon world-frame position (mm) → Spot body frame (m)."""
    R = _rotation_matrix_from_quat(body_quat)
    body_mm = R.T @ (target_mm - body_pos_mm)
    return body_mm / 1000.0


def _check_reach(pos_body_m):
    """Returns (ok: bool, reason: str)."""
    dist = float(np.linalg.norm(pos_body_m))
    if dist > ARM_REACH_MAX_M:
        return False, f"too far ({dist:.3f} m > {ARM_REACH_MAX_M} m)"
    if dist < ARM_REACH_MIN_M:
        return False, f"too close ({dist:.3f} m < {ARM_REACH_MIN_M} m)"
    if pos_body_m[0] < 0.0:
        return False, f"behind body (x={pos_body_m[0]:.3f} m)"
    return True, "ok"


def _get_key(timeout=0.05):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ---------------------------------------------------------------------------
# Vicon status display
# ---------------------------------------------------------------------------


def _fmt_rb(rb, label):
    if rb is None:
        return f"no data from Vicon for {label}"
    if rb.occluded:
        return f"occluded (markers hidden) [{label}]"
    x, y, z = rb.position
    return f"({x:+8.1f}, {y:+8.1f}, {z:+8.1f}) mm"


def _print_status(frame, arm_mode, draw_mode, draw_u, draw_v, pen_down):
    if frame is None:
        print("  [Vicon] waiting for first frame…")
        return

    if frame.canvas is None:
        canvas_str = "no data from Vicon for canvas"
    elif not frame.canvas.is_valid():
        canvas_str = "canvas markers partially occluded"
    else:
        c = frame.canvas
        cx, cy, cz = (c.corners[0] + c.corners[1] + c.corners[2] + c.corners[3]) / 4
        canvas_str = f"({cx:+8.1f}, {cy:+8.1f}, {cz:+8.1f}) mm  [{c.width_mm:.0f}×{c.height_mm:.0f} mm]"

    mode_str = "DRAW" if draw_mode else ("ARM" if arm_mode else "BASE")
    draw_info = f"  UV ({draw_u:.3f}, {draw_v:.3f})  pen={'DOWN' if pen_down else 'UP '}" if draw_mode else ""

    print(
        f"  Spot body : {_fmt_rb(frame.spot_body, 'Spot')}\n"
        f"  Spot EE   : {_fmt_rb(frame.spot_ee,   'SpotEE')}\n"
        f"  Canvas    : {canvas_str}\n"
        f"  Mode      : {mode_str}{draw_info}"
    )


# ---------------------------------------------------------------------------
# Robot commands — arm
# ---------------------------------------------------------------------------


def send_arm_cartesian(cmd_client, x, y, z, roll, pitch, yaw, frame_name=BODY_FRAME_NAME):
    cmd = robot_command_pb2.RobotCommand()
    arm_cart = cmd.synchronized_command.arm_command.arm_cartesian_command
    arm_cart.root_frame_name = frame_name
    point = arm_cart.pose_trajectory_in_task.points.add()
    point.pose.position.x = x
    point.pose.position.y = y
    point.pose.position.z = z
    point.pose.rotation.CopyFrom(_rpy_to_quat(roll, pitch, yaw))
    point.time_since_reference.CopyFrom(_dur(0.4))
    cmd_client.robot_command(cmd)


def _send_draw_command(cmd_client, frame, draw_u, draw_v, pen_down):
    """
    Convert canvas UV → Vicon world → odom frame, check IK, send arm command.
    Returns True on success, False if skipped.
    """
    if frame is None or frame.canvas is None or not frame.canvas.is_valid():
        print("  [DRAW] No canvas visible — can't command.")
        return False
    if frame.spot_body is None or frame.spot_body.occluded:
        print("  [DRAW] Spot body occluded — can't check IK.")
        return False

    z_offset = BRUSH_LENGTH_MM + (0.0 if pen_down else PEN_UP_EXTRA_MM)
    target_mm = canvas_to_world(draw_u, draw_v, frame.canvas, z_offset_mm=z_offset)

    # IK check in body frame
    pos_body_m = _world_to_body(
        target_mm, frame.spot_body.position, frame.spot_body.rotation_quat
    )
    ok, reason = _check_reach(pos_body_m)
    if not ok:
        print(f"  [DRAW] OUT OF IK REACH — {reason}")
        _print_reach_box(frame)
        return False

    # Send in odom frame (Vicon world ≈ odom for this system)
    x, y, z = target_mm / 1000.0
    rot = Quaternion(w=0.7071068, x=0.0, y=0.7071068, z=0.0)   # pitch = 90°
    cmd = robot_command_pb2.RobotCommand()
    arm_cart = cmd.synchronized_command.arm_command.arm_cartesian_command
    arm_cart.root_frame_name = "odom"
    point = arm_cart.pose_trajectory_in_task.points.add()
    point.pose.position.x = x
    point.pose.position.y = y
    point.pose.position.z = z
    point.pose.rotation.CopyFrom(rot)
    point.time_since_reference.CopyFrom(_dur(0.15))
    cmd_client.robot_command(cmd)
    return True


def _print_reach_box(frame):
    """Sample the canvas at a 20×20 grid and print the reachable UV range."""
    if frame.canvas is None or not frame.canvas.is_valid():
        return
    if frame.spot_body is None or frame.spot_body.occluded:
        return

    reachable = []
    N = 20
    for i in range(N + 1):
        for j in range(N + 1):
            u, v = i / N, j / N
            target_mm = canvas_to_world(u, v, frame.canvas, z_offset_mm=BRUSH_LENGTH_MM)
            pos_body_m = _world_to_body(
                target_mm, frame.spot_body.position, frame.spot_body.rotation_quat
            )
            if _check_reach(pos_body_m)[0]:
                reachable.append((u, v))

    if reachable:
        us = [p[0] for p in reachable]
        vs = [p[1] for p in reachable]
        print(
            f"  [DRAW] Reachable canvas region:"
            f"  U [{min(us):.2f} – {max(us):.2f}]"
            f"  V [{min(vs):.2f} – {max(vs):.2f}]"
        )
    else:
        print("  [DRAW] No canvas region reachable — walk closer to canvas.")


# ---------------------------------------------------------------------------
# Robot commands — gripper / stow / estop
# ---------------------------------------------------------------------------


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
    print("Emergency stop complete — robot sitting, arm stowed.")


# ---------------------------------------------------------------------------
# Pick object (identical to full_control.py)
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
        cv2.putText(display, f"({cx}, {cy})", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow(win, display)
        raw = cv2.waitKeyEx(0)
        key = raw & 0xFF
        if key == 13 or key == ord(' '):
            cv2.destroyWindow(win)
            return (cx, cy)
        if key == ord('q') or key == 27:
            cv2.destroyWindow(win)
            return None
        if   raw == 2424832: cx = max(0,   cx - 1)
        elif raw == 2555904: cx = min(w-1, cx + 1)
        elif raw == 2490368: cy = max(0,   cy - 1)
        elif raw == 2621440: cy = min(h-1, cy + 1)
        elif key == ord('a'): cx = max(0,   cx - 10)
        elif key == ord('d'): cx = min(w-1, cx + 10)
        elif key == ord('w'): cy = max(0,   cy - 10)
        elif key == ord('s'): cy = min(h-1, cy + 10)


def _capture_image(img_client):
    req = build_image_request(HAND_COLOR_SOURCE, quality_percent=75)
    resp = img_client.get_image([req])[0]
    raw = np.frombuffer(resp.shot.image.data, dtype=np.uint8)
    return cv2.imdecode(raw, cv2.IMREAD_COLOR), resp


def _wait_for_grasp(manip_client, cmd_id, timeout=15.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        fb = manip_client.manipulation_api_feedback_command(
            manipulation_api_feedback_request=manipulation_api_pb2.ManipulationApiFeedbackRequest(
                manipulation_cmd_id=cmd_id
            )
        )
        state = fb.current_state
        print(f"  Grasp state: {manipulation_api_pb2.ManipulationFeedbackState.Name(state)}   ", end="\r")
        if state in (
            manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED,
            manipulation_api_pb2.MANIP_STATE_GRASP_FAILED,
            manipulation_api_pb2.MANIP_STATE_GRASP_PLANNING_NO_SOLUTION,
        ):
            print()
            return state
        time.sleep(0.3)
    print("\nGrasp timed out.")
    return None


def do_pick(cmd_client, img_client, manip_client, arm_pose):
    print("  Moving arm to survey position…")
    ax, ay, az = 0.7, 0.0, 0.0
    aroll, apitch, ayaw = 0.0, math.pi / 2, 0.0
    send_arm_cartesian(cmd_client, ax, ay, az, aroll, apitch, ayaw)
    time.sleep(3)

    PICK_ARM_KEYS = {
        ord('i'): ("ax", +ARM_STEP_POS), ord('k'): ("ax", -ARM_STEP_POS),
        ord('j'): ("ay", +ARM_STEP_POS), ord('l'): ("ay", -ARM_STEP_POS),
        ord('u'): ("az", +ARM_STEP_POS), ord('o'): ("az", -ARM_STEP_POS),
        ord('r'): ("aroll",  +ARM_STEP_ROT), ord('f'): ("aroll",  -ARM_STEP_ROT),
        ord('t'): ("apitch", +ARM_STEP_ROT), ord('g'): ("apitch", -ARM_STEP_ROT),
        ord('y'): ("ayaw",   +ARM_STEP_ROT), ord('h'): ("ayaw",   -ARM_STEP_ROT),
    }

    live_win = "Hand Camera  |  ijkl/uo=xyz  rf/tg/yh=rot  SPACE=confirm  q=cancel"
    cv2.namedWindow(live_win)
    print("  Live camera open. Adjust arm, then press SPACE.")

    while True:
        img, img_response = _capture_image(img_client)
        cv2.imshow(live_win, img)
        key = cv2.waitKey(50) & 0xFF
        if key == ord('q') or key == ord('Q'):
            cv2.destroyAllWindows()
            arm_pose[:] = [ax, ay, az, aroll, apitch, ayaw]
            print("  Pick cancelled.")
            return
        if key == ord(' ') or key == 13:
            break
        if key in PICK_ARM_KEYS:
            attr, delta = PICK_ARM_KEYS[key]
            if   attr == "ax":     ax     += delta
            elif attr == "ay":     ay     += delta
            elif attr == "az":     az     += delta
            elif attr == "aroll":  aroll  += delta
            elif attr == "apitch": apitch += delta
            elif attr == "ayaw":   ayaw   += delta
            send_arm_cartesian(cmd_client, ax, ay, az, aroll, apitch, ayaw)

    arm_pose[:] = [ax, ay, az, aroll, apitch, ayaw]
    cv2.destroyAllWindows()
    frozen_img, frozen_response = _capture_image(img_client)

    print("  Position crosshair with arrows/WASD, ENTER to confirm.")
    result = _select_pixel(frozen_img)
    if result is None:
        print("  Pick cancelled.")
        return

    px, py = result
    print(f"  Picking at pixel ({px}, {py})…")
    pick_req = manipulation_api_pb2.PickObjectInImage(
        pixel_xy=geometry_pb2.Vec2(x=float(px), y=float(py)),
        transforms_snapshot_for_camera=frozen_response.shot.transforms_snapshot,
        frame_name_image_sensor=frozen_response.shot.frame_name_image_sensor,
        camera_model=frozen_response.source.pinhole,
    )
    resp = manip_client.manipulation_api_command(
        manipulation_api_request=manipulation_api_pb2.ManipulationApiRequest(
            pick_object_in_image=pick_req
        )
    )
    final = _wait_for_grasp(manip_client, resp.manipulation_cmd_id)
    if final == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED:
        print("  Grasp SUCCEEDED!")
    else:
        name = manipulation_api_pb2.ManipulationFeedbackState.Name(final) if final else "timeout"
        print(f"  Grasp ended: {name}")


# ---------------------------------------------------------------------------
# Main control loop
# ---------------------------------------------------------------------------


def run(vicon_client, robot):
    print("[1/6] Acquiring clients…")
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    cmd_client   = robot.ensure_client(RobotCommandClient.default_service_name)
    img_client   = robot.ensure_client(ImageClient.default_service_name)
    manip_client = robot.ensure_client(ManipulationApiClient.default_service_name)
    estop_client = robot.ensure_client(EstopClient.default_service_name)

    print("[2/6] Setting up E-Stop…")
    estop_ep = EstopEndpoint(estop_client, name="vicon_draw_estop", estop_timeout=9.0)
    estop_ep.force_simple_setup()
    print("[3/6] E-Stop configured. Acquiring lease…")

    with (
        EstopKeepAlive(estop_ep),
        LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True),
    ):
        print("[4/6] Lease acquired. Powering on motors…")
        robot.power_on(timeout_sec=20)
        print("[5/6] Motors on. Standing up…")
        blocking_stand(cmd_client, timeout_sec=10)
        print("[6/6] Standing. Ready.\n")

        print("━" * 60)
        print("  BASE   w/s=fwd  a/d=strafe  q/e=yaw  SPC=stop")
        print("  ARM    TAB to toggle | i/k j/l u/o = xyz")
        print("         r/f t/g y/h = roll/pitch/yaw")
        print("  DRAW   b to toggle | i/k=±V  j/l=±U on canvas")
        print("         [ = pen down   ] = pen up")
        print("  GRIP   [ = open   ] = close  (when not in draw mode)")
        print("  ARM    z = stow  (also exits draw mode)")
        print("  PICK   n = pick object from hand camera")
        print("  ESTOP  ESC = stop + stow + sit")
        print("  EXIT   x   = clean exit")
        print("━" * 60 + "\n")

        # Arm state
        ax, ay, az         = 0.6, 0.0, 0.3
        aroll, apitch, ayaw = 0.0, 0.0, 0.0
        arm_pose = [ax, ay, az, aroll, apitch, ayaw]

        # Draw state
        draw_u, draw_v = 0.5, 0.5
        pen_down = False

        arm_mode  = False
        draw_mode = False

        BASE_KEYS = {
            "w": ( BASE_VELOCITY_MAX, 0,                0),
            "s": (-BASE_VELOCITY_MAX, 0,                0),
            "a": (0,  BASE_VELOCITY_MAX,                0),
            "d": (0, -BASE_VELOCITY_MAX,                0),
            "q": (0,  0,  BASE_ROTATION_MAX),
            "e": (0,  0, -BASE_ROTATION_MAX),
            " ": (0,  0,  0),
        }

        ARM_KEYS = {
            "i": ("ax",     +ARM_STEP_POS), "k": ("ax",     -ARM_STEP_POS),
            "j": ("ay",     +ARM_STEP_POS), "l": ("ay",     -ARM_STEP_POS),
            "u": ("az",     +ARM_STEP_POS), "o": ("az",     -ARM_STEP_POS),
            "r": ("aroll",  +ARM_STEP_ROT), "f": ("aroll",  -ARM_STEP_ROT),
            "t": ("apitch", +ARM_STEP_ROT), "g": ("apitch", -ARM_STEP_ROT),
            "y": ("ayaw",   +ARM_STEP_ROT), "h": ("ayaw",   -ARM_STEP_ROT),
        }

        DRAW_KEYS = {
            "i": ("draw_v", -DRAW_STEP_UV),   # forward on canvas
            "k": ("draw_v", +DRAW_STEP_UV),   # back
            "j": ("draw_u", -DRAW_STEP_UV),   # left
            "l": ("draw_u", +DRAW_STEP_UV),   # right
        }

        last_status_t = 0.0

        while True:
            # ── Periodic Vicon status ────────────────────────────────────
            now = time.time()
            if now - last_status_t >= STATUS_INTERVAL:
                print("─" * 60)
                _print_status(
                    vicon_client.latest_frame,
                    arm_mode, draw_mode, draw_u, draw_v, pen_down,
                )
                last_status_t = now

            key = _get_key()
            if key is None:
                continue

            # ── Universal keys ───────────────────────────────────────────
            if key == "\x1b":
                emergency_stop(cmd_client)
                continue
            if key == "x":
                break
            if key == "z":
                stow_arm(cmd_client)
                draw_mode = False
                continue
            if key == "n":
                do_pick(cmd_client, img_client, manip_client, arm_pose)
                ax, ay, az, aroll, apitch, ayaw = arm_pose
                continue
            if key == "p":
                print(f"  arm  x={ax:.3f} y={ay:.3f} z={az:.3f}  "
                      f"roll={math.degrees(aroll):.1f}°  "
                      f"pitch={math.degrees(apitch):.1f}°  "
                      f"yaw={math.degrees(ayaw):.1f}°")
                continue

            # ── Mode toggles ─────────────────────────────────────────────
            if key == "\t":             # TAB — toggle arm mode, exit draw
                draw_mode = False
                arm_mode  = not arm_mode
                print(f"  ARM mode: {'ON' if arm_mode else 'OFF'}")
                continue
            if key == "b":             # b — toggle draw mode, exit arm
                arm_mode  = False
                draw_mode = not draw_mode
                if draw_mode:
                    print(f"  DRAW mode ON  |  UV ({draw_u:.3f}, {draw_v:.3f})")
                    frame = vicon_client.latest_frame
                    if frame:
                        _print_reach_box(frame)
                else:
                    print("  DRAW mode OFF")
                continue

            # ── Draw mode keys ───────────────────────────────────────────
            if draw_mode:
                frame = vicon_client.latest_frame

                if key == "[":
                    pen_down = True
                    print("  Pen DOWN")
                    _send_draw_command(cmd_client, frame, draw_u, draw_v, pen_down)
                    continue
                if key == "]":
                    pen_down = False
                    print("  Pen UP")
                    _send_draw_command(cmd_client, frame, draw_u, draw_v, pen_down)
                    continue
                if key in DRAW_KEYS:
                    attr, delta = DRAW_KEYS[key]
                    if attr == "draw_u":
                        draw_u = float(np.clip(draw_u + delta, 0.0, 1.0))
                    else:
                        draw_v = float(np.clip(draw_v + delta, 0.0, 1.0))
                    _send_draw_command(cmd_client, frame, draw_u, draw_v, pen_down)
                    continue

            # ── Base ─────────────────────────────────────────────────────
            if key in BASE_KEYS:
                dvx, dvy, dvyaw = BASE_KEYS[key]
                cmd_client.robot_command(
                    command=RobotCommandBuilder.synchro_velocity_command(
                        v_x=dvx, v_y=dvy, v_rot=dvyaw
                    ),
                    end_time_secs=time.time() + BASE_CMD_DUR,
                )
                continue

            # ── Gripper (outside draw mode) ───────────────────────────────
            if key == "[":
                open_gripper(cmd_client)
                continue
            if key == "]":
                close_gripper(cmd_client)
                continue

            # ── Arm mode ─────────────────────────────────────────────────
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
        print("Done. Goodbye.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main():
    ap = argparse.ArgumentParser(description="Vicon Draw — full teleop + canvas draw mode")
    ap.add_argument("--vicon", metavar="HOST:PORT",
                    help=f"Vicon DataStream address (default: VICON_ADDRESS from env = {VICON_ADDRESS})")
    ap.add_argument("--mock", action="store_true",
                    help="Use simulated Vicon data (no hardware needed)")
    args = ap.parse_args()

    if args.mock:
        vicon = MockViconClient()
    else:
        vicon = ViconClient(host=args.vicon or VICON_ADDRESS)

    vicon.start()
    print("Waiting for first Vicon frame…")
    while vicon.latest_frame is None:
        time.sleep(0.01)
    print("Vicon connected.\n")

    sdk   = bosdyn.client.create_standard_sdk("ViconDraw")
    robot = sdk.create_robot(ROBOT_IP)
    robot.authenticate(USERNAME, PASSWORD)
    robot.time_sync.wait_for_sync()

    try:
        run(vicon, robot)
    finally:
        vicon.stop()


if __name__ == "__main__":
    main()
