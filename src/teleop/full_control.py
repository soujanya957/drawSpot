"""
Full Control — Base + Arm + Gripper + Pick
===========================================
Everything in teleop_full plus gripper open/close, arm stow, and
click-to-pick from the hand camera.

BASE controls (always active):
    w / s       — forward / backward
    a / d       — strafe left / right
    q / e       — yaw left / right
    SPACE       — stop base

ARM controls (toggle with TAB):
    i / k       — arm +X / -X  (forward / back)
    j / l       — arm +Y / -Y  (left / right)
    u / o       — arm +Z / -Z  (up / down)
    r / f       — +roll / -roll
    t / g       — +pitch / -pitch
    y / h       — +yaw / -yaw

GRIPPER & ARM:
    [           — open gripper
    ]           — close gripper
    z           — stow arm

PICK:
    n           — capture hand-camera image, click to pick object

OTHER:
    TAB         — toggle arm mode ON/OFF
    p           — print current arm pose
    ESC / x     — stow arm, sit, exit

Run from repo root:
    python -m src.manipulation.full_control

Requirements:
    pip install bosdyn-client bosdyn-mission bosdyn-api opencv-python
    Edit config/robot_config.py with your robot's IP, username, and password.
"""

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
from bosdyn.api import arm_command_pb2, geometry_pb2, manipulation_api_pb2
from bosdyn.api import robot_command_pb2, synchronized_command_pb2
from bosdyn.api.geometry_pb2 import Quaternion, SE3Pose, Vec3
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import BODY_FRAME_NAME
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.robot_command import (
    RobotCommandBuilder,
    RobotCommandClient,
    blocking_stand,
)
from google.protobuf import duration_pb2

sys.path.insert(0, __file__.split("/src/")[0])
from config.robot_config import (
    ARM_FRAME,
    BASE_ROTATION_MAX,
    BASE_VELOCITY_MAX,
    PASSWORD,
    ROBOT_IP,
    USERNAME,
)

BASE_CMD_DUR = 0.5  # seconds
ARM_STEP_POS = 0.02  # metres per key press
ARM_STEP_ROT = math.radians(3)
HAND_COLOR_SOURCE = "hand_color_image"

# ---------------------------------------------------------------------------
# Keyboard-driven pixel selector (replaces mouse callback — reliable on macOS)
# ---------------------------------------------------------------------------


def _select_pixel(img):
    """
    Show img with a keyboard-moveable crosshair.
    Returns (x, y) on ENTER/SPACE, or None on q/ESC.

    Controls:
        Arrow keys  — move 1 px
        WASD        — move 10 px
        ENTER/SPACE — confirm
        q / ESC     — cancel
    """
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

        raw = cv2.waitKeyEx(0)   # full key code so arrow keys are distinguishable
        key = raw & 0xFF

        if key == 13 or key == ord(' '):          # ENTER or SPACE — confirm
            cv2.destroyWindow(win)
            return (cx, cy)
        if key == ord('q') or key == 27:          # q or ESC — cancel
            cv2.destroyWindow(win)
            return None

        # Arrow keys (platform raw codes) — 1 px
        if   raw == 2424832: cx = max(0,   cx - 1)   # left
        elif raw == 2555904: cx = min(w-1, cx + 1)   # right
        elif raw == 2490368: cy = max(0,   cy - 1)   # up
        elif raw == 2621440: cy = min(h-1, cy + 1)   # down
        # WASD — 10 px
        elif key == ord('a'): cx = max(0,   cx - 10)
        elif key == ord('d'): cx = min(w-1, cx + 10)
        elif key == ord('w'): cy = max(0,   cy - 10)
        elif key == ord('s'): cy = min(h-1, cy + 10)


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
        w=cr * cp * cy + sr * sp * sy,
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
    )


def _get_key(timeout=0.05):
    """Return the next key press as a string, or None if nothing arrived."""
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
# Robot commands
# ---------------------------------------------------------------------------


def send_arm_cartesian(
    cmd_client, x, y, z, roll, pitch, yaw, frame_name=BODY_FRAME_NAME
):
    cmd = robot_command_pb2.RobotCommand()
    arm_cart = cmd.synchronized_command.arm_command.arm_cartesian_command
    arm_cart.root_frame_name = frame_name
    point = arm_cart.pose_trajectory_in_task.points.add()
    point.pose.position.x = x
    point.pose.position.y = y
    point.pose.position.z = z
    rot = _rpy_to_quat(roll, pitch, yaw)
    point.pose.rotation.x = rot.x
    point.pose.rotation.y = rot.y
    point.pose.rotation.z = rot.z
    point.pose.rotation.w = rot.w
    point.time_since_reference.CopyFrom(_dur(0.4))
    cmd_client.robot_command(cmd)


def stow_arm(cmd_client):
    cmd_client.robot_command(RobotCommandBuilder.arm_stow_command())
    print("Arm stowed.")


def open_gripper(cmd_client):
    cmd_client.robot_command(RobotCommandBuilder.claw_gripper_open_command())
    print("Gripper open.")


def close_gripper(cmd_client):
    cmd_client.robot_command(RobotCommandBuilder.claw_gripper_close_command())
    print("Gripper closed.")


# ---------------------------------------------------------------------------
# Pick object
# ---------------------------------------------------------------------------


def _capture_image(img_client):
    req = build_image_request(HAND_COLOR_SOURCE, quality_percent=75)
    resp = img_client.get_image([req])[0]
    raw = np.frombuffer(resp.shot.image.data, dtype=np.uint8)
    img = cv2.imdecode(raw, cv2.IMREAD_COLOR)
    return img, resp


def _wait_for_grasp(manip_client, cmd_id, timeout=15.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        fb = manip_client.manipulation_api_feedback_command(
            manipulation_api_feedback_request=manipulation_api_pb2.ManipulationApiFeedbackRequest(
                manipulation_cmd_id=cmd_id
            )
        )
        state = fb.current_state
        name = manipulation_api_pb2.ManipulationFeedbackState.Name(state)
        print(f"  Grasp state: {name}        ", end="\r")
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


def emergency_stop(cmd_client):
    """Stop base, stow arm, sit. Robot stays powered on."""
    print("\n!! EMERGENCY STOP !!")
    cmd_client.robot_command(RobotCommandBuilder.stop_command())
    time.sleep(0.5)
    cmd_client.robot_command(RobotCommandBuilder.arm_stow_command())
    time.sleep(2)
    cmd_client.robot_command(RobotCommandBuilder.synchro_sit_command())
    time.sleep(2)
    print("Emergency stop complete — robot sitting, arm stowed.")


def do_pick(cmd_client, img_client, manip_client, arm_pose):
    """
    Move arm to survey pose, show live hand-camera feed while user
    adjusts arm with keys, then SPACE to freeze frame and click to pick.

    arm_pose: [ax, ay, az, aroll, apitch, ayaw] — updated in-place on return.
    All key input during live view goes through cv2.waitKey (not terminal).
    """
    global g_image_click, g_image_display

    # Move to survey pose: arm forward, camera pointing straight down
    print("Moving arm to survey position…")
    ax, ay, az = 0.7, 0.0, 0.0
    aroll, apitch, ayaw = 0.0, math.pi / 2, 0.0
    send_arm_cartesian(cmd_client, ax, ay, az, aroll, apitch, ayaw)
    time.sleep(3)

    # ── Phase 1: live view + arm adjustment ─────────────────────────────
    PICK_ARM_KEYS = {
        ord('i'): ("ax",     +ARM_STEP_POS), ord('k'): ("ax",     -ARM_STEP_POS),
        ord('j'): ("ay",     +ARM_STEP_POS), ord('l'): ("ay",     -ARM_STEP_POS),
        ord('u'): ("az",     +ARM_STEP_POS), ord('o'): ("az",     -ARM_STEP_POS),
        ord('r'): ("aroll",  +ARM_STEP_ROT), ord('f'): ("aroll",  -ARM_STEP_ROT),
        ord('t'): ("apitch", +ARM_STEP_ROT), ord('g'): ("apitch", -ARM_STEP_ROT),
        ord('y'): ("ayaw",   +ARM_STEP_ROT), ord('h'): ("ayaw",   -ARM_STEP_ROT),
    }

    live_win = "Hand Camera  |  i/k j/l u/o=xyz  r/f t/g y/h=rot  SPACE=confirm  q=cancel"
    cv2.namedWindow(live_win)
    print(f"\nLive camera open. Adjust arm, then press SPACE to confirm.")

    last_img_response = None

    while True:
        img, img_response = _capture_image(img_client)
        last_img_response = img_response
        cv2.imshow(live_win, img)

        key = cv2.waitKey(50) & 0xFF   # ~20 fps refresh

        if key == ord('q') or key == ord('Q'):
            cv2.destroyAllWindows()
            arm_pose[:] = [ax, ay, az, aroll, apitch, ayaw]
            print("Pick cancelled.")
            return

        if key == ord(' ') or key == 13:   # SPACE or ENTER — freeze and switch to click
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

    # ── Phase 2: freeze the same window, switch to click mode ───────────
    # Capture one clean frame now that the arm has settled
    cv2.destroyAllWindows()
    frozen_img, frozen_response = _capture_image(img_client)

    print("Use arrow keys (1px) or WASD (10px) to position crosshair, ENTER to confirm.")
    result = _select_pixel(frozen_img)
    if result is None:
        print("Pick cancelled.")
        return

    px, py = result
    print(f"Picking at pixel ({px}, {py})…")

    pick_req = manipulation_api_pb2.PickObjectInImage(
        pixel_xy=geometry_pb2.Vec2(x=float(px), y=float(py)),
        transforms_snapshot_for_camera=frozen_response.shot.transforms_snapshot,
        frame_name_image_sensor=frozen_response.shot.frame_name_image_sensor,
        camera_model=frozen_response.source.pinhole,
    )
    grasp_request = manipulation_api_pb2.ManipulationApiRequest(
        pick_object_in_image=pick_req)
    resp = manip_client.manipulation_api_command(manipulation_api_request=grasp_request)

    final = _wait_for_grasp(manip_client, resp.manipulation_cmd_id)
    if final == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED:
        print("Grasp SUCCEEDED!")
    else:
        name = manipulation_api_pb2.ManipulationFeedbackState.Name(final) if final else "timeout"
        print(f"Grasp ended: {name}")


# ---------------------------------------------------------------------------
# Main control loop
# ---------------------------------------------------------------------------


def run(robot):
    print("[1/6] Acquiring clients…")
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    cmd_client = robot.ensure_client(RobotCommandClient.default_service_name)
    img_client = robot.ensure_client(ImageClient.default_service_name)
    manip_client = robot.ensure_client(ManipulationApiClient.default_service_name)
    estop_client = robot.ensure_client(EstopClient.default_service_name)

    print("[2/6] Setting up E-Stop…")
    estop_endpoint = EstopEndpoint(
        estop_client, name="full_control_estop", estop_timeout=9.0
    )
    estop_endpoint.force_simple_setup()
    print("[3/6] E-Stop configured. Acquiring lease…")

    with (
        EstopKeepAlive(estop_endpoint),
        LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True),
    ):
        print("[4/6] Lease acquired. Powering on motors…")
        robot.power_on(timeout_sec=20)
        print("[5/6] Motors on. Standing up…")
        blocking_stand(cmd_client, timeout_sec=10)
        print("[6/6] Standing. Ready.\n")

        print("━" * 54)
        print("  BASE   w/s=fwd  a/d=strafe  q/e=yaw  SPC=stop")
        print("  ARM    TAB to toggle | i/k j/l u/o = xyz")
        print("         r/f t/g y/h = roll/pitch/yaw")
        print("  GRIP   [ = open   ] = close")
        print("  ARM    z = stow")
        print("  PICK   n = survey pose → adjust → SPACE → click")
        print("  ESTOP  ESC = stop + stow + sit  (stays powered)")
        print("  EXIT   x   = clean exit")
        print("━" * 54 + "\n")

        ax, ay, az = 0.6, 0.0, 0.3
        aroll, apitch, ayaw = 0.0, 0.0, 0.0
        arm_pose = [ax, ay, az, aroll, apitch, ayaw]  # shared with do_pick
        arm_mode = False
        vx = vy = vyaw = 0.0

        BASE_KEYS = {
            "w": (BASE_VELOCITY_MAX, 0, 0),
            "s": (-BASE_VELOCITY_MAX, 0, 0),
            "a": (0, BASE_VELOCITY_MAX, 0),
            "d": (0, -BASE_VELOCITY_MAX, 0),
            "q": (0, 0, BASE_ROTATION_MAX),
            "e": (0, 0, -BASE_ROTATION_MAX),
            " ": (0, 0, 0),
        }

        ARM_KEYS = {
            "i": ("ax", +ARM_STEP_POS),
            "k": ("ax", -ARM_STEP_POS),
            "l": ("ay", -ARM_STEP_POS),
            "j": ("ay", +ARM_STEP_POS),
            "u": ("az", +ARM_STEP_POS),
            "o": ("az", -ARM_STEP_POS),
            "r": ("aroll", +ARM_STEP_ROT),
            "f": ("aroll", -ARM_STEP_ROT),
            "t": ("apitch", +ARM_STEP_ROT),
            "g": ("apitch", -ARM_STEP_ROT),
            "y": ("ayaw", +ARM_STEP_ROT),
            "h": ("ayaw", -ARM_STEP_ROT),
        }

        while True:
            key = _get_key()
            if key is None:
                continue

            # --- single-action keys ---
            if key == "\x1b":               # ESC — emergency stop
                emergency_stop(cmd_client)
                continue
            if key == "x":                  # clean exit
                break
            if key == "\t":                 # TAB — toggle arm mode
                arm_mode = not arm_mode
                print(f"ARM mode: {'ON' if arm_mode else 'OFF'}")
                continue
            if key == "p":
                print(f"arm  x={ax:.3f} y={ay:.3f} z={az:.3f}  "
                      f"roll={math.degrees(aroll):.1f}°  "
                      f"pitch={math.degrees(apitch):.1f}°  "
                      f"yaw={math.degrees(ayaw):.1f}°")
                continue
            if key == "z":
                stow_arm(cmd_client)
                continue
            if key == "[":
                open_gripper(cmd_client)
                continue
            if key == "]":
                close_gripper(cmd_client)
                continue
            if key == "n":
                do_pick(cmd_client, img_client, manip_client, arm_pose)
                ax, ay, az, aroll, apitch, ayaw = arm_pose
                continue

            # --- base ---
            if key in BASE_KEYS:
                dvx, dvy, dvyaw = BASE_KEYS[key]
                base_cmd = RobotCommandBuilder.synchro_velocity_command(
                    v_x=dvx, v_y=dvy, v_rot=dvyaw)
                cmd_client.robot_command(
                    command=base_cmd, end_time_secs=time.time() + BASE_CMD_DUR)
                continue

            # --- arm ---
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


def main():
    sdk = bosdyn.client.create_standard_sdk("FullControl")
    robot = sdk.create_robot(ROBOT_IP)
    robot.authenticate(USERNAME, PASSWORD)
    robot.time_sync.wait_for_sync()
    run(robot)


if __name__ == "__main__":
    main()
