"""
Full Teleop — Base + Arm Cartesian (keyboard)
=============================================
Simultaneous keyboard control of both Spot's mobile base and its arm
end-effector in Cartesian space.

BASE controls (numpad-style, always active):
    w / s   — base forward / backward
    a / d   — base strafe left / right
    q / e   — base yaw left / right
    SPACE   — stop base

ARM end-effector controls (active when arm mode is on, toggled with TAB):
    i / k   — arm +X / -X  (forward/back in body frame)
    l / j   — arm +Y / -Y  (left/right)
    u / o   — arm +Z / -Z  (up/down)
    r / f   — arm +roll / -roll
    t / g   — arm +pitch / -pitch
    y / h   — arm +yaw / -yaw

Other:
    TAB     — toggle arm control mode (printed in status)
    p       — print current arm pose
    ESC/x   — stow arm, sit, exit

Run from the repo root:
    python -m src.manipulation.teleop_full

Requirements:
    pip install bosdyn-client bosdyn-mission bosdyn-api
    Edit config/robot_config.py with your robot's IP, username, and password.
"""

import sys
import time
import math
import tty
import termios
import select

import bosdyn.client
import bosdyn.client.util
import bosdyn.client.math_helpers as math_helpers
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import (
    RobotCommandClient,
    RobotCommandBuilder,
    blocking_stand,
)
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, ODOM_FRAME_NAME
from bosdyn.api import (
    geometry_pb2,
    arm_command_pb2,
    robot_command_pb2,
    synchronized_command_pb2,
)
from bosdyn.api.geometry_pb2 import SE3Pose, Vec3, Quaternion
from bosdyn.api.spot import robot_command_pb2 as spot_cmd_pb2
from google.protobuf import duration_pb2

sys.path.insert(0, __file__.split("/src/")[0])
from config.robot_config import (
    ROBOT_IP,
    USERNAME,
    PASSWORD,
    BASE_VELOCITY_MAX,
    BASE_ROTATION_MAX,
    ARM_VELOCITY_MAX,
    ARM_FRAME,
)

BASE_CMD_DUR = 0.5  # seconds
ARM_STEP_POS = 0.02  # metres per key press
ARM_STEP_ROT = math.radians(3)  # radians per key press


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


def _seconds_to_duration(secs):
    return duration_pb2.Duration(seconds=int(secs), nanos=int((secs - int(secs)) * 1e9))


def _rpy_to_quaternion(roll, pitch, yaw):
    """Convert roll/pitch/yaw (radians) to a Quaternion protobuf message."""
    cr, sr = math.cos(roll / 2),  math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2),   math.sin(yaw / 2)
    return Quaternion(
        w=cr * cp * cy + sr * sp * sy,
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
    )


def send_arm_cartesian(
    cmd_client, x, y, z, roll, pitch, yaw, frame_name=BODY_FRAME_NAME
):
    """Command the arm end-effector to an absolute Cartesian pose in body frame."""
    cmd = robot_command_pb2.RobotCommand()
    arm_cart = cmd.synchronized_command.arm_command.arm_cartesian_command
    arm_cart.root_frame_name = frame_name

    point = arm_cart.pose_trajectory_in_task.points.add()
    point.pose.position.x = x
    point.pose.position.y = y
    point.pose.position.z = z
    rot = _rpy_to_quaternion(roll, pitch, yaw)
    point.pose.rotation.x = rot.x
    point.pose.rotation.y = rot.y
    point.pose.rotation.z = rot.z
    point.pose.rotation.w = rot.w
    point.time_since_reference.CopyFrom(_seconds_to_duration(0.4))

    cmd_client.robot_command(cmd)


def run_teleop(robot):
    print("[1/6] Acquiring clients…")
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    cmd_client   = robot.ensure_client(RobotCommandClient.default_service_name)
    estop_client = robot.ensure_client(EstopClient.default_service_name)
    print("[2/6] Setting up E-Stop…")
    estop_endpoint = EstopEndpoint(estop_client, name="teleop_full_estop", estop_timeout=9.0)
    estop_endpoint.force_simple_setup()
    print("[3/6] E-Stop configured. Acquiring lease…")

    with EstopKeepAlive(estop_endpoint), \
         LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        print("[4/6] Lease acquired. Powering on motors…")
        robot.power_on(timeout_sec=20)
        print("[5/6] Motors on. Standing up…")
        blocking_stand(cmd_client, timeout_sec=10)
        print("[6/6] Standing. Ready.\n")

        # Arm starting pose (body frame, metres / radians)
        ax, ay, az = 0.6, 0.0, 0.3
        aroll, apitch, ayaw = 0.0, 0.0, 0.0

        arm_mode = False
        vx = vy = vyaw = 0.0

        print("\nFull teleop ready.")
        print("Base: WASD=move, QE=yaw, SPACE=stop")
        print("TAB to toggle ARM mode | ESC/x to exit\n")

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

            if key in ("\x1b", "x"):
                break

            if key == "\t":  # TAB
                arm_mode = not arm_mode
                print(f"ARM mode: {'ON' if arm_mode else 'OFF'}")
                continue

            if key == "p":
                print(
                    f"arm pos  x={ax:.3f} y={ay:.3f} z={az:.3f}  "
                    f"roll={math.degrees(aroll):.1f}° "
                    f"pitch={math.degrees(apitch):.1f}° "
                    f"yaw={math.degrees(ayaw):.1f}°"
                )
                continue

            # --- base ---
            if key in BASE_KEYS:
                vx, vy, vyaw = BASE_KEYS[key]
                cmd = RobotCommandBuilder.synchro_velocity_command(
                    v_x=vx, v_y=vy, v_rot=vyaw
                )
                cmd_client.robot_command(
                    command=cmd, end_time_secs=time.time() + BASE_CMD_DUR
                )

            # --- arm ---
            if arm_mode and key in ARM_KEYS:
                attr, delta = ARM_KEYS[key]
                if attr == "ax":
                    ax += delta
                elif attr == "ay":
                    ay += delta
                elif attr == "az":
                    az += delta
                elif attr == "aroll":
                    aroll += delta
                elif attr == "apitch":
                    apitch += delta
                elif attr == "ayaw":
                    ayaw += delta

                send_arm_cartesian(cmd_client, ax, ay, az, aroll, apitch, ayaw)

        # Clean exit
        stow_cmd = RobotCommandBuilder.arm_stow_command()
        cmd_client.robot_command(stow_cmd)
        time.sleep(2)
        sit_cmd = RobotCommandBuilder.synchro_sit_command()
        cmd_client.robot_command(sit_cmd)
        time.sleep(2)
        print("Stowed and sat. Goodbye.")


def main():
    sdk = bosdyn.client.create_standard_sdk("TeleopFull")
    robot = sdk.create_robot(ROBOT_IP)
    robot.authenticate(USERNAME, PASSWORD)
    robot.time_sync.wait_for_sync()
    run_teleop(robot)


if __name__ == "__main__":
    main()
