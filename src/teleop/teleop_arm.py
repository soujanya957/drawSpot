"""
Arm-Only Teleop (keyboard — joint control)
==========================================
Move Spot's arm joint-by-joint from the keyboard. The base does not move.
The arm is controlled in joint-angle space: each key press increments or
decrements one joint by a fixed step.

Joint layout (Spot arm):
    sh0  — shoulder yaw  (j / J)
    sh1  — shoulder pitch (u / U)
    el0  — elbow pitch   (i / I)
    el1  — elbow roll    (o / O)
    wr0  — wrist pitch   (k / K)
    wr1  — wrist roll    (l / L)

Other keys:
    r    — reset to stow position
    p    — print current joint angles
    ESC  — stow arm and exit

Run from the repo root:
    python -m src.manipulation.teleop_arm

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
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api import arm_command_pb2, robot_command_pb2, synchronized_command_pb2

sys.path.insert(0, __file__.split("/src/")[0])
from config.robot_config import ROBOT_IP, USERNAME, PASSWORD

STEP = math.radians(3)   # degrees per key press

# Soft joint limits (radians) — roughly matching Spot arm spec
JOINT_LIMITS = {
    "sh0": (-3.14,  3.14),
    "sh1": (-3.14,  0.52),
    "el0": ( 0.00,  3.14),
    "el1": (-2.79,  2.79),
    "wr0": (-1.83,  1.83),
    "wr1": (-2.87,  2.87),
}

# Stow-like home pose (radians)
HOME_POSE = {
    "sh0":  0.0,
    "sh1": -2.0,
    "el0":  2.6,
    "el1":  0.0,
    "wr0": -0.5,
    "wr1":  0.0,
}


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


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


def send_joint_command(cmd_client, joints, duration=0.5):
    """Send a joint-move command with the given angle dict."""
    arm_joint_pos = arm_command_pb2.ArmJointPosition(
        sh0=arm_command_pb2.JointTarget(position=joints["sh0"]),
        sh1=arm_command_pb2.JointTarget(position=joints["sh1"]),
        el0=arm_command_pb2.JointTarget(position=joints["el0"]),
        el1=arm_command_pb2.JointTarget(position=joints["el1"]),
        wr0=arm_command_pb2.JointTarget(position=joints["wr0"]),
        wr1=arm_command_pb2.JointTarget(position=joints["wr1"]),
    )
    arm_joint_move = arm_command_pb2.ArmJointMoveCommand.Request(
        trajectory=arm_command_pb2.ArmJointTrajectory(
            points=[arm_command_pb2.ArmJointTrajectoryPoint(
                position=arm_joint_pos,
                time_since_reference=bosdyn.client.math_helpers.seconds_to_duration(duration),
            )]
        )
    )
    arm_cmd  = arm_command_pb2.ArmCommand.Request(arm_joint_move_command=arm_joint_move)
    sync_cmd = synchronized_command_pb2.SynchronizedCommand.Request(arm_command=arm_cmd)
    cmd      = robot_command_pb2.RobotCommand(synchronized_command=sync_cmd)
    cmd_client.robot_command(cmd)


def run_teleop(robot):
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    cmd_client   = robot.ensure_client(RobotCommandClient.default_service_name)

    with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        robot.power_on(timeout_sec=20)
        blocking_stand(cmd_client, timeout_sec=10)

        joints = dict(HOME_POSE)
        print("\nArm teleop ready.")
        print("sh0: j/J   sh1: u/U   el0: i/I   el1: o/O   wr0: k/K   wr1: l/L")
        print("r=reset   p=print angles   ESC=exit\n")

        send_joint_command(cmd_client, joints, duration=2.0)
        time.sleep(2)

        KEY_MAP = {
            'j': ("sh0", +STEP), 'J': ("sh0", -STEP),
            'u': ("sh1", +STEP), 'U': ("sh1", -STEP),
            'i': ("el0", +STEP), 'I': ("el0", -STEP),
            'o': ("el1", +STEP), 'O': ("el1", -STEP),
            'k': ("wr0", +STEP), 'K': ("wr0", -STEP),
            'l': ("wr1", +STEP), 'L': ("wr1", -STEP),
        }

        while True:
            key = _get_key()
            if key is None:
                continue
            if key == '\x1b':
                break
            if key == 'r':
                joints = dict(HOME_POSE)
                send_joint_command(cmd_client, joints, duration=2.0)
                print("Reset to home.")
                continue
            if key == 'p':
                print("  ".join(f"{k}={math.degrees(v):.1f}°" for k, v in joints.items()))
                continue
            if key in KEY_MAP:
                joint, delta = KEY_MAP[key]
                lo, hi = JOINT_LIMITS[joint]
                joints[joint] = clamp(joints[joint] + delta, lo, hi)
                send_joint_command(cmd_client, joints, duration=0.3)

        # Stow on exit
        stow_cmd = RobotCommandBuilder.arm_stow_command()
        cmd_client.robot_command(stow_cmd)
        time.sleep(3)
        print("Arm stowed. Goodbye.")


def main():
    import bosdyn.client.math_helpers  # noqa
    sdk   = bosdyn.client.create_standard_sdk("TeleopArm")
    robot = sdk.create_robot(ROBOT_IP)
    robot.authenticate(USERNAME, PASSWORD)
    robot.time_sync.wait_for_sync()
    run_teleop(robot)


if __name__ == "__main__":
    main()
