"""
Base Teleop (keyboard)
======================
Keyboard teleoperation for Spot's mobile base only — the arm is not touched.

Controls:
    w / s       — move forward / backward
    a / d       — strafe left / right
    q / e       — rotate (yaw) left / right
    SPACE       — stop (zero velocity)
    ESC or x    — sit down, return lease, and exit

Run from the repo root:
    python -m src.navigation.teleop_base

Requirements:
    pip install bosdyn-client bosdyn-mission bosdyn-api
    Edit config/robot_config.py with your robot's IP, username, and password.
"""

import sys
import tty
import termios
import time
import threading

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand
from bosdyn.client.robot_state import RobotStateClient

# Add repo root to path so config can be imported regardless of working dir
sys.path.insert(0, __file__.split("/src/")[0])
from config.robot_config import ROBOT_IP, USERNAME, PASSWORD, BASE_VELOCITY_MAX, BASE_ROTATION_MAX

# Velocity command duration — robot stops if no new command arrives within this window
COMMAND_DURATION = 0.5  # seconds


def _get_key(timeout=0.1):
    """Read one key from stdin without blocking for longer than `timeout` s."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        # Use select to honour the timeout
        import select
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def run_teleop(robot):
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    cmd_client   = robot.ensure_client(RobotCommandClient.default_service_name)

    with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        # Power on and stand
        robot.logger.info("Powering on…")
        robot.power_on(timeout_sec=20)
        blocking_stand(cmd_client, timeout_sec=10)
        robot.logger.info("Standing. Use WASD/QE to move, SPACE to stop, ESC/x to exit.")

        vx = vy = vyaw = 0.0

        while True:
            key = _get_key(timeout=0.05)

            if key is None:
                pass
            elif key in ('\x1b', 'x'):   # ESC or x
                break
            elif key == 'w':
                vx, vy, vyaw = BASE_VELOCITY_MAX, 0, 0
            elif key == 's':
                vx, vy, vyaw = -BASE_VELOCITY_MAX, 0, 0
            elif key == 'a':
                vx, vy, vyaw = 0, BASE_VELOCITY_MAX, 0
            elif key == 'd':
                vx, vy, vyaw = 0, -BASE_VELOCITY_MAX, 0
            elif key == 'q':
                vx, vy, vyaw = 0, 0,  BASE_ROTATION_MAX
            elif key == 'e':
                vx, vy, vyaw = 0, 0, -BASE_ROTATION_MAX
            elif key == ' ':
                vx = vy = vyaw = 0.0

            cmd = RobotCommandBuilder.synchro_velocity_command(
                v_x=vx, v_y=vy, v_rot=vyaw,
                params=None,
                body_height=0.0,
                locomotion_hint=bosdyn.api.spot.robot_command_pb2.HINT_AUTO,
                # duration tells the robot to keep moving for this many seconds
                # without a new command before auto-stopping
            )
            cmd_client.robot_command(command=cmd,
                                     end_time_secs=time.time() + COMMAND_DURATION)

        # Sit before exit
        sit_cmd = RobotCommandBuilder.synchro_sit_command()
        cmd_client.robot_command(sit_cmd)
        time.sleep(2)
        robot.logger.info("Sat down. Goodbye.")


def main():
    import bosdyn.api.spot.robot_command_pb2  # noqa: F401 – needed for HINT_AUTO

    sdk = bosdyn.client.create_standard_sdk("TeleopBase")
    robot = sdk.create_robot(ROBOT_IP)
    robot.authenticate(USERNAME, PASSWORD)
    robot.time_sync.wait_for_sync()

    run_teleop(robot)


if __name__ == "__main__":
    main()
