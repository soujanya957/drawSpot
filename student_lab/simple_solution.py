"""
simple_solution.py  —  Your First Robot SDK  (SOLUTION)
=========================================================
Reference implementation of simple_stencil.py — all three TODOs filled in.
"""

import os
import time

from dotenv import load_dotenv

import bosdyn.client
import bosdyn.client.util
import bosdyn.client.lease
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.robot_command import (
    RobotCommandBuilder,
    RobotCommandClient,
    blocking_stand,
)

load_dotenv(os.path.join(os.path.dirname(__file__), "..", ".env"))

ROBOT_IP = os.environ["SPOT_IP"]
USERNAME = os.environ["SPOT_USER"]
PASSWORD = os.environ["SPOT_PASSWORD"]

LINEAR_SPEED = 0.5
ANGULAR_SPEED = 0.8


# ─────────────────────────────────────────────────────────────────────────────
# TODO 1  —  Connect to the robot
# ─────────────────────────────────────────────────────────────────────────────


def connect_to_robot():
    sdk = bosdyn.client.create_standard_sdk("SpotTeleop")  # step 1: create SDK
    robot = sdk.create_robot(ROBOT_IP)  # step 2: point at device
    robot.authenticate(USERNAME, PASSWORD)  # step 3: authenticate
    robot.time_sync.wait_for_sync()  # step 4: sync clocks
    print("Connected to Spot.")
    return robot


# ─────────────────────────────────────────────────────────────────────────────
# TODO 2  —  Get a command client
# ─────────────────────────────────────────────────────────────────────────────


def get_command_client(robot):
    return robot.ensure_client(RobotCommandClient.default_service_name)


# ─────────────────────────────────────────────────────────────────────────────
# Pre-written examples
# ─────────────────────────────────────────────────────────────────────────────


def stand_up(robot, cmd_client):
    print("Powering on…")
    robot.power_on(timeout_sec=20)
    blocking_stand(cmd_client, timeout_sec=10)
    print("Spot is standing. Ready!")


def sit_down(robot, cmd_client):
    sit_cmd = RobotCommandBuilder.synchro_sit_command()  # step 1: build
    cmd_client.robot_command(sit_cmd)  # step 2: send
    time.sleep(2)
    robot.power_off(cut_immediately=False, timeout_sec=20)
    print("Powered off. Bye!")


# ─────────────────────────────────────────────────────────────────────────────
# TODO 3  —  Send a velocity command
# ─────────────────────────────────────────────────────────────────────────────


def send_velocity(cmd_client, vx, vy, vyaw, duration=0.6):
    cmd = RobotCommandBuilder.synchro_se2_velocity_command(
        vx, vy, vyaw
    )  # step 1: build
    cmd_client.robot_command(cmd, end_time_secs=time.time() + duration)  # step 2: send


# ─────────────────────────────────────────────────────────────────────────────
# Teleop loop + main
# ─────────────────────────────────────────────────────────────────────────────

CONTROLS = """
Controls
--------
  w / s   — forward / backward
  a / d   — strafe left / right
  q / e   — rotate left / right
  space   — stop
  x       — sit down and quit
"""

KEY_TO_VEL = {
    "w": (LINEAR_SPEED, 0, 0),
    "s": (-LINEAR_SPEED, 0, 0),
    "a": (0, LINEAR_SPEED, 0),
    "d": (0, -LINEAR_SPEED, 0),
    "q": (0, 0, ANGULAR_SPEED),
    "e": (0, 0, -ANGULAR_SPEED),
    " ": (0, 0, 0),
}


def teleop_loop(cmd_client):
    print(CONTROLS)
    while True:
        key = input("Command: ").strip().lower()
        if key == "x":
            break
        if key in KEY_TO_VEL:
            vx, vy, vyaw = KEY_TO_VEL[key]
            send_velocity(cmd_client, vx, vy, vyaw)
        else:
            print(f"  Unknown key '{key}'. See controls above.")


def main():
    bosdyn.client.util.setup_logging(verbose=False)

    robot = connect_to_robot()
    cmd_client = get_command_client(robot)

    lease_client = robot.ensure_client(
        bosdyn.client.lease.LeaseClient.default_service_name
    )

    estop_client = robot.ensure_client(EstopClient.default_service_name)
    estop_ep = EstopEndpoint(estop_client, name="teleop_simple", estop_timeout=9.0)
    estop_ep.force_simple_setup()

    with (
        EstopKeepAlive(estop_ep),
        bosdyn.client.lease.LeaseKeepAlive(
            lease_client, must_acquire=True, return_at_exit=True
        ),
    ):
        stand_up(robot, cmd_client)
        try:
            teleop_loop(cmd_client)
        finally:
            sit_down(robot, cmd_client)


if __name__ == "__main__":
    main()
