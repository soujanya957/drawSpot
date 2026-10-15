"""
simple_stencil.py  —  Your First Robot SDK
===========================================
An SDK (Software Development Kit) is a library that lets you talk to a
piece of hardware without knowing its low-level details.  You just call
functions like  robot.authenticate()  or  cmd_client.robot_command(...)
and the SDK handles the network messages, protocols, and timing for you.

In this exercise you will use Boston Dynamics' Spot SDK to:
  1. Connect to the robot.
  2. Get a client for sending movement commands.
  3. Send velocity commands from your keyboard.

The stand/sit functions, teleop loop, and main() are already written.
Read through them to see how the whole program fits together, then fill
in the three TODO sections.

Running
-------
    python simple_stencil.py

Controls (once running)
-----------------------
    w / s   — forward / backward
    a / d   — strafe left / right
    q / e   — rotate left / right
    space   — stop
    x       — sit down and quit
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

LINEAR_SPEED = 0.5  # m/s   — how fast Spot walks
ANGULAR_SPEED = 0.8  # rad/s — how fast Spot turns


# ─────────────────────────────────────────────────────────────────────────────
# TODO 1  —  Connect to the robot
# ─────────────────────────────────────────────────────────────────────────────
#
# Most SDKs follow this pattern:
#   step 1 — create a client/SDK object for your app
#   step 2 — point it at the device you want to talk to
#   step 3 — authenticate
#   step 4 — sync any state the SDK needs (here: clock sync so timestamps match)
#
# For Spot:
#   sdk   = bosdyn.client.create_standard_sdk("MyAppName")
#   robot = sdk.create_robot(ROBOT_IP)
#   robot.authenticate(USERNAME, PASSWORD)
#   robot.time_sync.wait_for_sync()
#
def connect_to_robot():
    """Return an authenticated, time-synced Robot object."""

    # TODO: follow the four steps above and return the robot
    robot = None
    return robot


# ─────────────────────────────────────────────────────────────────────────────
# TODO 2  —  Get a command client
# ─────────────────────────────────────────────────────────────────────────────
#
# SDKs often split functionality into smaller "service clients", one per task.
# Spot has separate clients for movement, the arm, cameras, and more.
#
# You ask the robot object to give you the one you need:
#   robot.ensure_client(SomeClient.default_service_name)
#
# For driving the base, use RobotCommandClient.
#
def get_command_client(robot):
    """Return a RobotCommandClient for sending movement commands."""

    # TODO: return robot.ensure_client(...) using RobotCommandClient
    return None


# ─────────────────────────────────────────────────────────────────────────────
# Pre-written examples — read these before doing TODO 3
# ─────────────────────────────────────────────────────────────────────────────
#
# Notice the pattern:
#   1. Use a Builder to construct a command object.
#   2. Pass it to cmd_client.robot_command(...) to send it.
#
# You will do the same thing in TODO 3.


def stand_up(robot, cmd_client):
    print("Powering on…")
    robot.power_on(timeout_sec=20)
    blocking_stand(cmd_client, timeout_sec=10)  # built-in helper: stand + wait
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
#
# Same two-step pattern as sit_down above:
#
#   step 1 — build:  RobotCommandBuilder.synchro_se2_velocity_command(vx, vy, vyaw)
#               vx      (m/s)    positive = forward,           negative = backward
#               vy      (m/s)    positive = left (strafe),     negative = right
#               vyaw    (rad/s)  positive = rotate left (CCW), negative = rotate right
#
#   step 2 — send:  cmd_client.robot_command(cmd, end_time_secs=time.time() + duration)
#               end_time_secs is a safety net: Spot stops automatically after
#               `duration` seconds even if this program crashes.
#
def send_velocity(cmd_client, vx, vy, vyaw, duration=0.6):
    """Send a velocity command lasting `duration` seconds."""

    # TODO: build the velocity command (step 1) and send it (step 2)
    pass


# ─────────────────────────────────────────────────────────────────────────────
# Teleop loop + main  —  no changes needed below this line
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

    # Lease: only one program may drive Spot at a time.
    # LeaseKeepAlive grabs the lease and releases it automatically when done.
    lease_client = robot.ensure_client(
        bosdyn.client.lease.LeaseClient.default_service_name
    )

    # E-stop: if this program crashes, Spot stops automatically.
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
