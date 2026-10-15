"""
stencil.py  —  Your First Robot SDK
=====================================
An SDK (Software Development Kit) is a library that lets you talk to a
piece of hardware without knowing its low-level details.  You just call
functions like  robot.authenticate()  or  cmd_client.robot_command(...)
and the SDK handles the network messages, protocols, and timing for you.

In this exercise you will use Boston Dynamics' Spot SDK to:
  1. Connect to the robot.
  2. Claim exclusive control so no other program drives it at the same time.
  3. Send velocity commands from your keyboard.

Parts 4 onward (stand/sit, the input loop, and main) are already written.
Read through them to see how the whole program fits together, then fill in
the three TODO sections.

Running
-------
    python stencil.py --sim    # no robot needed — great for testing!
    python stencil.py          # real Spot (needs .env with SPOT_IP etc.)

Controls (once running)
-----------------------
    w / s   — forward / backward
    a / d   — strafe left / right
    q / e   — rotate left / right
    space   — stop
    x       — sit down and quit
"""

import os
import sys
import time

# ─────────────────────────────────────────────────────────────────────────────
# Simulation — skip to TODO 1 below
# ─────────────────────────────────────────────────────────────────────────────
# This section makes the file runnable without a real robot.
# It replaces the real SDK with a lightweight fake that just prints what Spot
# *would* do.  You do NOT need to read or understand this part.

_SIM = "--sim" in sys.argv
if not _SIM:
    try:
        import importlib

        _SIM = importlib.util.find_spec("bosdyn.client") is None
    except Exception:
        _SIM = True

if _SIM:
    import types

    class _Vel:
        def __init__(self, vx, vy, vyaw):
            self.vx, self.vy, self.vyaw = vx, vy, vyaw

    class _Cmd:
        pass

    class _CmdClient:
        default_service_name = "robot-command"

        def robot_command(self, cmd, end_time_secs=None):
            v = getattr(cmd, "_vel", None)
            if v is None:
                print(f"  [sim] Spot: {getattr(cmd, '_label', 'command')}")
                return
            dur = round(end_time_secs - time.time(), 1) if end_time_secs else "?"
            dirs = []
            if v.vx > 0:
                dirs.append("forward")
            elif v.vx < 0:
                dirs.append("backward")
            if v.vy > 0:
                dirs.append("left")
            elif v.vy < 0:
                dirs.append("right")
            if v.vyaw > 0:
                dirs.append("rotate-left")
            elif v.vyaw < 0:
                dirs.append("rotate-right")
            print(f"  [sim] Spot: {'+'.join(dirs) or 'stop'} ({dur}s)")

    class _LeaseClient:
        default_service_name = "lease"

    class _EstopClient:
        default_service_name = "estop"

    class _KeepAlive:
        def __init__(self, *a, **kw):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *a):
            pass

    class _EstopEP:
        def __init__(self, *a, **kw):
            pass

        def force_simple_setup(self):
            pass

    class _Robot:
        def __init__(self, ip):
            self._ip = ip

        def authenticate(self, user, pw):
            print(f"  [sim] authenticate({user}, ***)")

        class _TS:
            def wait_for_sync(self):
                pass

        time_sync = _TS()

        def power_on(self, **kw):
            print("  [sim] power on")

        def power_off(self, **kw):
            print("  [sim] power off")

        def ensure_client(self, name):
            return {
                _CmdClient.default_service_name: _CmdClient(),
                _LeaseClient.default_service_name: _LeaseClient(),
                _EstopClient.default_service_name: _EstopClient(),
            }.get(name, object())

    class _RCB:
        @staticmethod
        def synchro_se2_velocity_command(vx, vy, vyaw):
            c = _Cmd()
            c._vel = _Vel(vx, vy, vyaw)
            return c

        @staticmethod
        def synchro_sit_command():
            c = _Cmd()
            c._label = "sit"
            return c

    def _blocking_stand(cmd_client, **kw):
        print("  [sim] blocking_stand — Spot is standing")

    def _mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    _bc = _mod(
        "bosdyn.client",
        create_standard_sdk=lambda n: type(
            "_SDK", (), {"create_robot": staticmethod(lambda ip: _Robot(ip))}
        )(),
    )
    _b = _mod("bosdyn")
    _b.client = _bc
    _u = _mod("bosdyn.client.util", setup_logging=lambda **kw: None)
    _bc.util = _u
    _l = _mod(
        "bosdyn.client.lease", LeaseClient=_LeaseClient, LeaseKeepAlive=_KeepAlive
    )
    _bc.lease = _l
    _mod(
        "bosdyn.client.estop",
        EstopClient=_EstopClient,
        EstopEndpoint=_EstopEP,
        EstopKeepAlive=_KeepAlive,
    )
    _mod(
        "bosdyn.client.robot_command",
        RobotCommandBuilder=_RCB,
        RobotCommandClient=_CmdClient,
        blocking_stand=_blocking_stand,
    )

    print("[sim mode] No robot needed — commands will be printed instead.\n")

# ─────────────────────────────────────────────────────────────────────────────
# Imports (the real SDK — or the fakes above in sim mode)
# ─────────────────────────────────────────────────────────────────────────────

try:
    from dotenv import load_dotenv

    load_dotenv(os.path.join(os.path.dirname(__file__), "..", ".env"))
except ImportError:
    pass  # dotenv not installed; rely on environment variables being set manually

import bosdyn.client
import bosdyn.client.util
import bosdyn.client.lease
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.robot_command import (
    RobotCommandBuilder,
    RobotCommandClient,
    blocking_stand,
)

ROBOT_IP = os.environ.get("SPOT_IP", "0.0.0.0")
USERNAME = os.environ.get("SPOT_USER", "sim_user")
PASSWORD = os.environ.get("SPOT_PASSWORD", "sim_pass")

LINEAR_SPEED = 0.5  # m/s   — how fast Spot walks
ANGULAR_SPEED = 0.8  # rad/s — how fast Spot turns


# ═════════════════════════════════════════════════════════════════════════════
#  YOUR CODE — fill in the three TODOs below
# ═════════════════════════════════════════════════════════════════════════════


# ─────────────────────────────────────────────────────────────────────────────
# TODO 1  —  Connect to the robot
# ─────────────────────────────────────────────────────────────────────────────
#
# Most SDKs follow this pattern:
#   step 1 — create a "client" or "SDK" object for your app
#   step 2 — point it at the device you want to talk to
#   step 3 — authenticate
#   step 4 — sync any state the SDK needs (here: clock sync so timestamps match)
#
# For Spot it looks like this:
#
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
# Spot has separate clients for: movement, the arm, the camera, the lease, etc.
#
# You ask the robot object to give you the client you need:
#
#   robot.ensure_client(SomeClient.default_service_name)
#
# For driving the base, the client you want is RobotCommandClient.
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
#   1. Use a "Builder" helper to construct a command object.
#   2. Pass that object to cmd_client.robot_command(...) to send it.
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
# Same two-step pattern as sit_down above, but for velocity:
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


# ═════════════════════════════════════════════════════════════════════════════
#  The rest is already written — teleop loop and main
# ═════════════════════════════════════════════════════════════════════════════

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
    # LeaseKeepAlive grabs the lease and releases it when the block exits.
    lease_client = robot.ensure_client(
        bosdyn.client.lease.LeaseClient.default_service_name
    )

    # E-stop: if this program crashes, Spot stops automatically.
    estop_client = robot.ensure_client(EstopClient.default_service_name)
    estop_ep = EstopEndpoint(estop_client, name="teleop_stencil", estop_timeout=9.0)
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
