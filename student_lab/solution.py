"""
solution.py  —  Your First Robot SDK  (SOLUTION)
=================================================
Reference implementation of stencil.py — all three TODOs filled in, plus
single-key teleop (no Enter), body height, and dock / undock.
"""

import os
import select
import sys
import time

# ─────────────────────────────────────────────────────────────────────────────
# Simulation — run with --sim (or without bosdyn installed) to print commands
# ─────────────────────────────────────────────────────────────────────────────

_SIM = "--sim" in sys.argv
if not _SIM:
    import importlib.util
    if importlib.util.find_spec("bosdyn") is None:
        sys.exit(f"bosdyn not found in {sys.executable}.\n"
                 f"Run with the Python that has the Spot SDK (e.g. `conda activate drawSpot` then\n"
                 f"`python solution.py`), or pass --sim to run without a robot.")

if _SIM:
    import types

    class _Cmd:
        def __init__(self, label, vel=None, height=0.0):
            self._label, self._vel, self._height = label, vel, height

    class _CmdClient:
        default_service_name = "robot-command"
        def robot_command(self, cmd, end_time_secs=None):
            h = f", height {cmd._height:+.2f} m" if cmd._height else ""
            if cmd._vel is None:
                print(f"  [sim] Spot: {cmd._label}{h}")
                return
            vx, vy, vyaw = cmd._vel
            dirs = []
            if vx > 0:   dirs.append("forward")
            elif vx < 0: dirs.append("backward")
            if vy > 0:   dirs.append("left")
            elif vy < 0: dirs.append("right")
            if vyaw > 0:   dirs.append("rotate-left")
            elif vyaw < 0: dirs.append("rotate-right")
            dur = round(end_time_secs - time.time(), 1) if end_time_secs else "?"
            print(f"  [sim] Spot: {'+'.join(dirs) or 'stop'} ({dur}s{h})")

    class _LeaseClient:
        default_service_name = "lease"

    class _EstopClient:
        default_service_name = "estop"

    class _DockState:
        DOCK_STATUS_DOCKED = 1
        DOCK_STATUS_UNDOCKED = 3
        DockedStatus = types.SimpleNamespace(
            Name=lambda v: {1: "DOCK_STATUS_DOCKED", 3: "DOCK_STATUS_UNDOCKED"}[v])

    _dock = {"status": _DockState.DOCK_STATUS_DOCKED}

    class _DockingClient:
        default_service_name = "docking"
        def get_docking_state(self):
            return types.SimpleNamespace(status=_dock["status"], dock_id=520)

    class _KeepAlive:
        def __init__(self, *a, **kw): pass
        def __enter__(self): return self
        def __exit__(self, *a): pass

    class _EstopEP:
        def __init__(self, *a, **kw): pass
        def force_simple_setup(self): pass

    class _Robot:
        def __init__(self, ip): self._ip = ip
        def authenticate(self, user, pw): print(f"  [sim] authenticate({user}, ***)")
        class _TS:
            def wait_for_sync(self): pass
        time_sync = _TS()
        def power_on(self, **kw):  print("  [sim] power on")
        def power_off(self, **kw): print("  [sim] power off")
        def ensure_client(self, name):
            return {
                _CmdClient.default_service_name:     _CmdClient(),
                _LeaseClient.default_service_name:   _LeaseClient(),
                _EstopClient.default_service_name:   _EstopClient(),
                _DockingClient.default_service_name: _DockingClient(),
            }.get(name, object())

    class _RCB:
        @staticmethod
        def synchro_velocity_command(v_x, v_y, v_rot, body_height=0.0, **kw):
            return _Cmd("velocity", (v_x, v_y, v_rot), body_height)
        @staticmethod
        def synchro_stand_command(body_height=0.0, **kw):
            return _Cmd("stand", None, body_height)
        @staticmethod
        def synchro_sit_command(**kw):
            return _Cmd("sit")

    def _blocking_stand(cmd_client, **kw):
        print("  [sim] blocking_stand — Spot is standing")

    def _blocking_undock(robot, **kw):
        _dock["status"] = _DockState.DOCK_STATUS_UNDOCKED
        print("  [sim] blocking_undock — Spot backed off the dock")

    def _blocking_dock_robot(robot, dock_id, **kw):
        _dock["status"] = _DockState.DOCK_STATUS_DOCKED
        print(f"  [sim] blocking_dock_robot({dock_id}) — Spot is on the dock")

    def _mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items(): setattr(m, k, v)
        sys.modules[name] = m
        return m

    _bc = _mod("bosdyn.client",
               create_standard_sdk=lambda n: type("_SDK", (), {
                   "create_robot": staticmethod(lambda ip: _Robot(ip))
               })())
    _b  = _mod("bosdyn"); _b.client = _bc
    _u  = _mod("bosdyn.client.util", setup_logging=lambda **kw: None); _bc.util = _u
    _l  = _mod("bosdyn.client.lease", LeaseClient=_LeaseClient, LeaseKeepAlive=_KeepAlive)
    _bc.lease = _l
    _mod("bosdyn.client.estop",
         EstopClient=_EstopClient, EstopEndpoint=_EstopEP, EstopKeepAlive=_KeepAlive)
    _mod("bosdyn.client.robot_command",
         RobotCommandBuilder=_RCB, RobotCommandClient=_CmdClient, blocking_stand=_blocking_stand)
    _mod("bosdyn.client.docking", DockingClient=_DockingClient,
         blocking_undock=_blocking_undock, blocking_dock_robot=_blocking_dock_robot)
    _api = _mod("bosdyn.api"); _b.api = _api
    _api.docking = _mod("bosdyn.api.docking")
    _api.docking.docking_pb2 = _mod("bosdyn.api.docking.docking_pb2", DockState=_DockState)

    print("[sim mode] No robot needed — commands will be printed instead.\n")

# ─────────────────────────────────────────────────────────────────────────────
# Imports
# ─────────────────────────────────────────────────────────────────────────────

try:
    from dotenv import load_dotenv
    load_dotenv(os.path.join(os.path.dirname(__file__), "..", ".env"))
except ImportError:
    pass

import bosdyn.client
import bosdyn.client.util
import bosdyn.client.lease
from bosdyn.api.docking import docking_pb2
from bosdyn.client.docking import DockingClient, blocking_dock_robot, blocking_undock
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand

ROBOT_IP  = os.environ.get("SPOT_IP",       "0.0.0.0")
USERNAME  = os.environ.get("SPOT_USER",     "sim_user")
PASSWORD  = os.environ.get("SPOT_PASSWORD", "sim_pass")
DOCK_ID   = os.environ.get("SPOT_DOCK_ID")  # optional; otherwise the dock Spot started on

LINEAR_SPEED  = 0.5
ANGULAR_SPEED = 0.8

# Each key press drives Spot for this long. Holding a key auto-repeats faster
# than this, so Spot keeps moving and stops shortly after the key is released.
MOVE_DURATION = 0.6

HEIGHT_STEP = 0.05                 # m per r / f press
HEIGHT_MIN, HEIGHT_MAX = -0.15, 0.10


# ═════════════════════════════════════════════════════════════════════════════
#  SOLUTION
# ═════════════════════════════════════════════════════════════════════════════


# ─────────────────────────────────────────────────────────────────────────────
# TODO 1  —  Connect to the robot
# ─────────────────────────────────────────────────────────────────────────────

def connect_to_robot():
    sdk   = bosdyn.client.create_standard_sdk("SpotTeleop")  # step 1: create SDK
    robot = sdk.create_robot(ROBOT_IP)                        # step 2: point at device
    robot.authenticate(USERNAME, PASSWORD)                    # step 3: authenticate
    robot.time_sync.wait_for_sync()                           # step 4: sync clocks
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
    sit_cmd = RobotCommandBuilder.synchro_sit_command()
    cmd_client.robot_command(sit_cmd)
    time.sleep(2)
    robot.power_off(cut_immediately=False, timeout_sec=20)
    print("Powered off. Bye!")


def dock_state(robot):
    """(is_docked, dock_id) as Spot reports it."""
    state = robot.ensure_client(DockingClient.default_service_name).get_docking_state()
    print(f"Dock status: {docking_pb2.DockState.DockedStatus.Name(state.status)}")
    return state.status == docking_pb2.DockState.DOCK_STATUS_DOCKED, state.dock_id


def undock(robot):
    print("Undocking…")
    blocking_undock(robot)  # backs off the dock to its prep pose
    print("Undocked.")


def dock(robot, dock_id):
    print(f"Docking on dock {dock_id}… (the dock's fiducial must be in view)")
    blocking_dock_robot(robot, dock_id)  # walks on, sits down and starts charging
    print("Docked.")


# ─────────────────────────────────────────────────────────────────────────────
# TODO 3  —  Send a velocity command
# ─────────────────────────────────────────────────────────────────────────────

def send_velocity(cmd_client, vx, vy, vyaw, body_height=0.0, duration=MOVE_DURATION):
    cmd = RobotCommandBuilder.synchro_velocity_command(vx, vy, vyaw, body_height=body_height)  # step 1: build
    cmd_client.robot_command(cmd, end_time_secs=time.time() + duration)                         # step 2: send


# ─────────────────────────────────────────────────────────────────────────────
# Teleop loop + main
# ─────────────────────────────────────────────────────────────────────────────

CONTROLS = """
Controls (no Enter needed — hold a key to keep moving)
--------
  w / s   — forward / backward
  a / d   — strafe left / right
  q / e   — rotate left / right
  space   — stop
  r / f   — body up / down
  u       — undock
  k       — dock (sits and charges)
  x       — sit down and quit (stays put if docked)
"""

KEY_TO_VEL = {
    "w": ( LINEAR_SPEED,  0,             0),
    "s": (-LINEAR_SPEED,  0,             0),
    "a": ( 0,             LINEAR_SPEED,  0),
    "d": ( 0,            -LINEAR_SPEED,  0),
    "q": ( 0,             0,             ANGULAR_SPEED),
    "e": ( 0,             0,            -ANGULAR_SPEED),
    " ": ( 0,             0,             0),
}


class _Keys:
    """Reads single key presses without Enter (falls back to plain stdin when piped)."""

    def __enter__(self):
        self.tty = sys.stdin.isatty()
        if self.tty:
            import termios, tty
            self._old = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, *a):
        if self.tty:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old)

    def get(self, timeout=0.1):
        if not select.select([sys.stdin], [], [], timeout)[0]:
            return None
        key = sys.stdin.read(1)
        if key == "":                # stdin closed (piped input ran out)
            return "x"
        return key.lower()


def teleop_loop(robot, cmd_client, docked, dock_id):
    print(CONTROLS)
    height = 0.0
    with _Keys() as keys:
        while True:
            try:
                key = keys.get()
            except KeyboardInterrupt:  # Ctrl+C quits like x
                key = "x"
            if key is None or key == "\n":
                continue
            if key == "x":
                return docked
            if key in KEY_TO_VEL or key in ("r", "f"):
                if docked:
                    print("  On the dock — press u to undock first.")
                    continue
            if key in KEY_TO_VEL:
                send_velocity(cmd_client, *KEY_TO_VEL[key], body_height=height)
            elif key in ("r", "f"):
                step = HEIGHT_STEP if key == "r" else -HEIGHT_STEP
                height = min(HEIGHT_MAX, max(HEIGHT_MIN, height + step))
                cmd_client.robot_command(RobotCommandBuilder.synchro_stand_command(body_height=height))
                print(f"  Body height {height:+.2f} m")
            elif key == "u":
                if docked:
                    undock(robot)
                    docked = False
                else:
                    print("  Not on the dock.")
            elif key == "k":
                if docked:
                    print("  Already docked.")
                elif dock_id is None:
                    print("  No dock ID — set SPOT_DOCK_ID in .env.")
                else:
                    dock(robot, dock_id)
                    docked, height = True, 0.0
            else:
                print(f"  Unknown key {key!r}. See controls above.")


def main():
    bosdyn.client.util.setup_logging(verbose=False)

    robot      = connect_to_robot()
    cmd_client = get_command_client(robot)

    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)

    estop_client = robot.ensure_client(EstopClient.default_service_name)
    estop_ep = EstopEndpoint(estop_client, name="teleop_solution", estop_timeout=9.0)
    estop_ep.force_simple_setup()

    docked, start_dock = dock_state(robot)
    dock_id = int(DOCK_ID) if DOCK_ID else (start_dock if docked else None)

    with (
        EstopKeepAlive(estop_ep),
        bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True),
    ):
        stand_up(robot, cmd_client)
        if docked:
            print("Spot is on its dock — press u to undock.")
        try:
            docked = teleop_loop(robot, cmd_client, docked, dock_id)
        finally:
            if docked:
                # sitting would drop Spot onto the dock; docking already left it seated
                robot.power_off(cut_immediately=False, timeout_sec=20)
                print("Powered off on the dock. Bye!")
            else:
                sit_down(robot, cmd_client)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:  # Ctrl+C outside the teleop loop; cleanup already ran
        print("\nInterrupted.")
