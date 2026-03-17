"""
gcode_manual/calibrate.py
=========================
Calibrate how hard the marker presses onto the paper.

AUTO mode (default)
    Lifts the arm above the paper, descends in 0.5 mm steps, reads wrist
    force at each step, and saves the height where contact is first detected
    plus a small backoff.  No input needed — just confirm position and wait.

MANUAL mode  (--manual)
    After touching down, nudge the marker up/down with +/- until the
    contact pressure looks right, then press S to save.

Usage
-----
    python gcode_manual/calibrate.py           # auto
    python gcode_manual/calibrate.py --manual  # manual nudge

Controls (manual mode)
----------------------
    +  or  ↑   move marker UP   1 mm  (less pressing)
    -  or  ↓   move marker DOWN 1 mm  (more pressing)
    s          save offset to gcode.cfg and exit
    q          quit without saving
"""

import configparser
import os
import select
import sys
import termios
import time
import tty

import numpy as np
from dotenv import load_dotenv
from google.protobuf import duration_pb2, wrappers_pb2

import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.api import arm_surface_contact_pb2, arm_surface_contact_service_pb2, geometry_pb2
from bosdyn.client.arm_surface_contact import ArmSurfaceContactClient
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME,
                                         GROUND_PLANE_FRAME_NAME, HAND_FRAME_NAME,
                                         ODOM_FRAME_NAME, VISION_FRAME_NAME, WR1_FRAME_NAME,
                                         get_a_tform_b)
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_until_arm_arrives, blocking_stand)
from bosdyn.client.robot_state import RobotStateClient

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_HERE)
_CFG_PATH = os.path.join(_HERE, 'gcode.cfg')

load_dotenv(os.path.join(_REPO_ROOT, '.env'))


def _require_env(name):
    val = os.environ.get(name)
    if not val:
        raise SystemExit(f'Missing required env var: {name}  (set it in .env)')
    return val


def _get_key(timeout=0.05):
    """Read one keypress (non-blocking). Returns None if no key within timeout."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if not r:
            return None
        ch = sys.stdin.read(1)
        if ch == '\x1b':          # possible arrow key escape sequence
            r2, _, _ = select.select([sys.stdin], [], [], 0.05)
            if r2:
                ch += sys.stdin.read(2)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _send_arm_to_z(cmd_client, anchor, target_z_odom):
    """
    Move arm to a fixed x/y/rotation with only z varying.
    anchor = (x, y, qw, qx, qy, qz) captured once after touchdown.
    Uses position mode so z is exact.
    """
    x, y, qw, qx, qy, qz = anchor
    arm_cmd = RobotCommandBuilder.arm_pose_command(
        x, y, target_z_odom,
        qw, qx, qy, qz,
        ODOM_FRAME_NAME, 0.2,   # short move time → snappy real-time response
    )
    cmd_client.robot_command(RobotCommandBuilder.build_synchro_command(arm_cmd))


def _touch_to_find_ground(robot, state_client, asc_client, cmd_client,
                          tool_length, press_force_pct, velocity):
    """
    Press marker down in force mode to find the paper surface.
    Returns (ground_z_in_odom, vision_T_odom) after touchdown.
    """
    from bosdyn.api import trajectory_pb2

    robot_state = state_client.get_robot_state()
    snap = robot_state.kinematic_state.transforms_snapshot

    vision_T_body = get_a_tform_b(snap, VISION_FRAME_NAME, BODY_FRAME_NAME)
    odom_T_body   = get_a_tform_b(snap, ODOM_FRAME_NAME,   BODY_FRAME_NAME)
    vision_T_odom = vision_T_body * odom_T_body.inverse()

    vision_T_wr1  = get_a_tform_b(snap, VISION_FRAME_NAME, WR1_FRAME_NAME)
    wr1_T_tool    = SE3Pose(0.23589 + tool_length, 0, 0, Quat(w=1, x=0, y=0, z=0))
    vision_T_tool = vision_T_wr1 * wr1_T_tool

    vision_T_admittance = geometry_pb2.SE3Pose(
        position=geometry_pb2.Vec3(x=0, y=0, z=0),
        rotation=geometry_pb2.Quaternion(w=1, x=0, y=0, z=0))

    traj = trajectory_pb2.SE3Trajectory(points=[
        trajectory_pb2.SE3TrajectoryPoint(
            pose=geometry_pb2.SE3Pose(
                position=geometry_pb2.Vec3(x=vision_T_tool.x,
                                           y=vision_T_tool.y,
                                           z=vision_T_tool.z),
                rotation=geometry_pb2.Quaternion(w=vision_T_tool.rot.w,
                                                 x=vision_T_tool.rot.x,
                                                 y=vision_T_tool.rot.y,
                                                 z=vision_T_tool.rot.z),
            ),
            time_since_reference=duration_pb2.Duration(seconds=0, nanos=0),
        )
    ])

    cmd = arm_surface_contact_pb2.ArmSurfaceContact.Request(
        pose_trajectory_in_task=traj,
        root_frame_name=VISION_FRAME_NAME,
        root_tform_task=vision_T_admittance,
        press_force_percentage=geometry_pb2.Vec3(x=0, y=0, z=press_force_pct),
        x_axis=arm_surface_contact_pb2.ArmSurfaceContact.Request.AXIS_MODE_POSITION,
        y_axis=arm_surface_contact_pb2.ArmSurfaceContact.Request.AXIS_MODE_POSITION,
        z_axis=arm_surface_contact_pb2.ArmSurfaceContact.Request.AXIS_MODE_FORCE,
        max_linear_velocity=wrappers_pb2.DoubleValue(value=velocity),
    )
    cmd.press_force_percentage.z = press_force_pct
    cmd.xy_admittance  = arm_surface_contact_pb2.ArmSurfaceContact.Request.ADMITTANCE_SETTING_OFF
    cmd.z_admittance   = arm_surface_contact_pb2.ArmSurfaceContact.Request.ADMITTANCE_SETTING_LOOSE
    cmd.xy_to_z_cross_term_admittance = arm_surface_contact_pb2.ArmSurfaceContact.Request.ADMITTANCE_SETTING_OFF
    cmd.bias_force_ewrt_body.CopyFrom(geometry_pb2.Vec3(x=0, y=0, z=0))

    gripper = RobotCommandBuilder.claw_gripper_open_fraction_command(0)
    cmd.gripper_command.CopyFrom(
        gripper.synchronized_command.gripper_command.claw_gripper_command)
    cmd.is_robot_following_hand = False
    asc_client.arm_surface_contact_command(
        arm_surface_contact_service_pb2.ArmSurfaceContactCommand(request=cmd))

    time.sleep(4.0)   # wait for touchdown and force settle

    # Re-read state after touch
    robot_state = state_client.get_robot_state()
    snap = robot_state.kinematic_state.transforms_snapshot

    odom_T_ground = get_a_tform_b(snap, ODOM_FRAME_NAME, GROUND_PLANE_FRAME_NAME)
    odom_T_hand   = get_a_tform_b(snap, ODOM_FRAME_NAME, HAND_FRAME_NAME)

    return odom_T_ground.z, odom_T_hand.z


def _read_force_magnitude(state_client):
    """Return the estimated end-effector force magnitude (N), or None if unavailable."""
    try:
        f = state_client.get_robot_state().manipulator_state.estimated_end_effector_force_in_hand
        return (f.x**2 + f.y**2 + f.z**2) ** 0.5
    except Exception:
        return None


def _auto_calibrate(cmd_client, state_client, anchor, ground_z,
                    backoff_m=0.001, force_threshold_n=1.5):
    """
    Lift arm to +20 mm, descend in 0.5 mm steps, detect contact by wrist force spike.
    Returns the calibrated draw_z_offset in metres.

    backoff_m        how far above the contact point to save (default 1 mm)
    force_threshold_n force increase (N) that counts as contact (default 1.5 N)
    """
    LIFT_MM   = 20
    STEP_MM   = 0.5
    SETTLE_S  = 0.25   # wait after each step before reading force

    # Lift to safe height
    print(f'  Lifting to +{LIFT_MM} mm above surface…')
    _send_arm_to_z(cmd_client, anchor, ground_z + LIFT_MM / 1000)
    time.sleep(1.0)

    # Baseline force at lifted position
    baseline = _read_force_magnitude(state_client)
    if baseline is None:
        print('  [WARN] Force data unavailable — falling back to draw_z_offset = 0.0')
        return 0.0
    print(f'  Baseline force: {baseline:.2f} N')

    # Descend step by step
    current_offset_mm = LIFT_MM
    contact_offset_mm = None

    print(f'  Descending in {STEP_MM} mm steps…')
    while current_offset_mm > -5:   # don't go more than 5 mm below surface
        current_offset_mm -= STEP_MM
        _send_arm_to_z(cmd_client, anchor, ground_z + current_offset_mm / 1000)
        time.sleep(SETTLE_S)

        force = _read_force_magnitude(state_client)
        if force is None:
            continue
        delta = force - baseline
        print(f'    z={current_offset_mm:+.1f} mm   force={force:.2f} N  (Δ{delta:+.2f})',
              flush=True)

        if delta > force_threshold_n:
            contact_offset_mm = current_offset_mm
            print(f'  Contact detected at {contact_offset_mm:+.1f} mm')
            break

    if contact_offset_mm is None:
        print('  [WARN] No contact detected — is the paper under the marker?')
        print('         Saving offset = 0.0 mm (surface level)')
        return 0.0

    # Back off by backoff_m above contact
    result = (contact_offset_mm / 1000) + backoff_m
    print(f'  Contact at {contact_offset_mm:+.1f} mm  +  {backoff_m*1000:.0f} mm backoff'
          f'  →  draw_z_offset = {result*1000:+.1f} mm')

    # Move arm to the calibrated position so user can see it
    _send_arm_to_z(cmd_client, anchor, ground_z + result)
    time.sleep(0.5)
    return result


def _save_offset(offset_m):
    cfg = configparser.ConfigParser()
    cfg.read(_CFG_PATH)
    cfg.set('General', 'draw_z_offset', f'{offset_m:.4f}')
    with open(_CFG_PATH, 'w') as f:
        cfg.write(f)
    print(f'\n  Saved draw_z_offset = {offset_m*1000:.1f} mm  →  {_CFG_PATH}')


def run(manual=False):
    cfg = configparser.ConfigParser()
    cfg.read(_CFG_PATH)
    g = cfg['General']
    tool_length     = g.getfloat('tool_length', 0.0)
    velocity        = g.getfloat('velocity', 0.15)
    press_force_pct = g.getfloat('press_force_percent', -0.005)
    current_offset  = g.getfloat('draw_z_offset', 0.005)

    robot_ip = _require_env('SPOT_IP')
    username = _require_env('SPOT_USER')
    password = _require_env('SPOT_PASSWORD')

    bosdyn.client.util.setup_logging(verbose=False)
    sdk   = bosdyn.client.create_standard_sdk('GcodeCalibrate')
    robot = sdk.create_robot(robot_ip)
    robot.authenticate(username, password)
    robot.time_sync.wait_for_sync()
    assert robot.has_arm(), 'Robot requires an arm.'

    asc_client   = robot.ensure_client(ArmSurfaceContactClient.default_service_name)
    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    cmd_client   = robot.ensure_client(RobotCommandClient.default_service_name)
    state_client = robot.ensure_client(RobotStateClient.default_service_name)
    estop_client = robot.ensure_client(EstopClient.default_service_name)

    estop_ep = EstopEndpoint(estop_client, name='gcode_calibrate_estop', estop_timeout=9.0)
    estop_ep.force_simple_setup()

    with (
        EstopKeepAlive(estop_ep),
        bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True),
    ):
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), 'Power on failed.'
        blocking_stand(cmd_client, timeout_sec=10)
        robot.logger.info('Robot standing.')

        # ── Move arm to start position ────────────────────────────────────
        robot_state = state_client.get_robot_state()
        snap = robot_state.kinematic_state.transforms_snapshot
        odom_T_flat_body = get_a_tform_b(snap, ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
        flat_body_T_hand = SE3Pose(0.75, 0, -0.35, Quat(w=0.707, x=0, y=0.707, z=0))
        odom_T_hand = odom_T_flat_body * flat_body_T_hand
        p = odom_T_hand.to_proto()

        arm_cmd = RobotCommandBuilder.arm_pose_command(
            p.position.x, p.position.y, p.position.z,
            p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z,
            ODOM_FRAME_NAME, 0.000001,
        )
        cmd_id = cmd_client.robot_command(RobotCommandBuilder.build_synchro_command(arm_cmd))
        block_until_arm_arrives(cmd_client, cmd_id)

        # ── Close gripper ─────────────────────────────────────────────────
        gripper_cmd = RobotCommandBuilder.claw_gripper_close_command()
        gripper_cmd.synchronized_command.gripper_command.claw_gripper_command.maximum_torque.value = 15.0
        cmd_client.robot_command(gripper_cmd)
        time.sleep(1.0)

        # ── Confirm ───────────────────────────────────────────────────────
        print()
        print('=' * 60)
        print(' Arm is in position.')
        print(' Place the paper under the marker tip.')
        print(' Press Enter to touch down and begin calibration,')
        print(' or Ctrl-C to abort.')
        print('=' * 60)
        input()

        # ── Touch to find ground ──────────────────────────────────────────
        print('  Pressing marker down to find surface…')
        ground_z, touch_z = _touch_to_find_ground(
            robot, state_client, asc_client, cmd_client,
            tool_length, press_force_pct, velocity)

        print(f'  Surface found at odom z = {ground_z*1000:.1f} mm')
        print(f'  Marker touched at       = {touch_z*1000:.1f} mm')

        # Capture arm x/y/rotation once after touchdown — these stay fixed during nudging
        snap = state_client.get_robot_state().kinematic_state.transforms_snapshot
        odom_T_hand = get_a_tform_b(snap, ODOM_FRAME_NAME, HAND_FRAME_NAME)
        hp = odom_T_hand.to_proto()
        anchor = (hp.position.x, hp.position.y,
                  hp.rotation.w, hp.rotation.x, hp.rotation.y, hp.rotation.z)

        if not manual:
            # ── Auto calibration ──────────────────────────────────────────
            print()
            offset = _auto_calibrate(cmd_client, state_client, anchor, ground_z)
            print()
            choice = input(f'  Save draw_z_offset = {offset*1000:+.1f} mm? [Y/n] ').strip().lower()
            if choice not in ('n', 'no'):
                _save_offset(offset)
            else:
                print('  Not saved.')
        else:
            # ── Manual nudge loop ─────────────────────────────────────────
            offset = current_offset
            _send_arm_to_z(cmd_client, anchor, ground_z + offset)
            time.sleep(0.3)

            print()
            print('┌──────────────────────────────────────────────────┐')
            print('│  Manual calibration                              │')
            print('│                                                  │')
            print('│   +  or  ↑   move UP   1 mm  (less pressing)    │')
            print('│   -  or  ↓   move DOWN 1 mm  (more pressing)    │')
            print('│   s          save and exit                       │')
            print('│   q          quit without saving                 │')
            print('└──────────────────────────────────────────────────┘')
            print()
            print(f'  offset: {offset*1000:+.1f} mm  '
                  f'(0 = just touching, + = above paper, - = pressing in)')
            print()

            saved = False
            while True:
                key = _get_key(timeout=0.1)
                if key is None:
                    continue
                elif key in ('+', '=', '\x1b[A'):
                    offset += 0.001
                    _send_arm_to_z(cmd_client, anchor, ground_z + offset)
                    print(f'  ↑  {offset*1000:+.1f} mm', flush=True)
                elif key in ('-', '\x1b[B'):
                    offset -= 0.001
                    _send_arm_to_z(cmd_client, anchor, ground_z + offset)
                    print(f'  ↓  {offset*1000:+.1f} mm', flush=True)
                elif key in ('s', 'S'):
                    _save_offset(offset)
                    saved = True
                    break
                elif key in ('q', 'Q', '\x03', '\x04'):
                    print('\n  Exiting without saving.')
                    break

            if not saved:
                print(f'  (draw_z_offset unchanged: {current_offset*1000:.1f} mm)')

        robot.logger.info('Stowing arm and sitting…')
        cmd_client.robot_command(RobotCommandBuilder.arm_stow_command())
        time.sleep(2)
        cmd_client.robot_command(RobotCommandBuilder.synchro_sit_command())
        time.sleep(2)
        robot.power_off(cut_immediately=False, timeout_sec=20)


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Calibrate marker contact height')
    parser.add_argument('--manual', action='store_true',
                        help='Manual nudge mode instead of auto-detect')
    args = parser.parse_args()
    try:
        run(manual=args.manual)
    except KeyboardInterrupt:
        print('\nAborted.')
    except Exception:
        bosdyn.client.util.get_logger().exception('Fatal error')
        sys.exit(1)


if __name__ == '__main__':
    main()
