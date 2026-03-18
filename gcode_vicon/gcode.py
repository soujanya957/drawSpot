"""
gcode_vicon/gcode.py
====================
Gcode drawing for Spot — marker pre-taped to gripper, canvas bounds from Vicon.
Credentials are read from the .env file in the repo root.

Usage
-----
    python gcode_vicon/gcode.py
    python gcode_vicon/gcode.py triangle.ngc
    python gcode_vicon/gcode.py triangle.ngc --vicon 169.254.217.218:801
    python gcode_vicon/gcode.py triangle.ngc --test-file-parsing

Drawing parameters (speed, force, etc.) are in gcode.cfg.

How it works
------------
1. Connects to Vicon and reads the 4-corner canvas geometry.
2. Auto-scales the gcode to fill *margin* fraction of the canvas
   (aspect-ratio preserved) and centers the drawing within it.
3. Spot stands up and closes the gripper on the pre-taped marker.
4. Arm moves above the canvas centre.
5. Press Enter — drawing begins; gcode is bounded within the Vicon canvas.

Key difference from gcode_manual
---------------------------------
gcode_manual uses touch-to-find-ground to locate the surface and sets gcode
(0,0) wherever the marker first touches.  gcode_vicon skips that step: the
Vicon canvas geometry provides the surface height, origin, and exact bounds so
the drawing can never go outside the physical canvas.

Positioning
-----------
Place the physical canvas within Vicon's tracking volume with all 4 corner
markers visible.  Stand Spot so its arm can reach the canvas
(front feet ~50-70 cm from the near edge, facing the canvas).
"""

import argparse
import configparser
import os
import sys
import time

import numpy as np
from dotenv import load_dotenv
from google.protobuf import duration_pb2, wrappers_pb2

import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.api import (arm_surface_contact_pb2, arm_surface_contact_service_pb2,
                        geometry_pb2, trajectory_pb2)
from bosdyn.client.arm_surface_contact import ArmSurfaceContactClient
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME,
                                         HAND_FRAME_NAME, ODOM_FRAME_NAME, VISION_FRAME_NAME,
                                         WR1_FRAME_NAME, get_a_tform_b, math_helpers)
from bosdyn.client.math_helpers import Quat, SE3Pose, math
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_until_arm_arrives, blocking_stand)
from bosdyn.client.robot_state import RobotStateClient

_HERE      = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_HERE)

sys.path.insert(0, _REPO_ROOT)
from vicon.client import ViconClient  # noqa: E402

load_dotenv(os.path.join(_REPO_ROOT, '.env'))


def _require_env(name):
    val = os.environ.get(name)
    if not val:
        raise SystemExit(f'Missing required env var: {name}  (set it in .env)')
    return val


# ---------------------------------------------------------------------------
# Gcode bounds scan
# ---------------------------------------------------------------------------

def scan_gcode_bounds(file_path):
    """Return (x_min, x_max, y_min, y_max) in raw gcode units (scale=1)."""
    x_vals, y_vals = [], []
    cur_x = cur_y = 0.0
    with open(file_path) as f:
        for line in f:
            for ch in ('(', '%', ';'):
                idx = line.find(ch)
                if idx >= 0:
                    line = line[:idx]
            parts = line.split()
            if not parts:
                continue
            cmd = parts[0]
            if cmd not in ('G00', 'G0', 'G01', 'G1', 'G02', 'G2', 'G03', 'G3'):
                continue
            x, y, i_val, j_val = cur_x, cur_y, 0.0, 0.0
            for p in parts[1:]:
                if not p:
                    continue
                if   p[0] == 'X': x     = float(p[1:])
                elif p[0] == 'Y': y     = float(p[1:])
                elif p[0] == 'I': i_val = float(p[1:])
                elif p[0] == 'J': j_val = float(p[1:])
            x_vals.append(x)
            y_vals.append(y)
            if cmd in ('G02', 'G2', 'G03', 'G3') and (i_val or j_val):
                x_vals.append(cur_x + i_val)
                y_vals.append(cur_y + j_val)
            cur_x, cur_y = x, y
    if not x_vals or not y_vals:
        return 0.0, 1.0, 0.0, 1.0
    return min(x_vals), max(x_vals), min(y_vals), max(y_vals)


# ---------------------------------------------------------------------------
# Gcode helpers  (identical to gcode_manual)
# ---------------------------------------------------------------------------

def make_orthogonal(primary, secondary):
    p = primary / np.linalg.norm(primary)
    s = secondary / np.linalg.norm(secondary)
    u = s - (np.dot(p, s) / np.dot(s, s)) * p
    return u / np.linalg.norm(u)


def pick_gcode_file():
    """Show numbered menu of g_codes/ files and return the chosen path."""
    g_codes_dir = os.path.join(_HERE, 'g_codes')
    exts = ('.gcode', '.ngc', '.nc', '.tap')
    files = sorted(
        os.path.join(g_codes_dir, f)
        for f in os.listdir(g_codes_dir)
        if f.lower().endswith(exts)
    ) if os.path.isdir(g_codes_dir) else []
    if not files:
        raise SystemExit('No gcode files found in g_codes/')
    print()
    print('Available gcode files:')
    for i, f in enumerate(files, 1):
        print(f'  {i}. {os.path.basename(f)}')
    while True:
        raw = input('Pick a file [1]: ').strip()
        if not raw:
            return files[0]
        try:
            idx = int(raw) - 1
            if 0 <= idx < len(files):
                return files[idx]
        except ValueError:
            pass
        print(f'  Enter a number between 1 and {len(files)}.')


class GcodeReader:

    def __init__(self, file_path, tool_length, scale, logger,
                 below_z_is_admittance, travel_z, draw_on_wall,
                 gcode_start_x=0, gcode_start_y=0, draw_z_offset=0.0):
        self.file = open(file_path, 'r')
        self.tool_length = tool_length
        self.scale = scale
        self.logger = logger
        self.below_z_is_admittance = below_z_is_admittance
        self.travel_z = travel_z
        self.draw_on_wall = draw_on_wall
        self.gcode_start_x = gcode_start_x
        self.gcode_start_y = gcode_start_y
        self.draw_z_offset = draw_z_offset
        self.current_origin_T_goals = None
        self.last_x = self.last_y = self.last_z = 0
        self._last_line = ''

    def set_origin(self, vision_T_origin, vision_T_admittance_frame):
        if not self.draw_on_wall:
            zhat = [0.0, 0.0, 1.0]
            x1, x2, x3 = vision_T_origin.rot.transform_point(-1.0, 0.0, 0.0)
            xhat = make_orthogonal(zhat, [x1, x2, x3])
            yhat = np.cross(zhat, xhat)
            mat = np.array([xhat, yhat, zhat]).T
            self.vision_T_origin = SE3Pose(vision_T_origin.x, vision_T_origin.y,
                                           vision_T_origin.z, Quat.from_matrix(mat))
        else:
            x1, x2, x3 = vision_T_admittance_frame.rot.transform_point(0, -1, 0)
            y1, y2, y3 = vision_T_admittance_frame.rot.transform_point(1, 0, 0)
            z1, z2, z3 = vision_T_admittance_frame.rot.transform_point(0, 0, 1)
            mat = np.array([[x1, y1, z1], [x2, y2, z2], [x3, y3, z3]]).T
            self.vision_T_origin = SE3Pose(vision_T_origin.x, vision_T_origin.y,
                                           vision_T_origin.z, Quat.from_matrix(mat))

    def _goal_quat(self):
        if not self.draw_on_wall:
            xhat = np.array([0, 0, -1], dtype=float)
            zhat = np.array([-1, 0, 0], dtype=float)
            yhat = np.cross(zhat, xhat)
            mat = np.array([xhat, yhat, zhat]).T
        else:
            mat = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float).T
        return Quat.from_matrix(mat)

    def convert_gcode_to_origin_T_goals(self, line):
        for ch in ('(', '%', ';'):
            idx = line.find(ch)
            if idx >= 0:
                line = line[:idx]
        parts = line.split()
        if not parts:
            return None

        if parts[0] in ('G00', 'G0', 'G01', 'G1'):
            x, y, z = self.last_x, self.last_y, self.last_z
            for p in parts[1:]:
                if   p[0] == 'X': x = (float(p[1:]) - self.gcode_start_x) * self.scale
                elif p[0] == 'Y': y = (float(p[1:]) - self.gcode_start_y) * self.scale
                elif p[0] == 'Z': z = float(p[1:]) * self.scale
            self.last_x, self.last_y, self.last_z = x, y, z
            return [SE3Pose(x, y, z, self._goal_quat())]

        elif parts[0] in ('G02', 'G2', 'G03', 'G3'):
            x, y, z = self.last_x, self.last_y, self.last_z
            i_val = j_val = k_val = 0.0
            for p in parts[1:]:
                if   p[0] == 'X': x     = (float(p[1:]) - self.gcode_start_x) * self.scale
                elif p[0] == 'Y': y     = (float(p[1:]) - self.gcode_start_y) * self.scale
                elif p[0] == 'Z': z     = float(p[1:]) * self.scale
                elif p[0] == 'I': i_val = float(p[1:]) * self.scale
                elif p[0] == 'J': j_val = float(p[1:]) * self.scale
                elif p[0] == 'K': k_val = float(p[1:]) * self.scale
            clockwise = parts[0] in ('G02', 'G2')
            if i_val != 0 or j_val != 0:
                lp = [self.last_x, self.last_y]; ep = [x, y]
                cp = [self.last_x + i_val, self.last_y + j_val]
                def make_pt(a, b): return SE3Pose(a, b, self.last_z, self._goal_quat())
            elif i_val != 0 or k_val != 0:
                lp = [self.last_x, self.last_z]; ep = [x, z]
                cp = [self.last_x + i_val, self.last_z + k_val]
                def make_pt(a, b): return SE3Pose(a, self.last_y, b, self._goal_quat())
            else:
                lp = [self.last_y, self.last_z]; ep = [y, z]
                cp = [self.last_y + j_val, self.last_z + k_val]
                def make_pt(a, b): return SE3Pose(self.last_x, a, b, self._goal_quat())
            lr = np.subtract(lp, cp); er = np.subtract(ep, cp)
            lt = math.atan2(lr[1], lr[0]); et = math.atan2(er[1], er[0])
            if clockwise:
                if lt < et: lt += 2 * math.pi
            else:
                if lt > et: et += 2 * math.pi
            n = max(abs(int((et - lt) / 0.01)), 1)
            r = math.sqrt(lr[0]**2 + lr[1]**2)
            goals = []
            for i in range(n - 1):
                th = lt - i * 0.01 if clockwise else lt + i * 0.01
                goals.append(make_pt(cp[0] + r * math.cos(th), cp[1] + r * math.sin(th)))
            goals.append(make_pt(ep[0], ep[1]))
            self.last_x, self.last_y, self.last_z = goals[-1].x, goals[-1].y, goals[-1].z
            return goals
        else:
            return None

    def get_vision_T_goal(self, origin_T_goal, ground_plane_rt_vision):
        vision_T_goal = self.vision_T_origin * origin_T_goal
        if not self.is_admittance():
            vision_T_goal.z = self.travel_z + ground_plane_rt_vision[2]
        else:
            vision_T_goal.z = ground_plane_rt_vision[2] + self.draw_z_offset
        return vision_T_goal

    def is_admittance(self):
        return self.current_origin_T_goals[0].z < self.below_z_is_admittance

    def get_next_vision_T_goals(self, ground_plane_rt_vision, read_new_line=True):
        if not read_new_line:
            if not self._last_line:
                return (False, None, False)
            origin_T_goals = self.convert_gcode_to_origin_T_goals(self._last_line)
            if not origin_T_goals:
                return (False, None, False)
            self.current_origin_T_goals = origin_T_goals
            return (self.is_admittance(),
                    [self.get_vision_T_goal(g, ground_plane_rt_vision) for g in origin_T_goals],
                    False)

        origin_T_goals = None
        while origin_T_goals is None:
            self._last_line = self.file.readline()
            if not self._last_line:
                return (False, None, False)
            if self._last_line.strip() == 'M0':
                return (False, None, True)
            origin_T_goals = self.convert_gcode_to_origin_T_goals(self._last_line)

        self.current_origin_T_goals = origin_T_goals
        if not self.is_admittance():
            return (False,
                    [self.get_vision_T_goal(g, ground_plane_rt_vision) for g in origin_T_goals],
                    False)

        # Draw move: batch all following admittance lines into one trajectory.
        vision_T_goals = [self.get_vision_T_goal(g, ground_plane_rt_vision)
                          for g in origin_T_goals]
        while True:
            pos = self.file.tell()
            next_line = self.file.readline()
            if not next_line:
                break
            if next_line.strip() == 'M0':
                self.file.seek(pos)
                break
            next_goals = self.convert_gcode_to_origin_T_goals(next_line)
            if next_goals is None:
                continue
            self.current_origin_T_goals = next_goals
            if not self.is_admittance():
                self.file.seek(pos)
                break
            self._last_line = next_line
            vision_T_goals.extend(
                self.get_vision_T_goal(g, ground_plane_rt_vision) for g in next_goals)

        return (True, vision_T_goals, False)

    def test_file_parsing(self):
        for line in self.file:
            self.convert_gcode_to_origin_T_goals(line)


# ---------------------------------------------------------------------------
# Arm helpers  (identical to gcode_manual)
# ---------------------------------------------------------------------------

def move_along_trajectory(frame, velocity, tool_T_goals, vision_T_task):
    task_T_vision = SE3Pose.from_proto(vision_T_task).inverse()
    last = None
    last_t = 0.0
    points = []
    for goal in tool_T_goals:
        if last is None:
            seconds, nanos = 0, 0
        else:
            dt = math.sqrt((last.x - goal.x)**2 + (last.y - goal.y)**2 +
                           (last.z - goal.z)**2) / velocity
            last_t += dt
            seconds = int(last_t)
            nanos = int((last_t - seconds) * 1e9)
        tg = task_T_vision * goal
        points.append(trajectory_pb2.SE3TrajectoryPoint(
            pose=geometry_pb2.SE3Pose(
                position=geometry_pb2.Vec3(x=tg.x, y=tg.y, z=tg.z),
                rotation=geometry_pb2.Quaternion(w=tg.rot.w, x=tg.rot.x,
                                                 y=tg.rot.y, z=tg.rot.z),
            ),
            time_since_reference=duration_pb2.Duration(seconds=seconds, nanos=nanos),
        ))
        last = goal
    return trajectory_pb2.SE3Trajectory(points=points)


def move_arm(robot_state, is_admittance, vision_T_goals, asc_client, cmd_client, velocity,
             allow_walking, vision_T_admittance, press_force_pct,
             api_send_frame, use_xy_cross, bias_force_x):
    if not is_admittance:
        goal = vision_T_goals[-1]
        p = goal.to_proto()
        arm_cmd = RobotCommandBuilder.arm_pose_command(
            p.position.x, p.position.y, p.position.z,
            p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z,
            VISION_FRAME_NAME, 2.0)
        cmd_client.robot_command(RobotCommandBuilder.build_synchro_command(arm_cmd))
        return

    traj = move_along_trajectory(api_send_frame, velocity, vision_T_goals, vision_T_admittance)
    cmd = arm_surface_contact_pb2.ArmSurfaceContact.Request(
        pose_trajectory_in_task=traj,
        root_frame_name=api_send_frame,
        root_tform_task=vision_T_admittance,
        press_force_percentage=geometry_pb2.Vec3(x=0, y=0, z=press_force_pct),
        x_axis=arm_surface_contact_pb2.ArmSurfaceContact.Request.AXIS_MODE_POSITION,
        y_axis=arm_surface_contact_pb2.ArmSurfaceContact.Request.AXIS_MODE_POSITION,
        z_axis=arm_surface_contact_pb2.ArmSurfaceContact.Request.AXIS_MODE_FORCE,
        max_linear_velocity=wrappers_pb2.DoubleValue(value=velocity),
    )
    cmd.press_force_percentage.z = press_force_pct
    cmd.xy_admittance = arm_surface_contact_pb2.ArmSurfaceContact.Request.ADMITTANCE_SETTING_OFF
    cmd.z_admittance  = arm_surface_contact_pb2.ArmSurfaceContact.Request.ADMITTANCE_SETTING_LOOSE
    cmd.xy_to_z_cross_term_admittance = (
        arm_surface_contact_pb2.ArmSurfaceContact.Request.ADMITTANCE_SETTING_VERY_STIFF
        if use_xy_cross else
        arm_surface_contact_pb2.ArmSurfaceContact.Request.ADMITTANCE_SETTING_OFF
    )
    cmd.bias_force_ewrt_body.CopyFrom(geometry_pb2.Vec3(x=bias_force_x, y=0, z=0))
    gripper = RobotCommandBuilder.claw_gripper_open_fraction_command(0)
    cmd.gripper_command.CopyFrom(
        gripper.synchronized_command.gripper_command.claw_gripper_command)
    cmd.is_robot_following_hand = allow_walking
    asc_client.arm_surface_contact_command(
        arm_surface_contact_service_pb2.ArmSurfaceContactCommand(request=cmd))


def get_transforms(use_vision_frame, robot_state):
    world = VISION_FRAME_NAME if use_vision_frame else ODOM_FRAME_NAME
    snap  = robot_state.kinematic_state.transforms_snapshot
    world_T_body      = get_a_tform_b(snap, world,           BODY_FRAME_NAME)
    world_T_flat_body = get_a_tform_b(snap, world,           GRAV_ALIGNED_BODY_FRAME_NAME)
    body_T_hand       = get_a_tform_b(snap, BODY_FRAME_NAME, HAND_FRAME_NAME)
    body_T_wrist      = get_a_tform_b(snap, BODY_FRAME_NAME, WR1_FRAME_NAME)
    world_T_hand      = world_T_body * body_T_hand
    odom_T_body       = get_a_tform_b(snap, ODOM_FRAME_NAME, BODY_FRAME_NAME)
    return (world_T_body, world_T_flat_body, body_T_hand, world_T_hand, body_T_wrist, odom_T_body)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def run(initial_gcode_file, vicon_client, test_only=False):
    cfg = configparser.ConfigParser()
    cfg.read(os.path.join(_HERE, 'gcode.cfg'))
    g = cfg['General']

    tool_length      = g.getfloat('tool_length',           0.0)
    margin           = g.getfloat('margin',                0.90)
    min_dist_to_goal = g.getfloat('min_dist_to_goal',      0.03)
    allow_walking    = g.getboolean('allow_walking',        False)
    velocity         = g.getfloat('velocity',              0.15)
    press_force_pct  = g.getfloat('press_force_percent',  -0.002)
    below_z_adm      = g.getfloat('below_z_is_admittance', 0.0)
    travel_z         = g.getfloat('travel_z',              0.05)
    draw_z_offset    = g.getfloat('draw_z_offset',         0.005)
    draw_on_wall     = g.getboolean('draw_on_wall',         False)
    use_vision_frame = g.getboolean('use_vision_frame',     True)
    use_xy_cross     = g.getboolean('use_xy_to_z_cross_term', False)
    bias_force_x     = g.getfloat('bias_force_x',          0.0)

    api_send_frame = VISION_FRAME_NAME if use_vision_frame else ODOM_FRAME_NAME

    if test_only:
        bosdyn.client.util.setup_logging(verbose=False)
        sdk   = bosdyn.client.create_standard_sdk('GcodeViconClient')
        robot = sdk.create_robot('0.0.0.0')
        gcode = GcodeReader(initial_gcode_file, tool_length, 0.001, robot.logger,
                            below_z_adm, travel_z, draw_on_wall)
        gcode.test_file_parsing()
        print('File parsing OK.')
        return

    robot_ip = _require_env('SPOT_IP')
    username = _require_env('SPOT_USER')
    password = _require_env('SPOT_PASSWORD')

    bosdyn.client.util.setup_logging(verbose=False)
    sdk   = bosdyn.client.create_standard_sdk('GcodeViconClient')
    robot = sdk.create_robot(robot_ip)
    robot.authenticate(username, password)
    robot.time_sync.wait_for_sync()
    assert robot.has_arm(), 'Robot requires an arm.'

    asc_client   = robot.ensure_client(ArmSurfaceContactClient.default_service_name)
    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    cmd_client   = robot.ensure_client(RobotCommandClient.default_service_name)
    state_client = robot.ensure_client(RobotStateClient.default_service_name)
    estop_client = robot.ensure_client(EstopClient.default_service_name)

    estop_ep = EstopEndpoint(estop_client, name='gcode_vicon_estop', estop_timeout=9.0)
    estop_ep.force_simple_setup()

    vision_T_admittance = geometry_pb2.SE3Pose(
        position=geometry_pb2.Vec3(x=0, y=0, z=0),
        rotation=geometry_pb2.Quaternion(w=1, x=0, y=0, z=0))

    def _draw_file(gcode_file):
        # ── Read canvas from Vicon ────────────────────────────────────────
        print('  Waiting for canvas from Vicon…')
        deadline = time.time() + 8.0
        frame = None
        while time.time() < deadline:
            frame = vicon_client.latest_frame
            if frame and frame.canvas and frame.canvas.is_valid():
                break
            time.sleep(0.1)
        if frame is None or frame.canvas is None or not frame.canvas.is_valid():
            raise RuntimeError('Canvas not visible in Vicon.')

        c = frame.canvas

        # ── Auto-scale: fit gcode within canvas (aspect-ratio preserved) ──
        x_min, x_max, y_min, y_max = scan_gcode_bounds(gcode_file)
        gw = x_max - x_min
        gh = y_max - y_min

        if gw > 0 and gh > 0:
            canvas_w_m = (c.width_mm  / 1000.0) * margin
            canvas_h_m = (c.height_mm / 1000.0) * margin
            scale = min(canvas_w_m / gw, canvas_h_m / gh)
            print(f'  Gcode extents   {gw:.1f} x {gh:.1f} raw units')
            print(f'  Canvas          {c.width_mm:.0f} x {c.height_mm:.0f} mm'
                  f'  (margin {margin*100:.0f}%)')
            print(f'  Auto-scale      {scale:.6f} m/unit'
                  f'  ->  drawing {gw*scale*1000:.0f} x {gh*scale*1000:.0f} mm on canvas')
        else:
            print('  [WARN] Could not determine gcode extents — using scale 0.001')
            scale = 0.001

        # ── Center drawing within canvas ──────────────────────────────────
        # Shift gcode coords so the bounding box starts at (0, 0), then offset
        # the origin position so equal blank space surrounds the drawing.
        pad_x = (c.width_mm  / 1000.0 - gw * scale) / 2.0  # metres, along canvas x_axis
        pad_y = (c.height_mm / 1000.0 - gh * scale) / 2.0  # metres, along canvas y_axis

        tl_m  = c.corners[0] / 1000.0   # canvas TL corner in metres
        x_ax  = c.x_axis                # unit vec TL→TR
        y_ax  = c.y_axis                # unit vec TL→BL

        # gcode (x_min, y_min) should land at TL + pad_x*x_ax + pad_y*y_ax.
        # With gcode_start_x=x_min, the reader maps x_min → 0, so gcode (0,0)
        # = the origin pose.  Origin is placed at TL + padding offsets.
        origin_m = tl_m + x_ax * pad_x + y_ax * pad_y

        print(f'  Canvas TL       ({tl_m[0]*1000:+.0f}, {tl_m[1]*1000:+.0f},'
              f' {tl_m[2]*1000:+.0f}) mm')
        print(f'  Gcode origin    ({origin_m[0]*1000:+.0f}, {origin_m[1]*1000:+.0f},'
              f' {origin_m[2]*1000:+.0f}) mm'
              f'  (pads {pad_x*1000:.0f} x {pad_y*1000:.0f} mm)')

        canvas_z_m = float(np.mean([c.corners[i][2] for i in range(4)])) / 1000.0
        ground_plane_rt_vision = [0.0, 0.0, canvas_z_m]

        # ── Build GcodeReader ─────────────────────────────────────────────
        gcode = GcodeReader(gcode_file, tool_length, scale, robot.logger,
                            below_z_adm, travel_z, draw_on_wall,
                            gcode_start_x=x_min, gcode_start_y=y_min,
                            draw_z_offset=draw_z_offset)

        # Set drawing frame directly from canvas geometry.
        # gcode +X → canvas x_axis (TL→TR), gcode +Y → canvas y_axis (TL→BL).
        # Z column = x_ax × y_ax (= canvas normal) keeps det = +1.
        gcode.vision_T_origin = SE3Pose(
            x=float(origin_m[0]),
            y=float(origin_m[1]),
            z=float(origin_m[2]),
            rot=Quat.from_matrix(np.column_stack([x_ax, y_ax, c.normal])),
        )

        # ── Move arm above canvas centre ──────────────────────────────────
        robot_state = state_client.get_robot_state()
        (_, odom_T_flat_body, _, _, _, _) = get_transforms(False, robot_state)
        start_z_odom     = canvas_z_m + 0.30
        flat_body_T_hand = SE3Pose(0.60, 0, start_z_odom - odom_T_flat_body.z,
                                   math_helpers.Quat(w=0.707, x=0, y=0.707, z=0))
        odom_T_hand_start = odom_T_flat_body * flat_body_T_hand
        p = odom_T_hand_start.to_proto()
        arm_cmd = RobotCommandBuilder.arm_pose_command(
            p.position.x, p.position.y, p.position.z,
            p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z,
            ODOM_FRAME_NAME, 0.000001)
        cmd_id = cmd_client.robot_command(RobotCommandBuilder.build_synchro_command(arm_cmd))
        block_until_arm_arrives(cmd_client, cmd_id)

        # ── Confirm before drawing ────────────────────────────────────────
        print()
        print('=' * 60)
        print(f' File: {os.path.basename(gcode_file)}')
        print(' Arm is above the canvas. Press Enter to begin drawing,')
        print(' or Ctrl-C to abort.')
        print('=' * 60)
        input()

        # ── Execute gcode ─────────────────────────────────────────────────
        robot_state = state_client.get_robot_state()
        (vision_T_body, _, body_T_hand, _, _, odom_T_body) = get_transforms(True, robot_state)
        vision_T_odom = vision_T_body * odom_T_body.inverse()

        (is_admittance, vision_T_goals, is_pause) = \
            gcode.get_next_vision_T_goals(ground_plane_rt_vision)

        while is_pause:
            input('M0 pause — press Enter to continue…')
            (is_admittance, vision_T_goals, is_pause) = \
                gcode.get_next_vision_T_goals(ground_plane_rt_vision)

        if vision_T_goals is None:
            robot.logger.info('Gcode file is empty.')
            return

        move_arm(robot_state, is_admittance, vision_T_goals, asc_client, cmd_client, velocity,
                 allow_walking, vision_T_admittance, press_force_pct,
                 api_send_frame, use_xy_cross, bias_force_x)

        odom_T_hand_goal = vision_T_odom.inverse() * vision_T_goals[-1]
        last_admittance  = is_admittance

        while True:
            robot_state = state_client.get_robot_state()
            (vision_T_body, _, body_T_hand, _, _, odom_T_body) = get_transforms(True, robot_state)
            vision_T_odom = vision_T_body * odom_T_body.inverse()

            adm_inv      = SE3Pose.from_proto(vision_T_admittance).inverse()
            hand_in_adm  = adm_inv * vision_T_odom * odom_T_body * body_T_hand
            goal_in_adm  = adm_inv * vision_T_odom * odom_T_hand_goal

            if is_admittance:
                dist = math.sqrt((hand_in_adm.x - goal_in_adm.x)**2 +
                                 (hand_in_adm.y - goal_in_adm.y)**2)
            else:
                dist = math.sqrt((hand_in_adm.x - goal_in_adm.x)**2 +
                                 (hand_in_adm.y - goal_in_adm.y)**2 +
                                 (hand_in_adm.z - goal_in_adm.z)**2)

            if dist < min_dist_to_goal:
                (is_admittance, vision_T_goals, is_pause) = \
                    gcode.get_next_vision_T_goals(ground_plane_rt_vision)

                while is_pause:
                    input('M0 pause — press Enter to continue…')
                    (is_admittance, vision_T_goals, is_pause) = \
                        gcode.get_next_vision_T_goals(ground_plane_rt_vision)

                if vision_T_goals is None:
                    robot.logger.info('Gcode program finished.')
                    break

                move_arm(robot_state, is_admittance, vision_T_goals, asc_client, cmd_client,
                         velocity, allow_walking, vision_T_admittance, press_force_pct,
                         api_send_frame, use_xy_cross, bias_force_x)
                odom_T_hand_goal = vision_T_odom.inverse() * vision_T_goals[-1]

                if is_admittance != last_admittance:
                    time.sleep(3.0 if is_admittance else 1.0)
                last_admittance = is_admittance

            elif not is_admittance:
                (is_admittance, vision_T_goals, is_pause) = \
                    gcode.get_next_vision_T_goals(ground_plane_rt_vision, read_new_line=False)

            time.sleep(0.05)

    # ── Robot startup ─────────────────────────────────────────────────────
    with (
        EstopKeepAlive(estop_ep),
        bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True),
    ):
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), 'Power on failed.'
        blocking_stand(cmd_client, timeout_sec=10)
        robot.logger.info('Robot standing.')

        robot.logger.info('Closing gripper on marker.')
        gripper_cmd = RobotCommandBuilder.claw_gripper_close_command()
        gripper_cmd.synchronized_command.gripper_command.claw_gripper_command.maximum_torque.value = 15.0
        cmd_client.robot_command(gripper_cmd)
        time.sleep(1.0)

        current_file = initial_gcode_file
        while True:
            _draw_file(current_file)
            blocking_stand(cmd_client, timeout_sec=10)

            print()
            print('┌─────────────────────────────────┐')
            print('│  Drawing complete. What next?   │')
            print('│  1  Stow arm and sit            │')
            print('│  2  Draw another file           │')
            print('└─────────────────────────────────┘')
            choice = input('> ').strip()

            if choice == '2':
                current_file = pick_gcode_file()
            else:
                break

        robot.logger.info('Stowing arm and sitting…')
        cmd_client.robot_command(RobotCommandBuilder.arm_stow_command())
        time.sleep(2)
        cmd_client.robot_command(RobotCommandBuilder.synchro_sit_command())
        time.sleep(2)
        robot.power_off(cut_immediately=False, timeout_sec=20)
        robot.logger.info('Done.')


def main():
    parser = argparse.ArgumentParser(
        description='Draw a gcode file bounded within the Vicon canvas — marker pre-taped to gripper')
    parser.add_argument('gcode', nargs='?', default=None,
                        help='Path to .gcode or .ngc file. Omit to pick from g_codes/.')
    parser.add_argument('--vicon', metavar='HOST:PORT', default=None,
                        help='Vicon address (default: VICON_ADDRESS from .env)')
    parser.add_argument('--test-file-parsing', action='store_true',
                        help='Parse gcode without connecting to robot or Vicon')
    args = parser.parse_args()

    if args.gcode is None:
        args.gcode = pick_gcode_file()
    elif not os.path.exists(args.gcode):
        raise SystemExit(f'File not found: {args.gcode}')

    if args.test_file_parsing:
        try:
            run(args.gcode, vicon_client=None, test_only=True)
        except Exception:
            bosdyn.client.util.get_logger().exception('Fatal error')
            sys.exit(1)
        return

    vicon_addr = args.vicon or os.environ.get('VICON_ADDRESS')
    if not vicon_addr:
        raise SystemExit('No VICON_ADDRESS set and --vicon not specified.')

    print(f'Connecting to Vicon at {vicon_addr}…')
    vicon = ViconClient(host=vicon_addr)
    vicon.start()
    deadline = time.time() + 6.0
    while vicon.latest_frame is None and time.time() < deadline:
        time.sleep(0.05)
    if vicon.latest_frame is None:
        vicon.stop()
        raise SystemExit('No Vicon frames received.')

    frame = vicon.latest_frame
    canvas_ok = frame.canvas is not None and frame.canvas.is_valid()
    print(f'Vicon OK  |  canvas: {"OK" if canvas_ok else "NO DATA"}')
    if not canvas_ok:
        vicon.stop()
        raise SystemExit('Canvas not visible — check corner markers in Vicon Tracker.')

    try:
        run(args.gcode, vicon_client=vicon)
    except Exception:
        bosdyn.client.util.get_logger().exception('Fatal error')
        sys.exit(1)
    finally:
        vicon.stop()


if __name__ == '__main__':
    main()
