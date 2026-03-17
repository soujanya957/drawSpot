"""
draw_gcode.robot.gcode_exec
============================
Gcode parsing (G00/G01/G02/G03) and force-compliant arm execution via
ArmSurfaceContact.  The Vicon canvas TL corner and orientation axes are used
as the gcode coordinate origin — no manual origin entry required.

Public API
----------
  execute(vicon_client, asc_client, cmd_client, state_client,
          gcode_path, scale, tool_length)  -> None

      Reads the current canvas from Vicon, sets the gcode origin to the TL
      corner, runs the gcode file using ArmSurfaceContact, and honours
      SPACE (pause) / RETURN (estop) from draw_gcode._ctrl throughout.

  scan_gcode_bounds(file_path, scale) -> (x_min, x_max, y_min, y_max)  metres
  print_bounds_check(file_path, scale, canvas) -> bool
"""

import math
import time

import numpy as np
from google.protobuf import duration_pb2, wrappers_pb2

from bosdyn.api import (arm_surface_contact_pb2, arm_surface_contact_service_pb2,
                        geometry_pb2, trajectory_pb2)
from bosdyn.client.frame_helpers import (
    ODOM_FRAME_NAME, VISION_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME,
    BODY_FRAME_NAME, HAND_FRAME_NAME, WR1_FRAME_NAME, get_a_tform_b,
)
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.robot_command import RobotCommandBuilder, block_until_arm_arrives

import draw_gcode._ctrl as _ctrl

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

VELOCITY            = 0.25    # arm speed m/s
PRESS_FORCE_PERCENT = -20.0   # negative = press down
BELOW_Z_ADMITTANCE  = 0.0     # gcode z below this (m) → force-controlled
TRAVEL_Z            = 0.03    # gcode z for travel moves (m)
MIN_DIST_TO_GOAL    = 0.02    # m — how close arm must be before advancing


# ---------------------------------------------------------------------------
# Gcode helpers
# ---------------------------------------------------------------------------

def _make_orthogonal(primary, secondary):
    p = primary / np.linalg.norm(primary)
    s = secondary / np.linalg.norm(secondary)
    u = s - np.dot(p, s) / np.dot(s, s) * p
    return u / np.linalg.norm(u)


class GcodeReader:
    def __init__(self, file_path, tool_length, scale, logger,
                 below_z_is_admittance, travel_z, draw_on_wall,
                 gcode_start_x=0, gcode_start_y=0):
        self.file = open(file_path, 'r')
        self.tool_length = tool_length
        self.scale = scale
        self.logger = logger
        self.below_z_is_admittance = below_z_is_admittance
        self.travel_z = travel_z
        self.draw_on_wall = draw_on_wall
        self.gcode_start_x = gcode_start_x
        self.gcode_start_y = gcode_start_y
        self.current_origin_T_goals = None
        self.last_x = self.last_y = self.last_z = 0

    def set_origin(self, vision_T_origin, vision_T_admittance_frame):
        if not self.draw_on_wall:
            zhat = [0.0, 0.0, 1.0]
            x1, x2, x3 = vision_T_origin.rot.transform_point(-1.0, 0.0, 0.0)
            xhat = _make_orthogonal(zhat, [x1, x2, x3])
            yhat = np.cross(zhat, xhat)
            mat  = np.array([xhat, yhat, zhat]).T
            self.vision_T_origin = SE3Pose(
                vision_T_origin.x, vision_T_origin.y, vision_T_origin.z,
                Quat.from_matrix(mat))
        else:
            x1, x2, x3 = vision_T_admittance_frame.rot.transform_point(0, -1, 0)
            y1, y2, y3 = vision_T_admittance_frame.rot.transform_point(1,  0, 0)
            z1, z2, z3 = vision_T_admittance_frame.rot.transform_point(0,  0, 1)
            mat = np.array([[x1, y1, z1], [x2, y2, z2], [x3, y3, z3]]).T
            self.vision_T_origin = SE3Pose(
                vision_T_origin.x, vision_T_origin.y, vision_T_origin.z,
                Quat.from_matrix(mat))

    def _origin_Q_goal(self):
        mat = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]], dtype=float).T
        return Quat.from_matrix(mat)

    def _convert_line(self, line):
        for ch in ('(', '%', ';'):
            idx = line.find(ch)
            if idx >= 0:
                line = line[:idx]
        parts = line.split()
        if not parts:
            return None
        cmd = parts[0]
        if cmd in ('G00', 'G0', 'G01', 'G1'):
            x, y, z = self.last_x, self.last_y, self.last_z
            for p in parts[1:]:
                if p[0] == 'X':
                    x = (float(p[1:]) - self.gcode_start_x) * self.scale
                elif p[0] == 'Y':
                    y = (float(p[1:]) - self.gcode_start_y) * self.scale
                elif p[0] == 'Z':
                    z = float(p[1:]) * self.scale
            self.last_x, self.last_y, self.last_z = x, y, z
            return [SE3Pose(x, y, z, self._origin_Q_goal())]
        elif cmd in ('G02', 'G2', 'G03', 'G3'):
            x, y, z = self.last_x, self.last_y, self.last_z
            i_val = j_val = k_val = 0.0
            for p in parts[1:]:
                if p[0] == 'X':
                    x = (float(p[1:]) - self.gcode_start_x) * self.scale
                elif p[0] == 'Y':
                    y = (float(p[1:]) - self.gcode_start_y) * self.scale
                elif p[0] == 'Z':
                    z = float(p[1:]) * self.scale
                elif p[0] == 'I':
                    i_val = float(p[1:]) * self.scale
                elif p[0] == 'J':
                    j_val = float(p[1:]) * self.scale
                elif p[0] == 'K':
                    k_val = float(p[1:]) * self.scale
            clockwise = cmd in ('G02', 'G2')
            if i_val != 0 or j_val != 0:
                lp = [self.last_x, self.last_y]; ep = [x, y]
                cp = [self.last_x + i_val, self.last_y + j_val]
                def make_pt(px, py): return SE3Pose(px, py, self.last_z, self._origin_Q_goal())
            elif i_val != 0 or k_val != 0:
                lp = [self.last_x, self.last_z]; ep = [x, z]
                cp = [self.last_x + i_val, self.last_z + k_val]
                def make_pt(px, py): return SE3Pose(px, self.last_y, py, self._origin_Q_goal())
            else:
                lp = [self.last_y, self.last_z]; ep = [y, z]
                cp = [self.last_y + j_val, self.last_z + k_val]
                def make_pt(px, py): return SE3Pose(self.last_x, px, py, self._origin_Q_goal())
            lr = np.array(lp) - np.array(cp)
            er = np.array(ep) - np.array(cp)
            lt = math.atan2(lr[1], lr[0]); et = math.atan2(er[1], er[0])
            if clockwise:
                if lt < et: lt += 2 * math.pi
            else:
                if lt > et: et += 2 * math.pi
            n = max(int(abs(et - lt) / 0.01), 1)
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

    def _get_vision_T_goal(self, origin_T_goal, ground_plane_rt_vision):
        vision_T_goal = self.vision_T_origin * origin_T_goal
        if not self.is_admittance():
            vision_T_goal.z = self.travel_z + ground_plane_rt_vision[2]
        else:
            vision_T_goal.z = ground_plane_rt_vision[2]
        return vision_T_goal

    def is_admittance(self):
        return self.current_origin_T_goals[0].z < self.below_z_is_admittance

    def get_next_vision_T_goals(self, ground_plane_rt_vision, read_new_line=True):
        origin_T_goals = None
        while not origin_T_goals:
            if read_new_line:
                self.last_line = self.file.readline()
            if not self.last_line:
                return (False, None, False)
            if self.last_line.strip() == 'M0':
                return (False, None, True)
            origin_T_goals = self._convert_line(self.last_line)
        self.current_origin_T_goals = origin_T_goals
        vision_T_goals = [self._get_vision_T_goal(g, ground_plane_rt_vision)
                          for g in origin_T_goals]
        return (self.is_admittance(), vision_T_goals, False)


# ---------------------------------------------------------------------------
# Robot state helpers
# ---------------------------------------------------------------------------

def _get_transforms(robot_state):
    snap  = robot_state.kinematic_state.transforms_snapshot
    world_T_body      = get_a_tform_b(snap, ODOM_FRAME_NAME, BODY_FRAME_NAME)
    world_T_flat_body = get_a_tform_b(snap, ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
    body_T_hand       = get_a_tform_b(snap, BODY_FRAME_NAME, HAND_FRAME_NAME)
    body_T_wrist      = get_a_tform_b(snap, BODY_FRAME_NAME, WR1_FRAME_NAME)
    world_T_hand      = world_T_body * body_T_hand
    return (world_T_body, world_T_flat_body, body_T_hand, world_T_hand, body_T_wrist, world_T_body)


def _move_arm(robot_state, is_admittance, vision_T_goals, asc_client, velocity,
              vision_T_admittance, press_force_percentage):
    task_T_vision = SE3Pose.from_proto(vision_T_admittance).inverse()
    points = []
    last = None
    t = 0.0
    for goal in vision_T_goals:
        if last is None:
            seconds, nanos = 0, 0
        else:
            dt = math.sqrt((last.x - goal.x)**2 + (last.y - goal.y)**2 +
                           (last.z - goal.z)**2) / velocity
            t += dt
            seconds, nanos = int(t), int((t - int(t)) * 1e9)
        tg = task_T_vision * goal
        points.append(trajectory_pb2.SE3TrajectoryPoint(
            pose=geometry_pb2.SE3Pose(
                position=geometry_pb2.Vec3(x=tg.x, y=tg.y, z=tg.z),
                rotation=geometry_pb2.Quaternion(
                    w=tg.rot.w, x=tg.rot.x, y=tg.rot.y, z=tg.rot.z),
            ),
            time_since_reference=duration_pb2.Duration(seconds=seconds, nanos=nanos),
        ))
        last = goal

    traj  = trajectory_pb2.SE3Trajectory(points=points)
    press = geometry_pb2.Vec3(x=0, y=0, z=press_force_percentage)
    cmd   = arm_surface_contact_pb2.ArmSurfaceContact.Request(
        pose_trajectory_in_task=traj,
        root_frame_name=ODOM_FRAME_NAME,
        root_tform_task=vision_T_admittance,
        press_force_percentage=press,
        x_axis=arm_surface_contact_pb2.ArmSurfaceContact.Request.AXIS_MODE_POSITION,
        y_axis=arm_surface_contact_pb2.ArmSurfaceContact.Request.AXIS_MODE_POSITION,
        z_axis=arm_surface_contact_pb2.ArmSurfaceContact.Request.AXIS_MODE_POSITION,
        max_linear_velocity=wrappers_pb2.DoubleValue(value=velocity),
    )
    if is_admittance:
        cmd.z_axis = arm_surface_contact_pb2.ArmSurfaceContact.Request.AXIS_MODE_FORCE
        cmd.press_force_percentage.z = press_force_percentage
        cmd.xy_admittance = arm_surface_contact_pb2.ArmSurfaceContact.Request.ADMITTANCE_SETTING_OFF
        cmd.z_admittance  = arm_surface_contact_pb2.ArmSurfaceContact.Request.ADMITTANCE_SETTING_LOOSE
        cmd.xy_to_z_cross_term_admittance = (
            arm_surface_contact_pb2.ArmSurfaceContact.Request.ADMITTANCE_SETTING_VERY_STIFF)
        cmd.bias_force_ewrt_body.CopyFrom(geometry_pb2.Vec3(x=0, y=0, z=0))
    else:
        cmd.bias_force_ewrt_body.CopyFrom(geometry_pb2.Vec3(x=0, y=0, z=0))

    gripper = RobotCommandBuilder.claw_gripper_open_fraction_command(0)
    cmd.gripper_command.CopyFrom(
        gripper.synchronized_command.gripper_command.claw_gripper_command)
    cmd.is_robot_following_hand = False
    asc_client.arm_surface_contact_command(
        arm_surface_contact_service_pb2.ArmSurfaceContactCommand(request=cmd))


# ---------------------------------------------------------------------------
# Gcode bounds check
# ---------------------------------------------------------------------------

def scan_gcode_bounds(file_path: str, scale: float):
    """Return (x_min, x_max, y_min, y_max) in metres."""
    x_vals, y_vals = [0.0], [0.0]
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
            if cmd not in ('G00','G0','G01','G1','G02','G2','G03','G3'):
                continue
            x, y, i_val, j_val = cur_x, cur_y, 0.0, 0.0
            for p in parts[1:]:
                if not p: continue
                if p[0] == 'X': x = float(p[1:]) * scale
                elif p[0] == 'Y': y = float(p[1:]) * scale
                elif p[0] == 'I': i_val = float(p[1:]) * scale
                elif p[0] == 'J': j_val = float(p[1:]) * scale
            x_vals.append(x); y_vals.append(y)
            if cmd in ('G02','G2','G03','G3') and (i_val or j_val):
                x_vals.append(cur_x + i_val)
                y_vals.append(cur_y + j_val)
            cur_x, cur_y = x, y
    return min(x_vals), max(x_vals), min(y_vals), max(y_vals)


def print_bounds_check(file_path: str, scale: float, canvas) -> bool:
    """Print gcode extents vs canvas. Returns True if gcode fits."""
    x_min, x_max, y_min, y_max = scan_gcode_bounds(file_path, scale)
    gw = (x_max - x_min) * 1000
    gh = (y_max - y_min) * 1000
    print(f"  Gcode extents  X: {x_min*1000:.1f} … {x_max*1000:.1f} mm  ({gw:.1f} mm wide)")
    print(f"                 Y: {y_min*1000:.1f} … {y_max*1000:.1f} mm  ({gh:.1f} mm tall)")
    print(f"  Canvas size    {canvas.width_mm:.1f} mm × {canvas.height_mm:.1f} mm")
    fits = True
    if x_min < 0 or y_min < 0:
        print(f"  \033[93m[WARN] Gcode has negative coordinates — will draw outside TL corner\033[0m")
        fits = False
    if x_max * 1000 > canvas.width_mm:
        print(f"  \033[93m[WARN] Gcode X overflows canvas by {x_max*1000 - canvas.width_mm:.1f} mm\033[0m")
        fits = False
    if y_max * 1000 > canvas.height_mm:
        print(f"  \033[93m[WARN] Gcode Y overflows canvas by {y_max*1000 - canvas.height_mm:.1f} mm\033[0m")
        fits = False
    if fits:
        print(f"  \033[92m[OK]   Gcode fits within canvas.\033[0m")
    return fits


# ---------------------------------------------------------------------------
# Auto-scale: fit gcode within canvas
# ---------------------------------------------------------------------------

def compute_canvas_scale(gcode_path: str, canvas, margin: float = 0.90) -> float:
    """
    Compute the scale (metres per gcode unit) that fits the gcode drawing
    within the canvas, preserving aspect ratio and leaving a *margin* border.

    Works regardless of whether gcode units are mm, inches, or arbitrary —
    it reads raw unit extents (scale=1.0) and maps them to canvas dimensions.

    Args:
        gcode_path: path to the .gcode file
        canvas:     CanvasFrame from Vicon (provides width_mm / height_mm)
        margin:     fraction of canvas to fill, e.g. 0.90 = 90% (10% border)

    Returns:
        scale in metres per gcode unit
    """
    x_min, x_max, y_min, y_max = scan_gcode_bounds(gcode_path, scale=1.0)
    gw = x_max - x_min   # raw gcode width
    gh = y_max - y_min   # raw gcode height

    if gw <= 0 or gh <= 0:
        print("  [WARN] Could not determine gcode extents — using default scale 0.001")
        return 0.001

    canvas_w_m = (canvas.width_mm  / 1000.0) * margin
    canvas_h_m = (canvas.height_mm / 1000.0) * margin

    scale = min(canvas_w_m / gw, canvas_h_m / gh)

    fitted_w_mm = gw * scale * 1000
    fitted_h_mm = gh * scale * 1000
    print(f"  Gcode extents   {gw:.2f} × {gh:.2f} raw units")
    print(f"  Canvas          {canvas.width_mm:.0f} × {canvas.height_mm:.0f} mm"
          f"  (margin {margin*100:.0f}%)")
    print(f"  Auto-scale      {scale:.6f} m/unit"
          f"  →  drawing {fitted_w_mm:.0f} × {fitted_h_mm:.0f} mm on canvas")
    return scale


# ---------------------------------------------------------------------------
# Canvas origin from Vicon
# ---------------------------------------------------------------------------

def _build_canvas_origin(frame) -> SE3Pose:
    """SE3Pose (odom frame) for gcode (0,0) = canvas TL corner."""
    c   = frame.canvas
    tl  = c.corners[0]
    mat = np.column_stack([c.x_axis, c.y_axis, c.normal])
    return SE3Pose(
        x=float(tl[0]) / 1000.0,
        y=float(tl[1]) / 1000.0,
        z=float(tl[2]) / 1000.0,
        rot=Quat.from_matrix(mat),
    )


# ---------------------------------------------------------------------------
# Core drawing loop
# ---------------------------------------------------------------------------

def _draw_loop(asc_client, cmd_client, state_client,
               gcode_path: str, scale: float, tool_length: float,
               odom_T_origin: SE3Pose, canvas_z: float,
               canvas_width_mm: float = None, canvas_height_mm: float = None) -> None:
    import logging
    logger = logging.getLogger("draw_gcode.gcode_exec")

    print(f"  Canvas origin  odom ({odom_T_origin.x:.3f},"
          f" {odom_T_origin.y:.3f}, {odom_T_origin.z:.3f}) m"
          f"  |  canvas_z {canvas_z:.3f} m  |  scale {scale}")

    if canvas_width_mm is not None and canvas_height_mm is not None:
        class _FakeCanvas:
            width_mm  = canvas_width_mm
            height_mm = canvas_height_mm
        print_bounds_check(gcode_path, scale, _FakeCanvas())
    print()

    vision_T_admittance = geometry_pb2.SE3Pose(
        position=geometry_pb2.Vec3(x=0, y=0, z=0),
        rotation=geometry_pb2.Quaternion(w=1, x=0, y=0, z=0),
    )

    gcode = GcodeReader(
        file_path=gcode_path,
        tool_length=tool_length,
        scale=scale,
        logger=logger,
        below_z_is_admittance=BELOW_Z_ADMITTANCE,
        travel_z=TRAVEL_Z,
        draw_on_wall=False,
    )
    gcode.set_origin(odom_T_origin, SE3Pose.from_proto(vision_T_admittance))
    print("  Gcode origin set.\n")

    ground_plane = [0.0, 0.0, canvas_z]

    # Move arm to starting position
    robot_state = state_client.get_robot_state()
    (_, odom_T_flat_body, _, _, _, _) = _get_transforms(robot_state)

    flat_body_T_hand  = SE3Pose(0.75, 0, -0.35, Quat(w=0.707, x=0, y=0.707, z=0))
    odom_T_hand_start = odom_T_flat_body * flat_body_T_hand
    p                 = odom_T_hand_start.to_proto()

    arm_cmd = RobotCommandBuilder.arm_pose_command(
        p.position.x, p.position.y, p.position.z,
        p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z,
        ODOM_FRAME_NAME, 0.000001,
    )
    cmd_id = cmd_client.robot_command(RobotCommandBuilder.build_synchro_command(arm_cmd))
    block_until_arm_arrives(cmd_client, cmd_id)
    print("  Arm at start position.")

    if not _ctrl.check():
        return

    print("  Executing gcode…\n")

    def _wait_m0():
        print("  [M0] Gcode pause — press SPACE to continue", flush=True)
        while True:
            if not _ctrl.check():
                return False
            time.sleep(0.1)
            nonlocal vision_T_goals, is_admittance, is_pause
            (is_admittance, vision_T_goals, is_pause) = \
                gcode.get_next_vision_T_goals(ground_plane)
            if not is_pause:
                return True

    (is_admittance, vision_T_goals, is_pause) = gcode.get_next_vision_T_goals(ground_plane)
    if is_pause:
        if not _wait_m0():
            return
    if vision_T_goals is None:
        print("  Gcode file is empty.")
        return

    robot_state = state_client.get_robot_state()
    _move_arm(robot_state, is_admittance, vision_T_goals,
              asc_client, VELOCITY, vision_T_admittance, PRESS_FORCE_PERCENT)

    odom_T_hand_goal = vision_T_goals[-1]
    last_admittance  = is_admittance

    while True:
        if not _ctrl.check():
            break

        robot_state = state_client.get_robot_state()
        (_, _, _, odom_T_hand, _, _) = _get_transforms(robot_state)

        if is_admittance:
            dist = math.sqrt((odom_T_hand.x - odom_T_hand_goal.x)**2 +
                             (odom_T_hand.y - odom_T_hand_goal.y)**2)
        else:
            dist = math.sqrt((odom_T_hand.x - odom_T_hand_goal.x)**2 +
                             (odom_T_hand.y - odom_T_hand_goal.y)**2 +
                             (odom_T_hand.z - odom_T_hand_goal.z)**2)

        if dist < MIN_DIST_TO_GOAL:
            (is_admittance, vision_T_goals, is_pause) = \
                gcode.get_next_vision_T_goals(ground_plane)
            if is_pause:
                if not _wait_m0():
                    break
            if vision_T_goals is None:
                print("  Gcode complete.")
                break

            _move_arm(robot_state, is_admittance, vision_T_goals,
                      asc_client, VELOCITY, vision_T_admittance, PRESS_FORCE_PERCENT)
            odom_T_hand_goal = vision_T_goals[-1]

            if is_admittance != last_admittance:
                time.sleep(3.0 if is_admittance else 1.0)
            last_admittance = is_admittance

        elif not is_admittance:
            (is_admittance, vision_T_goals, is_pause) = \
                gcode.get_next_vision_T_goals(ground_plane, read_new_line=False)

        time.sleep(0.05)


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------

def execute(vicon_client, asc_client, cmd_client, state_client,
            gcode_path: str, scale: float = None, tool_length: float = 0.0) -> None:
    """
    Execute a gcode file using the Vicon canvas as the coordinate origin.

    The canvas TL corner (from Vicon) becomes gcode (0, 0).
    The canvas x_axis becomes gcode +X, y_axis becomes gcode +Y.

    Args:
        vicon_client:   Running ViconClient with canvas visible.
        asc_client:     ArmSurfaceContactClient.
        cmd_client:     RobotCommandClient.
        state_client:   RobotStateClient.
        gcode_path:     Path to .gcode file.
        scale:          Metres per gcode unit.  Pass None (default) to
                        auto-scale so the drawing fills 90% of the canvas.
        tool_length:    Marker/brush length beyond wrist in metres (default 0).
    """
    print("  Reading canvas from Vicon…")
    deadline = time.time() + 5.0
    frame = None
    while time.time() < deadline:
        frame = vicon_client.latest_frame
        if frame and frame.canvas and frame.canvas.is_valid():
            break
        time.sleep(0.1)
    if frame is None or frame.canvas is None or not frame.canvas.is_valid():
        raise RuntimeError("Canvas not visible in Vicon — cannot set gcode origin.")

    c = frame.canvas

    # Auto-scale if no explicit scale provided
    if scale is None:
        print("  Computing auto-scale to fit canvas…")
        scale = compute_canvas_scale(gcode_path, c)
    else:
        print(f"  Using explicit scale {scale} m/unit")
        print_bounds_check(gcode_path, scale, c)

    odom_T_origin = _build_canvas_origin(frame)
    canvas_z      = float(np.mean([c.corners[i][2] for i in range(4)])) / 1000.0

    _draw_loop(
        asc_client, cmd_client, state_client,
        gcode_path, scale, tool_length,
        odom_T_origin, canvas_z,
        canvas_width_mm=c.width_mm,
        canvas_height_mm=c.height_mm,
    )
