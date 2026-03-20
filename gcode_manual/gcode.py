"""
gcode_manual/gcode.py
=====================
Gcode drawing for Spot — marker already in gripper, no Vicon needed.
Credentials are read from the .env file in the repo root.

Usage
-----
    python gcode_manual/gcode.py triangle.ngc
    python gcode_manual/gcode.py my_drawing.gcode
    python gcode_manual/gcode.py triangle.ngc --test-file-parsing

Drawing parameters (scale, speed, force, etc.) are in gcode.cfg.

How it works
------------
1. Spot stands up and extends the arm forward and down.
2. Closes the gripper firmly on the marker already taped there.
3. Pauses — press Enter once the marker looks to be above the paper.
4. Presses the marker down to physically find the surface (touch-to-find-ground).
5. Sets gcode (0,0) at that contact point and draws.

Positioning
-----------
Stand Spot so its front feet are ~50-70 cm from the paper's near edge,
facing straight at it.  The arm reaches ~75 cm forward, so gcode (0,0)
lands wherever the marker first touches the paper.
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
from bosdyn.api import (
    arm_surface_contact_pb2,
    arm_surface_contact_service_pb2,
    basic_command_pb2,
    geometry_pb2,
    trajectory_pb2,
)
from bosdyn.client.arm_surface_contact import ArmSurfaceContactClient
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import (
    BODY_FRAME_NAME,
    GRAV_ALIGNED_BODY_FRAME_NAME,
    GROUND_PLANE_FRAME_NAME,
    HAND_FRAME_NAME,
    ODOM_FRAME_NAME,
    VISION_FRAME_NAME,
    WR1_FRAME_NAME,
    get_a_tform_b,
    math_helpers,
)
from bosdyn.client.math_helpers import Quat, SE2Pose, SE3Pose, math
from bosdyn.client.robot_command import (
    RobotCommandBuilder,
    RobotCommandClient,
    block_for_trajectory_cmd,
    block_until_arm_arrives,
    blocking_stand,
)
from bosdyn.client.robot_state import RobotStateClient

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_HERE)

# Load .env from repo root
load_dotenv(os.path.join(_REPO_ROOT, ".env"))


def _require_env(name):
    val = os.environ.get(name)
    if not val:
        raise SystemExit(f"Missing required env var: {name}  (set it in .env)")
    return val


# ---------------------------------------------------------------------------
# Gcode helpers
# ---------------------------------------------------------------------------


def make_orthogonal(primary, secondary):
    p = primary / np.linalg.norm(primary)
    s = secondary / np.linalg.norm(secondary)
    u = s - (np.dot(p, s) / np.dot(s, s)) * p
    return u / np.linalg.norm(u)


def pick_gcode_file():
    """Show numbered menu of g_codes/ files and return the chosen path."""
    g_codes_dir = os.path.join(_HERE, "g_codes")
    exts = (".gcode", ".ngc", ".nc", ".tap")
    files = (
        sorted(
            os.path.join(g_codes_dir, f)
            for f in os.listdir(g_codes_dir)
            if f.lower().endswith(exts)
        )
        if os.path.isdir(g_codes_dir)
        else []
    )
    if not files:
        raise SystemExit("No gcode files found in g_codes/")
    print()
    print("Available gcode files:")
    for i, f in enumerate(files, 1):
        print(f"  {i}. {os.path.basename(f)}")
    while True:
        raw = input("Pick a file [1]: ").strip()
        if not raw:
            return files[0]
        try:
            idx = int(raw) - 1
            if 0 <= idx < len(files):
                return files[idx]
        except ValueError:
            pass
        print(f"  Enter a number between 1 and {len(files)}.")


class GcodeReader:
    def __init__(
        self,
        file_path,
        tool_length,
        scale,
        logger,
        below_z_is_admittance,
        travel_z,
        draw_on_wall,
        gcode_start_x=0,
        gcode_start_y=0,
        draw_z_offset=0.0,
        flip_goal_quat=False,
    ):
        self.file = open(file_path, "r")
        self.tool_length = tool_length
        self.scale = scale
        self.logger = logger
        self.below_z_is_admittance = below_z_is_admittance
        self.travel_z = travel_z
        self.draw_on_wall = draw_on_wall
        self.gcode_start_x = gcode_start_x
        self.gcode_start_y = gcode_start_y
        self.draw_z_offset = draw_z_offset
        self.flip_goal_quat = flip_goal_quat
        self.current_origin_T_goals = None
        self.last_x = self.last_y = self.last_z = 0
        self._last_line = ""

    def set_origin(self, vision_T_origin, vision_T_admittance_frame):
        if not self.draw_on_wall:
            zhat = [0.0, 0.0, 1.0]
            x1, x2, x3 = vision_T_origin.rot.transform_point(-1.0, 0.0, 0.0)
            xhat = make_orthogonal(zhat, [x1, x2, x3])
            yhat = np.cross(zhat, xhat)
            mat = np.array([xhat, yhat, zhat]).T
            self.vision_T_origin = SE3Pose(
                vision_T_origin.x,
                vision_T_origin.y,
                vision_T_origin.z,
                Quat.from_matrix(mat),
            )
        else:
            x1, x2, x3 = vision_T_admittance_frame.rot.transform_point(0, -1, 0)
            y1, y2, y3 = vision_T_admittance_frame.rot.transform_point(1, 0, 0)
            z1, z2, z3 = vision_T_admittance_frame.rot.transform_point(0, 0, 1)
            mat = np.array([[x1, y1, z1], [x2, y2, z2], [x3, y3, z3]]).T
            self.vision_T_origin = SE3Pose(
                vision_T_origin.x,
                vision_T_origin.y,
                vision_T_origin.z,
                Quat.from_matrix(mat),
            )

    def _goal_quat(self):
        if not self.draw_on_wall:
            xhat = np.array([0, 0, -1], dtype=float)
            # flip_goal_quat=True (draw_gcode): origin +X = body forward → gripper faces forward
            # flip_goal_quat=False (gcode_manual): origin +X = body backward → gripper faces forward
            zhat = np.array([1, 0, 0] if self.flip_goal_quat else [-1, 0, 0], dtype=float)
            yhat = np.cross(zhat, xhat)
            mat = np.array([xhat, yhat, zhat]).T
        else:
            mat = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float).T
        return Quat.from_matrix(mat)

    def convert_gcode_to_origin_T_goals(self, line):
        for ch in ("(", "%", ";"):
            idx = line.find(ch)
            if idx >= 0:
                line = line[:idx]
        parts = line.split()
        if not parts:
            return None

        if parts[0] in ("G00", "G0", "G01", "G1"):
            x, y, z = self.last_x, self.last_y, self.last_z
            for p in parts[1:]:
                if p[0] == "X":
                    x = (float(p[1:]) - self.gcode_start_x) * self.scale
                elif p[0] == "Y":
                    y = (float(p[1:]) - self.gcode_start_y) * self.scale
                elif p[0] == "Z":
                    z = float(p[1:]) * self.scale
            self.last_x, self.last_y, self.last_z = x, y, z
            return [SE3Pose(x, y, z, self._goal_quat())]

        elif parts[0] in ("G02", "G2", "G03", "G3"):
            x, y, z = self.last_x, self.last_y, self.last_z
            i_val = j_val = k_val = 0.0
            for p in parts[1:]:
                if p[0] == "X":
                    x = (float(p[1:]) - self.gcode_start_x) * self.scale
                elif p[0] == "Y":
                    y = (float(p[1:]) - self.gcode_start_y) * self.scale
                elif p[0] == "Z":
                    z = float(p[1:]) * self.scale
                elif p[0] == "I":
                    i_val = float(p[1:]) * self.scale
                elif p[0] == "J":
                    j_val = float(p[1:]) * self.scale
                elif p[0] == "K":
                    k_val = float(p[1:]) * self.scale
            clockwise = parts[0] in ("G02", "G2")
            if i_val != 0 or j_val != 0:
                lp = [self.last_x, self.last_y]
                ep = [x, y]
                cp = [self.last_x + i_val, self.last_y + j_val]

                def make_pt(a, b):
                    return SE3Pose(a, b, self.last_z, self._goal_quat())
            elif i_val != 0 or k_val != 0:
                lp = [self.last_x, self.last_z]
                ep = [x, z]
                cp = [self.last_x + i_val, self.last_z + k_val]

                def make_pt(a, b):
                    return SE3Pose(a, self.last_y, b, self._goal_quat())
            else:
                lp = [self.last_y, self.last_z]
                ep = [y, z]
                cp = [self.last_y + j_val, self.last_z + k_val]

                def make_pt(a, b):
                    return SE3Pose(self.last_x, a, b, self._goal_quat())

            lr = np.subtract(lp, cp)
            er = np.subtract(ep, cp)
            lt = math.atan2(lr[1], lr[0])
            et = math.atan2(er[1], er[0])
            if clockwise:
                if lt < et:
                    lt += 2 * math.pi
            else:
                if lt > et:
                    et += 2 * math.pi
            n = max(abs(int((et - lt) / 0.01)), 1)
            r = math.sqrt(lr[0] ** 2 + lr[1] ** 2)
            goals = []
            for i in range(n - 1):
                th = lt - i * 0.01 if clockwise else lt + i * 0.01
                goals.append(
                    make_pt(cp[0] + r * math.cos(th), cp[1] + r * math.sin(th))
                )
            goals.append(make_pt(ep[0], ep[1]))
            self.last_x, self.last_y, self.last_z = (
                goals[-1].x,
                goals[-1].y,
                goals[-1].z,
            )
            return goals
        else:
            return None

    def get_vision_T_goal(self, origin_T_goal, ground_plane_rt_vision):
        if not self.draw_on_wall:
            vision_T_goal = self.vision_T_origin * origin_T_goal
            if not self.is_admittance():
                vision_T_goal.z = self.travel_z + ground_plane_rt_vision[2]
            else:
                vision_T_goal.z = ground_plane_rt_vision[2] + self.draw_z_offset
        else:
            z_offset = 0 if self.is_admittance() else self.travel_z
            goal2 = SE3Pose(
                origin_T_goal.x,
                origin_T_goal.y,
                origin_T_goal.z + z_offset,
                origin_T_goal.rot,
            )
            vision_T_goal = self.vision_T_origin * goal2
        return vision_T_goal

    def is_admittance(self):
        return self.current_origin_T_goals[0].z < self.below_z_is_admittance

    def get_next_vision_T_goals(self, ground_plane_rt_vision, read_new_line=True):
        if not read_new_line:
            # Re-evaluate current line for travel moves (frame drift refresh).
            if not self._last_line:
                return (False, None, False)
            origin_T_goals = self.convert_gcode_to_origin_T_goals(self._last_line)
            if not origin_T_goals:
                return (False, None, False)
            self.current_origin_T_goals = origin_T_goals
            return (
                self.is_admittance(),
                [
                    self.get_vision_T_goal(g, ground_plane_rt_vision)
                    for g in origin_T_goals
                ],
                False,
            )

        # Read forward to find next motion command.
        origin_T_goals = None
        while origin_T_goals is None:
            self._last_line = self.file.readline()
            self.logger.info("Gcode: %s", self._last_line.strip())
            if not self._last_line:
                return (False, None, False)
            if self._last_line.strip() == "M0":
                return (False, None, True)
            origin_T_goals = self.convert_gcode_to_origin_T_goals(self._last_line)

        self.current_origin_T_goals = origin_T_goals
        vision_T_goals = [
            self.get_vision_T_goal(g, ground_plane_rt_vision) for g in origin_T_goals
        ]

        if not self.is_admittance():
            # Travel move: return single point, no batching.
            return (False, vision_T_goals, False)

        # Draw move: greedily batch ALL following admittance lines into one
        # trajectory so the arm executes the full stroke path in one smooth motion.
        while True:
            pos = self.file.tell()
            next_line = self.file.readline()
            if not next_line:
                break  # EOF
            if next_line.strip() == "M0":
                self.file.seek(pos)  # put M0 back for next call
                break
            next_goals = self.convert_gcode_to_origin_T_goals(next_line)
            if next_goals is None:
                continue  # comment / blank line
            self.current_origin_T_goals = next_goals
            if not self.is_admittance():
                self.file.seek(pos)  # put travel line back for next call
                break
            # Still a draw line — log it and append to batch.
            self.logger.info("Gcode: %s", next_line.strip())
            self._last_line = next_line
            vision_T_goals.extend(
                self.get_vision_T_goal(g, ground_plane_rt_vision) for g in next_goals
            )

        return (True, vision_T_goals, False)

    def test_file_parsing(self):
        for line in self.file:
            self.logger.debug("Line: %s", line.strip())
            self.convert_gcode_to_origin_T_goals(line)


# ---------------------------------------------------------------------------
# Arm helpers
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
            dt = (
                math.sqrt(
                    (last.x - goal.x) ** 2
                    + (last.y - goal.y) ** 2
                    + (last.z - goal.z) ** 2
                )
                / velocity
            )
            last_t += dt
            seconds = int(last_t)
            nanos = int((last_t - seconds) * 1e9)
        tg = task_T_vision * goal
        points.append(
            trajectory_pb2.SE3TrajectoryPoint(
                pose=geometry_pb2.SE3Pose(
                    position=geometry_pb2.Vec3(x=tg.x, y=tg.y, z=tg.z),
                    rotation=geometry_pb2.Quaternion(
                        w=tg.rot.w, x=tg.rot.x, y=tg.rot.y, z=tg.rot.z
                    ),
                ),
                time_since_reference=duration_pb2.Duration(
                    seconds=seconds, nanos=nanos
                ),
            )
        )
        last = goal
    return trajectory_pb2.SE3Trajectory(points=points)


def move_arm(
    robot_state,
    is_admittance,
    vision_T_goals,
    asc_client,
    cmd_client,
    velocity,
    allow_walking,
    vision_T_admittance,
    press_force_pct,
    api_send_frame,
    use_xy_cross,
    bias_force_x,
):
    if not is_admittance:
        # Travel move: use standard arm_pose_command — reliably lifts off the surface.
        goal = vision_T_goals[-1]
        p = goal.to_proto()
        arm_cmd = RobotCommandBuilder.arm_pose_command(
            p.position.x,
            p.position.y,
            p.position.z,
            p.rotation.w,
            p.rotation.x,
            p.rotation.y,
            p.rotation.z,
            VISION_FRAME_NAME,
            2.0,
        )
        cmd_client.robot_command(RobotCommandBuilder.build_synchro_command(arm_cmd))
        return

    # Draw move: use ArmSurfaceContact for force/admittance control.
    traj = move_along_trajectory(
        api_send_frame, velocity, vision_T_goals, vision_T_admittance
    )
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
    cmd.xy_admittance = (
        arm_surface_contact_pb2.ArmSurfaceContact.Request.ADMITTANCE_SETTING_OFF
    )
    cmd.z_admittance = (
        arm_surface_contact_pb2.ArmSurfaceContact.Request.ADMITTANCE_SETTING_LOOSE
    )
    cmd.xy_to_z_cross_term_admittance = (
        arm_surface_contact_pb2.ArmSurfaceContact.Request.ADMITTANCE_SETTING_VERY_STIFF
        if use_xy_cross
        else arm_surface_contact_pb2.ArmSurfaceContact.Request.ADMITTANCE_SETTING_OFF
    )
    cmd.bias_force_ewrt_body.CopyFrom(geometry_pb2.Vec3(x=bias_force_x, y=0, z=0))
    gripper = RobotCommandBuilder.claw_gripper_open_fraction_command(0)
    cmd.gripper_command.CopyFrom(
        gripper.synchronized_command.gripper_command.claw_gripper_command
    )
    cmd.is_robot_following_hand = allow_walking
    asc_client.arm_surface_contact_command(
        arm_surface_contact_service_pb2.ArmSurfaceContactCommand(request=cmd)
    )


def get_transforms(use_vision_frame, robot_state):
    world = VISION_FRAME_NAME if use_vision_frame else ODOM_FRAME_NAME
    snap = robot_state.kinematic_state.transforms_snapshot
    world_T_body = get_a_tform_b(snap, world, BODY_FRAME_NAME)
    world_T_flat_body = get_a_tform_b(snap, world, GRAV_ALIGNED_BODY_FRAME_NAME)
    body_T_hand = get_a_tform_b(snap, BODY_FRAME_NAME, HAND_FRAME_NAME)
    body_T_wrist = get_a_tform_b(snap, BODY_FRAME_NAME, WR1_FRAME_NAME)
    world_T_hand = world_T_body * body_T_hand
    odom_T_body = get_a_tform_b(snap, ODOM_FRAME_NAME, BODY_FRAME_NAME)
    return (
        world_T_body,
        world_T_flat_body,
        body_T_hand,
        world_T_hand,
        body_T_wrist,
        odom_T_body,
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def run(initial_gcode_file, test_only=False):
    cfg = configparser.ConfigParser()
    cfg.read(os.path.join(_HERE, "gcode.cfg"))
    g = cfg["General"]

    tool_length = g.getfloat("tool_length", 0.0)
    scale = g.getfloat("scale", 0.001)
    min_dist_to_goal = g.getfloat("min_dist_to_goal", 0.03)
    allow_walking = g.getboolean("allow_walking", False)
    velocity = g.getfloat("velocity", 0.15)
    press_force_pct = g.getfloat("press_force_percent", -0.005)
    below_z_adm = g.getfloat("below_z_is_admittance", 0.0)
    travel_z = g.getfloat("travel_z", 0.05)
    draw_z_offset = g.getfloat("draw_z_offset", 0.005)
    gcode_start_x = g.getfloat("gcode_start_x", 0)
    gcode_start_y = g.getfloat("gcode_start_y", 0)
    draw_on_wall = g.getboolean("draw_on_wall", False)
    use_vision_frame = g.getboolean("use_vision_frame", True)
    use_xy_cross = g.getboolean("use_xy_to_z_cross_term", False)
    bias_force_x = g.getfloat("bias_force_x", 0.0)

    api_send_frame = VISION_FRAME_NAME if use_vision_frame else ODOM_FRAME_NAME

    if test_only:
        bosdyn.client.util.setup_logging(verbose=False)
        sdk = bosdyn.client.create_standard_sdk("GcodeManualClient")
        robot = sdk.create_robot("0.0.0.0")  # dummy — not connecting
        gcode = GcodeReader(
            initial_gcode_file,
            tool_length,
            scale,
            robot.logger,
            below_z_adm,
            travel_z,
            draw_on_wall,
            gcode_start_x,
            gcode_start_y,
            draw_z_offset,
        )
        gcode.test_file_parsing()
        print("File parsing OK.")
        return

    robot_ip = _require_env("SPOT_IP")
    username = _require_env("SPOT_USER")
    password = _require_env("SPOT_PASSWORD")

    bosdyn.client.util.setup_logging(verbose=False)
    sdk = bosdyn.client.create_standard_sdk("GcodeManualClient")
    robot = sdk.create_robot(robot_ip)
    robot.authenticate(username, password)
    robot.time_sync.wait_for_sync()
    assert robot.has_arm(), "Robot requires an arm."

    asc_client = robot.ensure_client(ArmSurfaceContactClient.default_service_name)
    lease_client = robot.ensure_client(
        bosdyn.client.lease.LeaseClient.default_service_name
    )
    cmd_client = robot.ensure_client(RobotCommandClient.default_service_name)
    state_client = robot.ensure_client(RobotStateClient.default_service_name)
    estop_client = robot.ensure_client(EstopClient.default_service_name)

    estop_ep = EstopEndpoint(estop_client, name="gcode_manual_estop", estop_timeout=9.0)
    estop_ep.force_simple_setup()

    vision_T_admittance = geometry_pb2.SE3Pose(
        position=geometry_pb2.Vec3(x=0, y=0, z=0),
        rotation=geometry_pb2.Quaternion(w=1, x=0, y=0, z=0),
    )
    wr1_T_tool = SE3Pose(0.23589 + tool_length, 0, 0, Quat(w=1, x=0, y=0, z=0))

    def _draw_file(gcode_file):
        """Touch-to-find-ground, set origin, execute gcode. Returns vision_T_odom."""
        gcode = GcodeReader(
            gcode_file,
            tool_length,
            scale,
            robot.logger,
            below_z_adm,
            travel_z,
            draw_on_wall,
            gcode_start_x,
            gcode_start_y,
            draw_z_offset,
        )

        # ── Move arm to start position ────────────────────────────────────
        robot_state = state_client.get_robot_state()
        flat_body_T_hand = SE3Pose(
            0.75, 0, -0.35, math_helpers.Quat(w=0.707, x=0, y=0.707, z=0)
        )
        (_, odom_T_flat_body, _, _, _, _) = get_transforms(False, robot_state)
        odom_T_hand = odom_T_flat_body * flat_body_T_hand
        p = odom_T_hand.to_proto()

        robot.logger.info("Moving arm to starting position…")
        arm_cmd = RobotCommandBuilder.arm_pose_command(
            p.position.x,
            p.position.y,
            p.position.z,
            p.rotation.w,
            p.rotation.x,
            p.rotation.y,
            p.rotation.z,
            ODOM_FRAME_NAME,
            0.000001,
        )
        cmd_id = cmd_client.robot_command(
            RobotCommandBuilder.build_synchro_command(arm_cmd)
        )
        block_until_arm_arrives(cmd_client, cmd_id)

        # ── Confirm before touching down ──────────────────────────────────
        print()
        print("=" * 60)
        print(f" File: {os.path.basename(gcode_file)}")
        print(" Arm is in position. Press Enter to touch down")
        print(" and begin drawing, or Ctrl-C to abort.")
        print("=" * 60)
        input()

        # ── Touch-to-find-ground ──────────────────────────────────────────
        robot_state = state_client.get_robot_state()
        (vision_T_body, _, _, _, _, odom_T_body) = get_transforms(True, robot_state)

        vision_T_wr1 = get_a_tform_b(
            robot_state.kinematic_state.transforms_snapshot,
            VISION_FRAME_NAME,
            WR1_FRAME_NAME,
        )
        vision_T_tool = vision_T_wr1 * wr1_T_tool

        robot.logger.info("Pressing marker down to find surface…")
        move_arm(
            robot_state,
            True,
            [vision_T_tool],
            asc_client,
            cmd_client,
            0.04,
            allow_walking,
            vision_T_admittance,
            press_force_pct,
            api_send_frame,
            use_xy_cross,
            bias_force_x,
        )
        time.sleep(5.0)

        # ── Re-read state after touchdown ─────────────────────────────────
        robot_state = state_client.get_robot_state()
        (vision_T_body, _, body_T_hand, _, _, odom_T_body) = get_transforms(
            True, robot_state
        )

        odom_T_ground = get_a_tform_b(
            robot_state.kinematic_state.transforms_snapshot,
            ODOM_FRAME_NAME,
            GROUND_PLANE_FRAME_NAME,
        )
        vision_T_odom = vision_T_body * odom_T_body.inverse()
        gx, gy, gz = vision_T_odom.transform_point(
            odom_T_ground.x, odom_T_ground.y, odom_T_ground.z
        )
        ground_plane_rt_vision = [gx, gy, gz]

        vision_T_wr1 = get_a_tform_b(
            robot_state.kinematic_state.transforms_snapshot,
            VISION_FRAME_NAME,
            WR1_FRAME_NAME,
            validate=True,
        )
        vision_T_tool = vision_T_wr1 * wr1_T_tool

        # Use the actual touch z as the ground reference — more accurate than
        # GROUND_PLANE_FRAME when the marker is sitting on the paper.
        ground_plane_rt_vision[2] = vision_T_tool.z

        # ── Set gcode (0,0) at touch point ───────────────────────────────
        zhat = [0.0, 0.0, 1.0]
        x1, x2, x3 = vision_T_body.rot.transform_point(1.0, 0.0, 0.0)
        xhat = make_orthogonal(zhat, [x1, x2, x3])
        yhat = np.cross(zhat, xhat)
        mat = np.array([xhat, yhat, zhat]).T
        vision_T_origin = SE3Pose(
            vision_T_tool.x, vision_T_tool.y, vision_T_tool.z, Quat.from_matrix(mat)
        )

        gcode.set_origin(vision_T_origin, SE3Pose.from_proto(vision_T_admittance))
        robot.logger.info(
            "Origin set at (%.3f, %.3f, %.3f)",
            vision_T_origin.x,
            vision_T_origin.y,
            vision_T_origin.z,
        )

        # ── Lift arm to travel height before starting gcode loop ──────────
        touch_z = vision_T_tool.z
        tp = vision_T_tool.to_proto()
        robot.logger.info("Lifting arm to travel height (%.3f m)…", touch_z + travel_z)
        lift_cmd = RobotCommandBuilder.arm_pose_command(
            tp.position.x,
            tp.position.y,
            touch_z + travel_z,
            tp.rotation.w,
            tp.rotation.x,
            tp.rotation.y,
            tp.rotation.z,
            VISION_FRAME_NAME,
            2.0,
        )
        lift_id = cmd_client.robot_command(
            RobotCommandBuilder.build_synchro_command(lift_cmd)
        )
        block_until_arm_arrives(cmd_client, lift_id, timeout_sec=5.0)

        # ── Execute gcode ─────────────────────────────────────────────────
        (is_admittance, vision_T_goals, is_pause) = gcode.get_next_vision_T_goals(
            ground_plane_rt_vision
        )

        while is_pause:
            input("M0 pause — press Enter to continue…")
            (is_admittance, vision_T_goals, is_pause) = gcode.get_next_vision_T_goals(
                ground_plane_rt_vision
            )

        if vision_T_goals is None:
            robot.logger.info("Gcode file is empty.")
            return vision_T_odom

        move_arm(
            robot_state,
            is_admittance,
            vision_T_goals,
            asc_client,
            cmd_client,
            velocity,
            allow_walking,
            vision_T_admittance,
            press_force_pct,
            api_send_frame,
            use_xy_cross,
            bias_force_x,
        )
        odom_T_hand_goal = vision_T_odom.inverse() * vision_T_goals[-1]
        last_admittance = is_admittance

        while True:
            robot_state = state_client.get_robot_state()
            (vision_T_body, _, body_T_hand, _, _, odom_T_body) = get_transforms(
                True, robot_state
            )
            vision_T_odom = vision_T_body * odom_T_body.inverse()
            # Keep ground_plane_rt_vision[2] pinned to the actual touch z.
            ground_plane_rt_vision[0], ground_plane_rt_vision[1] = (
                vision_T_odom.transform_point(
                    odom_T_ground.x, odom_T_ground.y, odom_T_ground.z
                )[:2]
            )

            adm_inv = SE3Pose.from_proto(vision_T_admittance).inverse()
            hand_in_adm = adm_inv * vision_T_odom * odom_T_body * body_T_hand
            goal_in_adm = adm_inv * vision_T_odom * odom_T_hand_goal

            if is_admittance:
                dist = math.sqrt(
                    (hand_in_adm.x - goal_in_adm.x) ** 2
                    + (hand_in_adm.y - goal_in_adm.y) ** 2
                )
            else:
                dist = math.sqrt(
                    (hand_in_adm.x - goal_in_adm.x) ** 2
                    + (hand_in_adm.y - goal_in_adm.y) ** 2
                    + (hand_in_adm.z - goal_in_adm.z) ** 2
                )

            if dist < min_dist_to_goal:
                (is_admittance, vision_T_goals, is_pause) = (
                    gcode.get_next_vision_T_goals(ground_plane_rt_vision)
                )

                while is_pause:
                    input("M0 pause — press Enter to continue…")
                    (is_admittance, vision_T_goals, is_pause) = (
                        gcode.get_next_vision_T_goals(ground_plane_rt_vision)
                    )

                if vision_T_goals is None:
                    robot.logger.info("Gcode program finished.")
                    # Lift marker off paper before returning.
                    robot_state = state_client.get_robot_state()
                    snap = robot_state.kinematic_state.transforms_snapshot
                    vt_wr1 = get_a_tform_b(snap, VISION_FRAME_NAME, WR1_FRAME_NAME)
                    vt_tool = vt_wr1 * wr1_T_tool
                    ep = vt_tool.to_proto()
                    lift = RobotCommandBuilder.arm_pose_command(
                        ep.position.x,
                        ep.position.y,
                        touch_z + travel_z,
                        ep.rotation.w,
                        ep.rotation.x,
                        ep.rotation.y,
                        ep.rotation.z,
                        VISION_FRAME_NAME,
                        2.0,
                    )
                    lid = cmd_client.robot_command(
                        RobotCommandBuilder.build_synchro_command(lift)
                    )
                    block_until_arm_arrives(cmd_client, lid, timeout_sec=5.0)
                    break

                move_arm(
                    robot_state,
                    is_admittance,
                    vision_T_goals,
                    asc_client,
                    cmd_client,
                    velocity,
                    allow_walking,
                    vision_T_admittance,
                    press_force_pct,
                    api_send_frame,
                    use_xy_cross,
                    bias_force_x,
                )
                odom_T_hand_goal = vision_T_odom.inverse() * vision_T_goals[-1]

                if is_admittance != last_admittance:
                    time.sleep(3.0 if is_admittance else 1.0)
                last_admittance = is_admittance
            elif not is_admittance:
                (is_admittance, vision_T_goals, is_pause) = (
                    gcode.get_next_vision_T_goals(
                        ground_plane_rt_vision, read_new_line=False
                    )
                )

            time.sleep(0.05)

        return vision_T_odom

    # ── Robot startup ─────────────────────────────────────────────────────
    with (
        EstopKeepAlive(estop_ep),
        bosdyn.client.lease.LeaseKeepAlive(
            lease_client, must_acquire=True, return_at_exit=True
        ),
    ):
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), "Power on failed."
        blocking_stand(cmd_client, timeout_sec=10)
        robot.logger.info("Robot standing.")

        # ── Close gripper on taped marker ─────────────────────────────────
        robot.logger.info("Closing gripper on marker.")
        gripper_cmd = RobotCommandBuilder.claw_gripper_close_command()
        gripper_cmd.synchronized_command.gripper_command.claw_gripper_command.maximum_torque.value = 15.0
        cmd_client.robot_command(gripper_cmd)
        time.sleep(1.0)

        # ── Draw loop ─────────────────────────────────────────────────────
        current_file = initial_gcode_file
        while True:
            _draw_file(current_file)
            blocking_stand(cmd_client, timeout_sec=10)

            print()
            print("┌─────────────────────────────────┐")
            print("│  Drawing complete. What next?   │")
            print("│  1  Stow arm and sit            │")
            print("│  2  Draw another file           │")
            print("└─────────────────────────────────┘")
            choice = input("> ").strip()

            if choice == "2":
                current_file = pick_gcode_file()
            else:
                break

        # ── Clean exit ────────────────────────────────────────────────────
        robot.logger.info("Stowing arm and sitting…")
        cmd_client.robot_command(RobotCommandBuilder.arm_stow_command())
        time.sleep(2)
        cmd_client.robot_command(RobotCommandBuilder.synchro_sit_command())
        time.sleep(2)
        robot.power_off(cut_immediately=False, timeout_sec=20)
        robot.logger.info("Done.")


def main():
    parser = argparse.ArgumentParser(
        description="Draw a gcode file on the floor — marker pre-taped to gripper, no Vicon needed"
    )
    parser.add_argument(
        "gcode",
        nargs="?",
        default=None,
        help="Path to .gcode or .ngc file. Omit to pick from g_codes/.",
    )
    parser.add_argument(
        "--test-file-parsing",
        action="store_true",
        help="Parse gcode without connecting to robot",
    )
    args = parser.parse_args()

    if args.gcode is None:
        args.gcode = pick_gcode_file()
    elif not os.path.exists(args.gcode):
        raise SystemExit(f"File not found: {args.gcode}")

    try:
        run(args.gcode, test_only=args.test_file_parsing)
    except Exception:
        bosdyn.client.util.get_logger().exception("Fatal error")
        sys.exit(1)


if __name__ == "__main__":
    main()
