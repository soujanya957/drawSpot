"""
draw_gcode/main.py — Autonomous walk + marker grab + gcode drawing
====================================================================
Pipeline
--------
  1. Connect Vicon — verify canvas is visible
  2. Connect Spot  — authenticate + time-sync
  3. Stand up + start key listener
  4. Walk to canvas (auto-nav via Vicon)
  5. Open for marker (extend arm, wait ENTER, close gripper)
  --- Drawing loop ---
  6. Select gcode file
  7. Touch-to-find-ground + draw (same mechanism as gcode_manual/gcode.py)
  8. Stow arm, stand
  9. "Draw another?" → if yes, back to 6
     "Change marker?" → if yes, back to 5 then 6
  ---
  10. Sit and exit

Key bindings (during robot operation)
--------------------------------------
  SPACE / ESC  — emergency stop (stow + sit)
  ENTER        — confirm (marker placement, arm position, M0 pauses)
  x / X        — sit and exit when prompted

Usage
-----
    python -m draw_gcode.main --vicon 169.254.217.218:801
"""

import configparser
import math as _math
import os
import sys
import time

import numpy as np

import bosdyn.client
import bosdyn.client.util
from bosdyn.api import geometry_pb2
from bosdyn.client.arm_surface_contact import ArmSurfaceContactClient
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import (
    ODOM_FRAME_NAME,
    VISION_FRAME_NAME,
    WR1_FRAME_NAME,
    GROUND_PLANE_FRAME_NAME,
    get_a_tform_b,
)
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.robot_command import (
    RobotCommandBuilder,
    RobotCommandClient,
    blocking_stand,
    block_until_arm_arrives,
)
from bosdyn.client.robot_state import RobotStateClient

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config.robot_config import PASSWORD, ROBOT_IP, USERNAME, VICON_ADDRESS  # noqa: E402
from vicon.client import ViconClient  # noqa: E402

import draw_gcode._ctrl as _ctrl  # noqa: E402
from draw_gcode.robot.navigator import walk_to_canvas, open_for_marker  # noqa: E402

# Drawing primitives — same implementation as the proven gcode_manual approach
from gcode_manual.gcode import GcodeReader, make_orthogonal, move_arm, get_transforms  # noqa: E402

VICON_CONNECT_TO = 6.0
G_CODES_DIR = os.path.join(os.path.dirname(__file__), "g_codes")
_CFG_PATH = os.path.join(os.path.dirname(__file__), "gcode.cfg")

_ARM_REACH_IDEAL_MM = 600.0  # desired distance from body to canvas centre (mm)
_ARM_REACH_TOL_MM = 50.0  # acceptable error before walking is triggered (mm)


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------


def _load_cfg() -> dict:
    cfg = configparser.ConfigParser()
    cfg.read(_CFG_PATH)
    g = cfg["General"]
    return dict(
        tool_length=g.getfloat("tool_length", 0.0),
        allow_walking=g.getboolean("allow_walking", False),
        velocity=g.getfloat("velocity", 0.15),
        press_force_pct=g.getfloat("press_force_percent", -0.002),
        below_z_adm=g.getfloat("below_z_is_admittance", 0.0),
        travel_z=g.getfloat("travel_z", 0.05),
        draw_z_offset=g.getfloat("draw_z_offset", 0.005),
        min_dist_to_goal=g.getfloat("min_dist_to_goal", 0.03),
        gcode_start_x=g.getfloat("gcode_start_x", 0),
        gcode_start_y=g.getfloat("gcode_start_y", 0),
        draw_on_wall=g.getboolean("draw_on_wall", False),
        use_vision_frame=g.getboolean("use_vision_frame", True),
        use_xy_cross=g.getboolean("use_xy_to_z_cross_term", False),
        bias_force_x=g.getfloat("bias_force_x", 0.0),
        touch_z_offset=g.getfloat("touch_z_offset_mm", 0.0) / 1000.0,
        canvas_margin=g.getfloat("canvas_margin", 0.80),
        arm_draw_x=g.getfloat("arm_draw_x_mm", 300.0) / 1000.0,
        arm_draw_y=g.getfloat("arm_draw_y_mm", 440.0) / 1000.0,
    )


# ---------------------------------------------------------------------------
# Gcode selection  (uses input() — call with key listener stopped)
# ---------------------------------------------------------------------------


def select_gcode() -> str:
    exts = (".gcode", ".nc", ".ngc", ".tap")
    files = []
    if os.path.isdir(G_CODES_DIR):
        files = sorted(
            os.path.join(G_CODES_DIR, n)
            for n in os.listdir(G_CODES_DIR)
            if n.lower().endswith(exts)
        )
    if not files:
        raise SystemExit(
            f"\nNo gcode files found in {G_CODES_DIR}/\n"
            f"Add .gcode / .ngc files there and re-run.\n"
        )

    print(f"\nGcode files in {os.path.relpath(G_CODES_DIR)}/")
    print("─" * 50)
    for i, f in enumerate(files, 1):
        kb = os.path.getsize(f) / 1024
        print(f"  [{i}]  {os.path.basename(f)}  ({kb:.1f} KB)")
    print("─" * 50)

    while True:
        raw = input(f"Select [1–{len(files)}]: ").strip()
        if not raw:
            chosen = files[0]
        else:
            try:
                idx = int(raw) - 1
                if 0 <= idx < len(files):
                    chosen = files[idx]
                else:
                    print(f"  Enter a number between 1 and {len(files)}.")
                    continue
            except ValueError:
                print(f"  Enter a number between 1 and {len(files)}.")
                continue
        print(f"  → {os.path.basename(chosen)}\n")
        return chosen


# ---------------------------------------------------------------------------
# Auto-scale: fit gcode within canvas
# ---------------------------------------------------------------------------


def _scan_bounds(gcode_path: str):
    """Return (x_min, x_max, y_min, y_max) at scale=1.0."""
    x_vals, y_vals = [], []
    with open(gcode_path) as fh:
        for line in fh:
            for ch in ("(", ";", "%"):
                if ch in line:
                    line = line[: line.find(ch)]
            parts = line.split()
            if not parts or parts[0] not in ("G0", "G00", "G1", "G01"):
                continue
            for p in parts[1:]:
                if p.startswith("X"):
                    try:
                        x_vals.append(float(p[1:]))
                    except ValueError:
                        pass
                elif p.startswith("Y"):
                    try:
                        y_vals.append(float(p[1:]))
                    except ValueError:
                        pass
    if not x_vals or not y_vals:
        return 0.0, 1.0, 0.0, 1.0
    return min(x_vals), max(x_vals), min(y_vals), max(y_vals)


def compute_scale(
    gcode_path: str,
    canvas,
    margin: float = 0.90,
    arm_draw_x_m: float = 0.30,
    arm_draw_y_m: float = 0.44,
) -> float:
    """Return scale (m/unit) that fits gcode within the canvas margin AND arm workspace limits."""
    x_min, x_max, y_min, y_max = _scan_bounds(gcode_path)
    gw = x_max - x_min
    gh = y_max - y_min
    if gw <= 0 or gh <= 0:
        print("  [WARN] Cannot determine gcode extents — using 0.001 m/unit")
        return 0.001
    cw = min((canvas.width_mm / 1000.0) * margin, arm_draw_x_m)
    ch = min((canvas.height_mm / 1000.0) * margin, arm_draw_y_m)
    scale = min(cw / gw, ch / gh)
    cx = (x_min + x_max) / 2.0
    cy = (y_min + y_max) / 2.0
    print(
        f"  Gcode extents  {gw:.1f} × {gh:.1f} raw units  (center {cx:.1f}, {cy:.1f})"
    )
    print(
        f"  Canvas         {canvas.width_mm:.0f} × {canvas.height_mm:.0f} mm"
        f"  (margin {margin * 100:.0f}%)"
        f"  arm limit {arm_draw_x_m * 1000:.0f} × {arm_draw_y_m * 1000:.0f} mm"
    )
    print(
        f"  Auto-scale     {scale:.6f} m/unit"
        f"  →  drawing {gw * scale * 1000:.0f} × {gh * scale * 1000:.0f} mm  (centered)"
    )
    return scale


def _rotation_matrix(q) -> np.ndarray:
    """Build 3×3 rotation matrix from (qx, qy, qz, qw)."""
    qx, qy, qz, qw = q
    return np.array(
        [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx**2 + qy**2)],
        ],
        dtype=float,
    )


# ---------------------------------------------------------------------------
# Pre-draw body positioning: walk until canvas centre is at ideal arm distance
# ---------------------------------------------------------------------------


def _walk_arm_to_canvas_center(vicon_client, cmd_client) -> bool:
    """
    Walk the robot body (forward/lateral, no rotation) so the canvas centre
    sits at _ARM_REACH_IDEAL_MM directly in front of the arm.
    Once within tolerance the base is stopped and drawing proceeds locked.
    Returns False only if estop fires.
    """
    _SPEED = 0.20  # m/s — slow and controlled
    _CMD_DUR = 0.4  # s per velocity burst

    print("  Positioning body so arm reaches canvas centre…", flush=True)

    for _ in range(120):  # max ~12 s
        if not _ctrl.check():
            return False

        frame = vicon_client.latest_frame
        if (
            frame is None
            or frame.canvas is None
            or not frame.canvas.is_valid()
            or frame.spot_body is None
        ):
            time.sleep(0.1)
            continue

        center_mm = np.mean(np.array(frame.canvas.corners), axis=0)
        robot_mm = np.array(frame.spot_body.position)
        delta_mm = center_mm - robot_mm
        R = _rotation_matrix(frame.spot_body.rotation_quat)  # world←body
        delta_body = R.T @ delta_mm  # body frame mm

        fwd_err = float(delta_body[0]) - _ARM_REACH_IDEAL_MM  # +ve → too far, step fwd
        lat_err = float(delta_body[1])  # +ve → centre is left

        if abs(fwd_err) < _ARM_REACH_TOL_MM and abs(lat_err) < _ARM_REACH_TOL_MM / 2:
            cmd_client.robot_command(
                RobotCommandBuilder.synchro_velocity_command(0, 0, 0),
                end_time_secs=time.time() + 0.3,
            )
            print(
                f"  Body locked: canvas centre at "
                f"({float(delta_body[0]):.0f}, {float(delta_body[1]):.0f}) mm body",
                flush=True,
            )
            return True

        # Normalise and scale velocity in body frame (v_x=fwd, v_y=left)
        mag = _math.sqrt(fwd_err**2 + lat_err**2)
        v_x = float(np.clip(fwd_err / mag * _SPEED, -_SPEED, _SPEED))
        v_y = float(np.clip(lat_err / mag * _SPEED, -_SPEED, _SPEED))

        cmd_client.robot_command(
            RobotCommandBuilder.synchro_velocity_command(v_x, v_y, 0),
            end_time_secs=time.time() + _CMD_DUR,
        )
        time.sleep(0.1)

    print(
        "  [WARN] Body positioning timed out — drawing from current position",
        flush=True,
    )
    return _ctrl.check()


# ---------------------------------------------------------------------------
# _draw_file — touch-to-find-ground + execute gcode
# Same mechanism as gcode_manual/gcode.py, adapted for key-listener controls.
# Key listener must be running (SPACE=estop, ENTER=confirm).
# Returns True on success, False if estop fired.
# ---------------------------------------------------------------------------


def _draw_file(
    gcode_file: str,
    scale: float,
    vicon_client,
    state_client,
    cmd_client,
    asc_client,
    robot_logger,
    cfg: dict,
) -> bool:
    tool_length = cfg["tool_length"]
    allow_walking = cfg["allow_walking"]
    velocity = cfg["velocity"]
    press_force_pct = cfg["press_force_pct"]
    below_z_adm = cfg["below_z_adm"]
    travel_z = cfg["travel_z"]
    draw_z_offset = cfg["draw_z_offset"]
    min_dist_to_goal = cfg["min_dist_to_goal"]
    draw_on_wall = cfg["draw_on_wall"]
    use_vision_frame = cfg["use_vision_frame"]
    use_xy_cross = cfg["use_xy_cross"]
    bias_force_x = cfg["bias_force_x"]
    api_send_frame = VISION_FRAME_NAME if use_vision_frame else ODOM_FRAME_NAME

    # Proto identity transform used by ArmSurfaceContact
    vision_T_admittance = geometry_pb2.SE3Pose(
        position=geometry_pb2.Vec3(x=0, y=0, z=0),
        rotation=geometry_pb2.Quaternion(w=1, x=0, y=0, z=0),
    )

    wr1_T_tool = SE3Pose(0.23589 + tool_length, 0, 0, Quat(w=1, x=0, y=0, z=0))

    # ── Corner-anchor: gcode (x_min, y_min) → canvas TL corner ──────────
    # Touch-down is at canvas centre. The canvas TL corner is at
    # (-width/2, -height/2) from centre in the gcode frame, so we set
    # gcode_start = x_min + canvas_half_m / scale.  This guarantees all
    # +X/+Y gcode coordinates stay within the canvas area.
    x_min, x_max, y_min, y_max = _scan_bounds(gcode_file)
    _canvas_vf = vicon_client.latest_frame
    if _canvas_vf and _canvas_vf.canvas and _canvas_vf.canvas.is_valid():
        gcode_start_x = x_min + (_canvas_vf.canvas.width_mm  / 1000.0) / (2.0 * scale)
        gcode_start_y = y_min + (_canvas_vf.canvas.height_mm / 1000.0) / (2.0 * scale)
        robot_logger.info(
            "Corner-anchor: gcode(%.1f, %.1f) → canvas TL corner; "
            "gcode_start=(%.2f, %.2f)",
            x_min, y_min, gcode_start_x, gcode_start_y)
    else:
        gcode_start_x = (x_min + x_max) / 2.0
        gcode_start_y = (y_min + y_max) / 2.0
        robot_logger.warning(
            "Canvas dims unavailable for corner-anchor — using centre-anchor fallback")

    gcode = GcodeReader(
        gcode_file,
        tool_length,
        scale,
        robot_logger,
        below_z_adm,
        travel_z,
        draw_on_wall,
        gcode_start_x,
        gcode_start_y,
        draw_z_offset,
        flip_goal_quat=True,  # draw_gcode origin: +X=body-forward; gcode_manual uses -X
    )

    # ── Walk body so canvas centre is at ideal arm distance, then lock base ─
    if not _walk_arm_to_canvas_center(vicon_client, cmd_client):
        return False

    # ── Move arm above canvas centre ─────────────────────────────────────────
    # Find the canvas centre in the SDK vision frame by rotating the
    # Vicon-measured body→canvas delta through the SDK body orientation.
    # This correctly bridges the two coordinate systems without assuming their
    # world-frame origins are aligned.
    #
    # Z: -0.25 m below flat_body keeps the marker tip ~5 cm above the floor
    # so touch-to-find-ground can press down to the surface (not already on it).
    robot_state = state_client.get_robot_state()
    (vision_T_body, vision_T_flat_body, _, _, _, _) = get_transforms(True, robot_state)
    hand_quat = Quat(w=0.707, x=0, y=0.707, z=0)
    # Fallback hover: keeps marker tip ~12 cm above floor when Vicon unavailable.
    # (body height ~0.53 m above floor; hand tip = arm_z - 0.23589 - tool_length;
    #  0.15 below flat_body gives tip ≈ 0.14 m above floor for tool_length=0)
    arm_z = vision_T_flat_body.z - 0.15

    # Default XY: ideal reach forward, no lateral offset
    _dflt_x, _dflt_y, _ = vision_T_flat_body.rot.transform_point(
        _ARM_REACH_IDEAL_MM / 1000.0, 0.0, 0.0)
    arm_x = vision_T_flat_body.x + _dflt_x
    arm_y = vision_T_flat_body.y + _dflt_y

    _vf = vicon_client.latest_frame
    if (_vf and _vf.canvas and _vf.canvas.is_valid()
            and _vf.spot_body and not _vf.spot_body.occluded):
        # body→canvas delta in Vicon world frame, rotated to body frame
        _ctr  = np.mean(np.array(_vf.canvas.corners), axis=0)
        _delt = _ctr - np.array(_vf.spot_body.position)              # mm, Vicon world
        _Rv   = _rotation_matrix(_vf.spot_body.rotation_quat)        # Vicon world ← body
        _db   = _Rv.T @ _delt / 1000.0                               # metres, body frame
        _fwd  = float(np.clip(_db[0], 0.40, 0.75))
        _lat  = float(np.clip(_db[1], -0.30, 0.30))
        # Rotate clamped body-frame offset into SDK vision frame for XY
        cx, cy, _ = vision_T_body.rot.transform_point(_fwd, _lat, 0.0)
        arm_x = vision_T_body.x + cx
        arm_y = vision_T_body.y + cy
        # Canvas Z in SDK vision frame: rotate body→canvas Z delta into vision frame.
        # Hand must be 12 cm above canvas tip + tool tip offset so touch-to-find-ground
        # can press down to the surface (marker hovers above, does NOT slam down).
        _, _, _cz = vision_T_body.rot.transform_point(0.0, 0.0, float(_db[2]))
        _canvas_z_vision = vision_T_body.z + _cz
        arm_z = _canvas_z_vision + 0.12 + 0.23589 + tool_length
        robot_logger.info(
            "Canvas centre → arm target: vision (%.3f, %.3f, %.3f)  "
            "canvas_z=%.3f  body fwd=%.0f mm  lat=%.0f mm",
            arm_x, arm_y, arm_z, _canvas_z_vision, _fwd * 1000, _lat * 1000)
    else:
        robot_logger.warning(
            "Vicon canvas/body not visible — using hardcoded %.0f mm forward, "
            "fallback hover arm_z=%.3f",
            _ARM_REACH_IDEAL_MM, arm_z)

    # ── Two-phase arm move: safe height first, then lower to hover ───────
    # Phase 1 — move arm to hover XY at flat-body level.  The arm travels
    # laterally and forward at a height that is guaranteed above the floor
    # (~48 cm) regardless of canvas-Z errors.  This prevents any downward
    # slam during the horizontal transit.
    safe_z = vision_T_flat_body.z - 0.05   # ~5 cm below flat body ≈ 48 cm
    robot_logger.info(
        "Phase 1 — arm to canvas XY at safe height z=%.3f…", safe_z)
    safe_cmd = RobotCommandBuilder.arm_pose_command(
        arm_x, arm_y, safe_z,
        hand_quat.w, hand_quat.x, hand_quat.y, hand_quat.z,
        VISION_FRAME_NAME,
        3.0,
    )
    cmd_client.robot_command(RobotCommandBuilder.build_synchro_command(safe_cmd))
    if not _ctrl.sleep(3.5):
        return False

    # Phase 2 — lower straight down to hover height (12 cm above canvas).
    # Only Z changes here, so there is no risk of sweeping across the floor.
    robot_logger.info(
        "Phase 2 — lowering to hover z=%.3f (12 cm above canvas)…", arm_z)
    arm_cmd = RobotCommandBuilder.arm_pose_command(
        arm_x, arm_y, arm_z,
        hand_quat.w, hand_quat.x, hand_quat.y, hand_quat.z,
        VISION_FRAME_NAME,
        2.0,
    )
    cmd_client.robot_command(RobotCommandBuilder.build_synchro_command(arm_cmd))
    if not _ctrl.sleep(2.5):
        return False

    # ── Confirm before touching down ──────────────────────────────────────
    if not _ctrl.wait_confirm(
        f"File: {os.path.basename(gcode_file)}\n"
        "  Arm is in position. Press ENTER to touch down and begin drawing."
    ):
        return False

    # ── Touch-to-find-ground ──────────────────────────────────────────────
    robot_state = state_client.get_robot_state()
    snap = robot_state.kinematic_state.transforms_snapshot

    vision_T_wr1 = get_a_tform_b(snap, VISION_FRAME_NAME, WR1_FRAME_NAME)
    vision_T_tool = vision_T_wr1 * wr1_T_tool

    robot_logger.info("Pressing marker down to find surface…")
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
    if not _ctrl.sleep(8.0):  # wait for surface contact; checks estop every 50 ms
        _emergency_lift_arm(state_client, cmd_client, wr1_T_tool)
        return False

    # ── Re-read state after touchdown ─────────────────────────────────────
    robot_state = state_client.get_robot_state()
    (vision_T_body, _, body_T_hand, _, _, odom_T_body) = get_transforms(
        True, robot_state
    )
    snap = robot_state.kinematic_state.transforms_snapshot

    odom_T_ground = get_a_tform_b(snap, ODOM_FRAME_NAME, GROUND_PLANE_FRAME_NAME)
    vision_T_odom = vision_T_body * odom_T_body.inverse()
    gx, gy, gz = vision_T_odom.transform_point(
        odom_T_ground.x, odom_T_ground.y, odom_T_ground.z
    )
    ground_plane_rt_vision = [gx, gy, gz]

    vision_T_wr1 = get_a_tform_b(snap, VISION_FRAME_NAME, WR1_FRAME_NAME, validate=True)
    vision_T_tool = vision_T_wr1 * wr1_T_tool
    ground_plane_rt_vision[2] = vision_T_tool.z + cfg["touch_z_offset"]  # pin Z to touch point ± offset
    robot_logger.info(
        "Touch Z: %.4f m  touch_z_offset: %+.1f mm  → drawing Z ref: %.4f m",
        vision_T_tool.z, cfg["touch_z_offset"] * 1000.0, ground_plane_rt_vision[2],
    )

    # ── Set gcode origin at the ACTUAL TOUCH POINT ────────────────────────
    #
    # Position = vision_T_tool (where the arm physically touched the canvas).
    # After _walk_arm_to_canvas_center + arm positioning, this is at canvas centre.
    # Using the real touch point avoids all Vicon↔SDK coordinate-frame mismatch.
    #
    # Orientation: walk_to_canvas aligns the robot to face +canvas_x_axis, so
    # body-forward == canvas x_axis after navigation.  We project body-forward
    # onto the horizontal plane and enforce z=[0,0,1] so the arm points straight
    # down throughout (no tilt) — identical to gcode_manual/gcode.py set_origin().
    #
    # gcode +X  = body forward  = canvas TL→TR  (width direction)
    # gcode +Y  = body left     = canvas TL→BL  (height direction)
    # gcode(0,0)= canvas centre (gcode_start_x/y = bounding-box centre above)
    zhat = np.array([0.0, 0.0, 1.0])
    bfwd = list(vision_T_body.rot.transform_point(1.0, 0.0, 0.0))
    xhat = make_orthogonal(zhat, bfwd)  # body forward projected to horizontal
    yhat = np.cross(zhat, xhat)  # body left, horizontal
    rot_mat = np.column_stack([xhat, yhat, zhat])

    gcode.vision_T_origin = SE3Pose(
        vision_T_tool.x, vision_T_tool.y, vision_T_tool.z, Quat.from_matrix(rot_mat)
    )
    robot_logger.info(
        "Origin at touch point (%.3f, %.3f, %.3f)  body-fwd=(%.3f,%.3f)",
        vision_T_tool.x,
        vision_T_tool.y,
        vision_T_tool.z,
        xhat[0],
        xhat[1],
    )

    # ── Lift arm to travel height before starting gcode loop ──────────────
    touch_z = vision_T_tool.z
    tp = vision_T_tool.to_proto()
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
    cmd_client.robot_command(RobotCommandBuilder.build_synchro_command(lift_cmd))
    if not _ctrl.sleep(2.5):
        return False

    # ── Execute gcode loop ────────────────────────────────────────────────
    (is_admittance, vision_T_goals, is_pause) = gcode.get_next_vision_T_goals(
        ground_plane_rt_vision
    )

    while is_pause:
        if not _ctrl.wait_confirm("M0 pause — press ENTER to continue."):
            return False
        (is_admittance, vision_T_goals, is_pause) = gcode.get_next_vision_T_goals(
            ground_plane_rt_vision
        )

    if vision_T_goals is None:
        robot_logger.info("Gcode file is empty.")
        return True

    if not _ctrl.check():
        return False

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
        if not _ctrl.check():
            _emergency_lift_arm(state_client, cmd_client, wr1_T_tool)
            return False

        robot_state = state_client.get_robot_state()
        (vision_T_body, _, body_T_hand, _, _, odom_T_body) = get_transforms(
            True, robot_state
        )
        vision_T_odom = vision_T_body * odom_T_body.inverse()
        ground_plane_rt_vision[0], ground_plane_rt_vision[1] = (
            vision_T_odom.transform_point(
                odom_T_ground.x, odom_T_ground.y, odom_T_ground.z
            )[:2]
        )

        adm_inv = SE3Pose.from_proto(vision_T_admittance).inverse()
        hand_in_adm = adm_inv * vision_T_odom * odom_T_body * body_T_hand
        goal_in_adm = adm_inv * vision_T_odom * odom_T_hand_goal

        if is_admittance:
            dist = _math.sqrt(
                (hand_in_adm.x - goal_in_adm.x) ** 2
                + (hand_in_adm.y - goal_in_adm.y) ** 2
            )
        else:
            dist = _math.sqrt(
                (hand_in_adm.x - goal_in_adm.x) ** 2
                + (hand_in_adm.y - goal_in_adm.y) ** 2
                + (hand_in_adm.z - goal_in_adm.z) ** 2
            )

        if dist < min_dist_to_goal:
            (is_admittance, vision_T_goals, is_pause) = gcode.get_next_vision_T_goals(
                ground_plane_rt_vision
            )

            while is_pause:
                if not _ctrl.wait_confirm("M0 pause — press ENTER to continue."):
                    _emergency_lift_arm(state_client, cmd_client, wr1_T_tool)
                    return False
                (is_admittance, vision_T_goals, is_pause) = (
                    gcode.get_next_vision_T_goals(ground_plane_rt_vision)
                )

            if vision_T_goals is None:
                robot_logger.info("Gcode program finished.")

                # 1. Exit force/admittance control and lift arm well above the
                #    floor.  travel_z (5 cm) is nowhere near enough clearance
                #    for stow — target flat-body height instead (~43 cm).
                robot_state = state_client.get_robot_state()
                (_, vision_T_flat_body_now, _, _, _, _) = get_transforms(
                    True, robot_state
                )
                # Move to centred body-frame position before stow.
                # Lifting straight up from the last draw position leaves the
                # arm at whatever lateral extreme it finished at, which can
                # put it in an under-body configuration that blocks stow.
                _hand_q = Quat(w=0.707, x=0, y=0.707, z=0)
                _ax, _ay, _az = vision_T_flat_body_now.transform_point(0.65, 0.0, 0.15)
                lift = RobotCommandBuilder.arm_pose_command(
                    _ax, _ay, _az,
                    _hand_q.w, _hand_q.x, _hand_q.y, _hand_q.z,
                    VISION_FRAME_NAME,
                    4.0,
                )
                robot_logger.info("Moving arm to centred pre-stow position…")
                try:
                    lift_id = cmd_client.robot_command(
                        RobotCommandBuilder.build_synchro_command(lift)
                    )
                    block_until_arm_arrives(cmd_client, lift_id, timeout_sec=5)
                except Exception:
                    time.sleep(4)  # fallback if feedback unavailable

                # 2. Stow arm.  If it fails, ask the operator to manually clear
                #    the arm (physically or via tablet teleop), then retry once.
                robot_logger.info("Stowing arm…")
                stow_ok = False
                for _attempt in range(2):
                    try:
                        cmd_client.robot_command(
                            RobotCommandBuilder.arm_stow_command()
                        )
                        time.sleep(6)  # stow feedback type incompatible with block_until_arm_arrives
                        stow_ok = True
                        break
                    except Exception as exc:
                        robot_logger.warning(
                            "Stow attempt %d failed: %s", _attempt + 1, exc)
                        time.sleep(1.0)

                if not stow_ok:
                    print(
                        "\n[WARN] Arm did not stow automatically.\n"
                        "  Manually raise the arm to a safe position\n"
                        "  (physically or via tablet teleop), then press ENTER."
                    )
                    _ctrl.wait_confirm("")
                    try:
                        cmd_id = cmd_client.robot_command(
                            RobotCommandBuilder.arm_stow_command()
                        )
                        block_until_arm_arrives(
                            cmd_client, cmd_id, timeout_sec=10)
                    except Exception:
                        pass

                return True

            if not _ctrl.check():
                _emergency_lift_arm(state_client, cmd_client, wr1_T_tool)
                return False

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
                _ctrl.sleep(3.0 if is_admittance else 1.0)
            last_admittance = is_admittance

        elif not is_admittance:
            (is_admittance, vision_T_goals, is_pause) = gcode.get_next_vision_T_goals(
                ground_plane_rt_vision, read_new_line=False
            )

        time.sleep(0.05)


# ---------------------------------------------------------------------------
# Arm stow helper
# ---------------------------------------------------------------------------


def _stow_arm_blocking(cmd_client) -> None:
    """
    Send arm_stow and wait long enough for the arm to reach the stow position.
    Uses a plain sleep rather than block_until_arm_arrives because the stow
    command's feedback type is not compatible with that helper and causes it to
    throw immediately, which means the 6 s fallback sleep starts late.
    Robot body is NOT commanded here; call blocking_stand / sit separately.
    """
    print("  Stowing arm…", flush=True)
    try:
        cmd_client.robot_command(RobotCommandBuilder.arm_stow_command())
    except Exception:
        pass
    time.sleep(6)  # stow takes up to ~4 s from any reasonable position
    print("  Arm stowed.", flush=True)


def _emergency_lift_arm(state_client, cmd_client, wr1_T_tool) -> None:
    """
    Move the arm to a centered, stow-safe position using a standard arm_pose_command.
    Called when ESC fires mid-draw to break out of ArmSurfaceContact mode.

    Targets a body-frame centered position (65 cm forward, 0 cm lateral, 15 cm
    above flat-body) so the arm is clear of both the canvas AND any lateral-extreme
    under-body configuration before arm_stow_command is issued.
    """
    try:
        robot_state = state_client.get_robot_state()
        (_, vision_T_flat_body, _, _, _, _) = get_transforms(True, robot_state)
        hand_quat = Quat(w=0.707, x=0, y=0.707, z=0)
        # Transform body-frame safe position to vision frame:
        # 0.65 m forward, 0.0 m lateral, 0.15 m above flat-body (centred, stow-safe)
        ax, ay, az = vision_T_flat_body.transform_point(0.65, 0.0, 0.15)
        lift_cmd = RobotCommandBuilder.arm_pose_command(
            ax, ay, az,
            hand_quat.w, hand_quat.x, hand_quat.y, hand_quat.z,
            VISION_FRAME_NAME, 4.0,
        )
        lift_id = cmd_client.robot_command(RobotCommandBuilder.build_synchro_command(lift_cmd))
        block_until_arm_arrives(cmd_client, lift_id, timeout_sec=5)
    except Exception:
        time.sleep(4)  # fallback wait if state read or arrival check fails


# ---------------------------------------------------------------------------
# Emergency-stop cleanup helper
# ---------------------------------------------------------------------------


def _do_estop_cleanup(cmd_client) -> None:
    print("\n[ESTOP] Stowing arm…")
    _stow_arm_blocking(cmd_client)
    print("  Sitting…")
    try:
        cmd_client.robot_command(RobotCommandBuilder.synchro_sit_command())
        time.sleep(2)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------


def run(vicon_client, robot, draw_speed: float = 1.0) -> None:
    cmd_client = robot.ensure_client(RobotCommandClient.default_service_name)
    state_client = robot.ensure_client(RobotStateClient.default_service_name)
    asc_client = robot.ensure_client(ArmSurfaceContactClient.default_service_name)
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    estop_client = robot.ensure_client(EstopClient.default_service_name)

    estop_ep = EstopEndpoint(estop_client, name="draw_gcode_estop", estop_timeout=9.0)
    estop_ep.force_simple_setup()

    cfg = _load_cfg()
    if draw_speed != 1.0:
        cfg["velocity"] *= draw_speed
        print(f"Draw speed ×{draw_speed:.2f}  →  velocity {cfg['velocity']:.3f} m/s")

    with (
        EstopKeepAlive(estop_ep),
        LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True),
    ):
        # ── Stand up ──────────────────────────────────────────────────────
        print("Standing up…")
        robot.power_on(timeout_sec=20)
        blocking_stand(cmd_client, timeout_sec=10)
        print("Robot standing.\n")
        print("  SPACE / ESC = emergency stop")
        print("  ENTER       = confirm")
        print("  x           = sit and exit when prompted\n")
        _ctrl.start_key_listener()

        # ── Walk to canvas ─────────────────────────────────────────────────
        print("─── Walk to canvas")
        if not walk_to_canvas(cmd_client, vicon_client):
            _do_estop_cleanup(cmd_client)
            return

        # ── Place marker ───────────────────────────────────────────────────
        print("\n─── Place marker")
        if not open_for_marker(cmd_client):
            _do_estop_cleanup(cmd_client)
            return

        # ── Drawing loop ───────────────────────────────────────────────────
        while True:
            if not _ctrl.check():
                break

            # Select gcode — stop key listener so input() works normally
            print("\n─── Select gcode")
            _ctrl.stop_key_listener()
            os.makedirs(G_CODES_DIR, exist_ok=True)
            gcode_path = select_gcode()

            # Compute auto-scale from Vicon canvas
            frame = vicon_client.latest_frame
            if frame and frame.canvas and frame.canvas.is_valid():
                print("  Computing auto-scale from canvas…")
                scale = compute_scale(
                    gcode_path, frame.canvas,
                    margin=cfg["canvas_margin"],
                    arm_draw_x_m=cfg["arm_draw_x"],
                    arm_draw_y_m=cfg["arm_draw_y"],
                )
            else:
                print("  [WARN] Canvas not visible — using default scale 0.001 m/unit")
                scale = 0.001

            _ctrl.start_key_listener()

            if not _ctrl.check():
                break

            # ── Draw ────────────────────────────────────────────────────
            print(f"\n─── Drawing: {os.path.basename(gcode_path)}")
            ok = _draw_file(
                gcode_path,
                scale,
                vicon_client,
                state_client,
                cmd_client,
                asc_client,
                robot.logger,
                cfg,
            )

            # ── Stow arm after drawing ───────────────────────────────────
            # _draw_file already stows on success; this is a safety fallback
            # for the estop / error path where it returned early.
            # block_until_arm_arrives ensures the arm is fully stowed before
            # we proceed — blind time.sleep(4) was not enough when pulling
            # the arm off the canvas surface.
            print()
            _stow_arm_blocking(cmd_client)
            try:
                blocking_stand(cmd_client, timeout_sec=10)
            except Exception:
                pass

            if not ok or not _ctrl.check():
                break

            # ── "Draw another?" — stop key listener for input() ──────────
            _ctrl.stop_key_listener()
            print()
            print("┌─────────────────────────────────────────┐")
            print("│  Drawing complete.                      │")
            print("│  Draw another file?  [y/N]              │")
            print("└─────────────────────────────────────────┘")
            ans = input("> ").strip().lower()
            if ans != "y":
                _ctrl.start_key_listener()
                break

            print()
            print("  Change marker?  [y/N]")
            change_marker = input("> ").strip().lower() == "y"
            _ctrl.start_key_listener()

            if change_marker:
                print("\n─── Change marker")
                if not open_for_marker(cmd_client):
                    break

            # Loop back to gcode selection

        # ── Sit and exit ───────────────────────────────────────────────────
        # IMPORTANT: always stow first, stand, then sit — never sit while the
        # arm is extended or the robot will sit on top of the arm and break it.
        print("\nCleaning up…")
        _stow_arm_blocking(cmd_client)
        try:
            blocking_stand(cmd_client, timeout_sec=10)
            cmd_client.robot_command(RobotCommandBuilder.synchro_sit_command())
            time.sleep(2)
        except Exception:
            pass
        print("Done.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(
        description="draw_gcode — autonomous walk + marker grab + gcode drawing",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument(
        "--vicon",
        metavar="HOST:PORT",
        help=f"Vicon address (default from .env: {VICON_ADDRESS})",
    )
    ap.add_argument(
        "--draw-speed",
        type=float,
        default=1.0,
        metavar="MULT",
        help="velocity multiplier for drawing (0.5 = half speed, 2.0 = double, etc.)",
    )
    args = ap.parse_args()

    vicon_addr = args.vicon or VICON_ADDRESS
    if not vicon_addr:
        raise SystemExit("No VICON_HOST set and --vicon not specified.")

    print(f"Connecting to Vicon at {vicon_addr}…")
    vicon = ViconClient(host=vicon_addr)
    vicon.start()
    deadline = time.time() + VICON_CONNECT_TO
    while vicon.latest_frame is None and time.time() < deadline:
        time.sleep(0.05)
    if vicon.latest_frame is None:
        raise SystemExit(f"No Vicon frames after {VICON_CONNECT_TO:.0f} s.")

    frame = vicon.latest_frame
    if frame.canvas is None or not frame.canvas.is_valid():
        vicon.stop()
        raise SystemExit(
            "Canvas not visible — check test_canvas markers in Vicon Tracker."
        )

    spot_ok = frame.spot_body is not None and not frame.spot_body.occluded
    print(
        f"Vicon OK  |  canvas {frame.canvas.width_mm:.0f} × {frame.canvas.height_mm:.0f} mm"
        f"  |  spot_base: {'OK' if spot_ok else 'NO DATA'}\n"
    )
    if not spot_ok:
        print("[WARN] spot_base not visible — navigation will wait for it.\n")

    print("Connecting to Spot…")
    bosdyn.client.util.setup_logging(verbose=False)
    sdk = bosdyn.client.create_standard_sdk("DrawGcode")
    robot = sdk.create_robot(ROBOT_IP)
    robot.authenticate(USERNAME, PASSWORD)
    robot.time_sync.wait_for_sync()
    print("Spot connected.\n")

    try:
        run(vicon, robot, draw_speed=args.draw_speed)
    finally:
        vicon.stop()


if __name__ == "__main__":
    main()
