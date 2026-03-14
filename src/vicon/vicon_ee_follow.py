"""
Vicon-driven End-Effector Control (base locked)
=================================================
Spot's arm end-effector follows a Vicon-tracked marker in 3D space.
The base does not move.

The target marker is resolved in the Vicon world frame, then converted to
Spot's body frame using the tracked body pose.  A reach check is applied
before sending any command — if the target is outside the arm's workspace
the command is silently skipped.

Arm workspace limits (conservative, from body centre):
    MAX reach : 0.85 m   (hardware limit ~0.9 m)
    MIN reach : 0.15 m   (arm cannot fold back this far)
    X (forward): must be positive — arm cannot reach behind the body

The end-effector orientation is kept fixed at pitch=90° (pointing straight
forward/down along the arm), which works well for overhead or downward tasks.
Pass --pitch DEG to override.

Run from repo root:
    python -m src.manipulation.vicon_ee_follow --vicon HOST:PORT [--marker BrushTip]

Requirements:
    pip install bosdyn-client bosdyn-api vicon-dssdk
    Edit config/robot_config.py with your robot's IP, username, and password.
"""

import argparse
import logging
import math
import sys
import time

import numpy as np
import bosdyn.client
import bosdyn.client.util
import bosdyn.client.math_helpers as math_helpers
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import (
    RobotCommandClient,
    RobotCommandBuilder,
    blocking_stand,
)
from bosdyn.client.frame_helpers import BODY_FRAME_NAME
from bosdyn.api import arm_command_pb2, robot_command_pb2, synchronized_command_pb2
from bosdyn.api.geometry_pb2 import SE3Pose, Vec3, Quaternion
from google.protobuf import duration_pb2

sys.path.insert(0, __file__.split("/src/")[0])
from config.robot_config import ROBOT_IP, USERNAME, PASSWORD

# Arm workspace limits (metres, measured from body centre)
SPOT_ARM_MAX_REACH_M = 0.85
SPOT_ARM_MIN_REACH_M = 0.15

# Arm command trajectory duration
ARM_CMD_DURATION_S = 0.3

# How often to push a new command (seconds)
COMMAND_PERIOD_S = 0.05

# Minimum position change (m) to re-send a command
DEAD_ZONE_M = 0.005

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("vicon_ee_follow")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _rotation_matrix_from_quat(q: np.ndarray) -> np.ndarray:
    """Build a 3x3 rotation matrix from quaternion [qx, qy, qz, qw]."""
    qx, qy, qz, qw = q
    return np.array(
        [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx**2 + qy**2)],
        ],
        dtype=float,
    )


def _world_to_body(
    target_mm: np.ndarray, body_pos_mm: np.ndarray, body_quat: np.ndarray
) -> np.ndarray:
    """
    Convert a world-frame position (mm) to Spot body frame (metres).

    Spot body frame: x=forward, y=left, z=up.
    """
    R = _rotation_matrix_from_quat(body_quat)
    delta_mm = target_mm - body_pos_mm
    # R maps world → body columns, so body coords = R^T @ delta
    body_mm = R.T @ delta_mm
    return body_mm / 1000.0  # mm → m


def _check_reach(pos_body_m: np.ndarray) -> tuple[bool, str]:
    """
    Return (ok, reason).  ok=True if the position is within the arm workspace.
    """
    dist = float(np.linalg.norm(pos_body_m))
    if dist > SPOT_ARM_MAX_REACH_M:
        return False, f"too far ({dist:.3f} m > {SPOT_ARM_MAX_REACH_M} m)"
    if dist < SPOT_ARM_MIN_REACH_M:
        return False, f"too close ({dist:.3f} m < {SPOT_ARM_MIN_REACH_M} m)"
    if pos_body_m[0] < 0.0:
        return False, f"behind body (x={pos_body_m[0]:.3f} m)"
    return True, "ok"


def _send_arm_command(
    cmd_client, x: float, y: float, z: float, pitch_rad: float
) -> None:
    """Send a Cartesian arm command in the body frame."""
    cp, sp = math.cos(pitch_rad / 2), math.sin(pitch_rad / 2)
    rot = Quaternion(w=cp, x=0.0, y=sp, z=0.0)  # pure pitch rotation
    duration = duration_pb2.Duration(seconds=0, nanos=int(ARM_CMD_DURATION_S * 1e9))
    cmd = robot_command_pb2.RobotCommand()
    arm_cart = cmd.synchronized_command.arm_command.arm_cartesian_command
    arm_cart.root_frame_name = BODY_FRAME_NAME
    point = arm_cart.pose_trajectory_in_task.points.add()
    point.pose.position.x = x
    point.pose.position.y = y
    point.pose.position.z = z
    point.pose.rotation.CopyFrom(rot)
    point.time_since_reference.CopyFrom(duration)
    cmd_client.robot_command(cmd)


def _connect_vicon(host: str):
    from draw.vicon.client import ViconClient

    client = ViconClient(host=host)
    client.start()
    log.info("Waiting for first Vicon frame…")
    while client.latest_frame is None:
        time.sleep(0.01)
    return client


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------


def run(vicon_client, robot, marker_name: str, pitch_deg: float) -> None:
    pitch_rad = math.radians(pitch_deg)

    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    cmd_client = robot.ensure_client(RobotCommandClient.default_service_name)

    with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        robot.power_on(timeout_sec=20)
        blocking_stand(cmd_client, timeout_sec=10)
        log.info(
            "Standing. Arm will follow Vicon marker '%s'. Ctrl-C to exit.", marker_name
        )
        log.info(
            "Reach limits: %.2f m – %.2f m from body centre.",
            SPOT_ARM_MIN_REACH_M,
            SPOT_ARM_MAX_REACH_M,
        )

        last_pos = None

        try:
            while True:
                frame = vicon_client.latest_frame
                if frame is None:
                    time.sleep(COMMAND_PERIOD_S)
                    continue

                # --- resolve target marker ---
                target_marker = None
                if marker_name.lower() == "brushtip" and frame.brush_tip:
                    target_marker = frame.brush_tip
                elif marker_name.lower() == "spotee" and frame.spot_ee:
                    target_marker = frame.spot_ee
                # fall back: search all markers on any rigid body
                if target_marker is None:
                    for rb in [frame.spot_body, frame.spot_ee]:
                        if rb is None:
                            continue
                        for m in rb.markers:
                            if m.name.lower() == marker_name.lower():
                                target_marker = m
                                break

                if target_marker is None or getattr(target_marker, "occluded", False):
                    log.debug("Marker '%s' not visible — skipping.", marker_name)
                    time.sleep(COMMAND_PERIOD_S)
                    continue

                # --- need body pose to do the transform ---
                if frame.spot_body is None or frame.spot_body.occluded:
                    log.debug("spot_body occluded — skipping.")
                    time.sleep(COMMAND_PERIOD_S)
                    continue

                # --- transform world → body frame ---
                pos_body_m = _world_to_body(
                    target_marker.position,
                    frame.spot_body.position,
                    frame.spot_body.rotation_quat,
                )

                # --- reach check ---
                ok, reason = _check_reach(pos_body_m)
                if not ok:
                    log.warning(
                        "Out of reach — %s  body=(%.3f, %.3f, %.3f) m",
                        reason,
                        *pos_body_m,
                    )
                    time.sleep(COMMAND_PERIOD_S)
                    continue

                # --- dead zone ---
                if last_pos is not None:
                    if np.linalg.norm(pos_body_m - last_pos) < DEAD_ZONE_M:
                        time.sleep(COMMAND_PERIOD_S)
                        continue

                log.info(
                    "EE target  body=(x=%+.3f  y=%+.3f  z=%+.3f) m  |r|=%.3f m",
                    pos_body_m[0],
                    pos_body_m[1],
                    pos_body_m[2],
                    np.linalg.norm(pos_body_m),
                )

                _send_arm_command(
                    cmd_client,
                    float(pos_body_m[0]),
                    float(pos_body_m[1]),
                    float(pos_body_m[2]),
                    pitch_rad,
                )
                last_pos = pos_body_m.copy()
                time.sleep(COMMAND_PERIOD_S)

        except KeyboardInterrupt:
            log.info("Interrupted — stowing arm and sitting.")

        cmd_client.robot_command(RobotCommandBuilder.arm_stow_command())
        time.sleep(2)
        cmd_client.robot_command(RobotCommandBuilder.synchro_sit_command())
        time.sleep(2)
        log.info("Done.")


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Vicon marker → Spot end-effector follower"
    )
    ap.add_argument(
        "--vicon",
        required=True,
        metavar="HOST:PORT",
        help="Vicon DataStream address, e.g. 192.168.1.10:801",
    )
    ap.add_argument(
        "--marker",
        default="BrushTip",
        metavar="NAME",
        help="Vicon marker/subject name to follow (default: BrushTip)",
    )
    ap.add_argument(
        "--pitch",
        type=float,
        default=90.0,
        metavar="DEG",
        help="End-effector pitch in degrees (default: 90 = pointing forward/down)",
    )
    args = ap.parse_args()

    vicon = _connect_vicon(args.vicon)

    sdk = bosdyn.client.create_standard_sdk("ViconEEFollow")
    robot = sdk.create_robot(ROBOT_IP)
    robot.authenticate(USERNAME, PASSWORD)
    robot.time_sync.wait_for_sync()

    try:
        run(vicon, robot, marker_name=args.marker, pitch_deg=args.pitch)
    finally:
        vicon.stop()


if __name__ == "__main__":
    main()
