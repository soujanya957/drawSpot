"""
Vicon-driven Base Navigation
=============================
Spot's base mirrors the positional displacement of the Vicon "Spot" rigid body
from its starting position.  If the tracked body moves 0.5 m in X, Spot walks
0.5 m in X — no velocity commands, just SE2 positional goals in the odom frame.

Z is ignored (Spot walks on flat ground).
Yaw is read from the Vicon body quaternion and mirrored too; pass --no-yaw to
lock heading at 0.

Run from repo root:
    python -m src.navigation.vicon_base_follow --vicon HOST:PORT

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
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME

sys.path.insert(0, __file__.split("/src/")[0])
from config.robot_config import ROBOT_IP, USERNAME, PASSWORD

# How often to push a new goal to the robot (seconds)
COMMAND_PERIOD_S = 0.1
# How far in advance the trajectory goal is valid (robot stops if no new command arrives)
GOAL_HORIZON_S = 3.0
# Minimum displacement change (m) to bother re-sending a command
DEAD_ZONE_M = 0.01

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("vicon_base_follow")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _yaw_from_quat(q: np.ndarray) -> float:
    """Extract yaw (rad) from quaternion [qx, qy, qz, qw]."""
    qx, qy, qz, qw = q
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _connect_vicon(host: str):
    sys.path.insert(0, __file__.split("/src/")[0])
    from vicon.client import ViconClient
    client = ViconClient(host=host)
    client.start()
    log.info("Waiting for first Vicon frame…")
    while client.latest_frame is None:
        time.sleep(0.01)
    return client


def _wait_for_body(vicon_client, timeout: float = 5.0) -> tuple:
    """Block until a valid spot_body frame arrives. Returns (position_mm, yaw_rad)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        f = vicon_client.latest_frame
        if f and f.spot_body and not f.spot_body.occluded:
            return f.spot_body.position.copy(), _yaw_from_quat(f.spot_body.rotation_quat)
        time.sleep(0.02)
    raise RuntimeError("Timed out waiting for Vicon spot_body data.")


def _send_se2_goal(cmd_client, goal_x: float, goal_y: float, goal_heading: float) -> None:
    """Command Spot's base to walk to (goal_x, goal_y, goal_heading) in the odom frame."""
    cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
        goal_x=goal_x,
        goal_y=goal_y,
        goal_heading=goal_heading,
        frame_name=ODOM_FRAME_NAME,
        params=None,
    )
    cmd_client.robot_command(command=cmd, end_time_secs=time.time() + GOAL_HORIZON_S)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def run(vicon_client, robot, mirror_yaw: bool) -> None:
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    cmd_client   = robot.ensure_client(RobotCommandClient.default_service_name)

    with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        robot.power_on(timeout_sec=20)
        blocking_stand(cmd_client, timeout_sec=10)
        log.info("Standing. Recording Vicon reference position…")

        ref_pos_mm, ref_yaw = _wait_for_body(vicon_client)
        log.info("Reference  x=%.1f mm  y=%.1f mm  yaw=%.2f rad",
                 ref_pos_mm[0], ref_pos_mm[1], ref_yaw)
        log.info("Spot will mirror Vicon body displacement 1:1 (mm → m). Ctrl-C to exit.")

        last_goal = (0.0, 0.0, 0.0)

        try:
            while True:
                frame = vicon_client.latest_frame
                if frame is None or frame.spot_body is None or frame.spot_body.occluded:
                    time.sleep(COMMAND_PERIOD_S)
                    continue

                # Displacement from reference in Vicon world frame (mm → m)
                delta_mm = frame.spot_body.position - ref_pos_mm
                goal_x = float(delta_mm[0]) / 1000.0
                goal_y = float(delta_mm[1]) / 1000.0  # z ignored

                if mirror_yaw:
                    current_yaw = _yaw_from_quat(frame.spot_body.rotation_quat)
                    goal_heading = current_yaw - ref_yaw
                else:
                    goal_heading = 0.0

                # Dead zone — skip if nothing meaningful changed
                dx = goal_x - last_goal[0]
                dy = goal_y - last_goal[1]
                if math.hypot(dx, dy) < DEAD_ZONE_M:
                    time.sleep(COMMAND_PERIOD_S)
                    continue

                log.info("Vicon disp  Δx=%+.3f m  Δy=%+.3f m  heading=%.2f rad",
                         goal_x, goal_y, goal_heading)

                _send_se2_goal(cmd_client, goal_x, goal_y, goal_heading)
                last_goal = (goal_x, goal_y, goal_heading)
                time.sleep(COMMAND_PERIOD_S)

        except KeyboardInterrupt:
            log.info("Interrupted — sitting down.")

        cmd_client.robot_command(RobotCommandBuilder.synchro_sit_command())
        time.sleep(2)
        log.info("Done.")


def main() -> None:
    ap = argparse.ArgumentParser(description="Vicon → Spot base positional follower")
    ap.add_argument("--vicon", required=True, metavar="HOST:PORT",
                    help="Vicon DataStream address, e.g. 192.168.1.10:801")
    ap.add_argument("--no-yaw", action="store_true",
                    help="Lock heading at 0 instead of mirroring Vicon body yaw")
    args = ap.parse_args()

    vicon = _connect_vicon(args.vicon)

    sdk   = bosdyn.client.create_standard_sdk("ViconBaseFollow")
    robot = sdk.create_robot(ROBOT_IP)
    robot.authenticate(USERNAME, PASSWORD)
    robot.time_sync.wait_for_sync()

    try:
        run(vicon, robot, mirror_yaw=not args.no_yaw)
    finally:
        vicon.stop()


if __name__ == "__main__":
    main()
