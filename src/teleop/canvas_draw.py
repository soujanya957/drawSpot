"""
Canvas Draw — 2-D Drag → Robot End-Effector
============================================
Opens a Pygame window showing a blank canvas (representing the physical
drawing surface in front of the robot). The user DRAGS the mouse on the
canvas; the script continuously maps the 2-D mouse position to the robot's
end-effector position parallel to the ground at a fixed Z height, so a
marker held in the gripper traces whatever the user draws.

Coordinate mapping:
    Canvas pixel (0,0)                   → robot body frame (x_max, y_max)
    Canvas pixel (WIN_W, WIN_H)          → robot body frame (x_min, y_min)
    Z is always CANVAS_Z_HEIGHT (metres) → end-effector stays parallel to ground

Usage:
    1. Place the robot in front of a flat surface (table / paper).
    2. Have the robot hold a marker in its gripper (or attach one).
    3. Run the script — the arm moves to the canvas centre on startup.
    4. Click and drag in the Pygame window to draw.

Run from the repo root:
    python -m src.manipulation.canvas_draw

Controls (Pygame window):
    Left mouse button held — draw (move end-effector)
    r                      — lift pen (raise Z slightly), reset canvas
    s                      — screenshot the canvas to canvas_output.png
    ESC / close window     — stow arm and exit

Requirements:
    pip install bosdyn-client bosdyn-mission bosdyn-api pygame
    Edit config/robot_config.py to set CANVAS_WIDTH_M, CANVAS_HEIGHT_M,
    CANVAS_Z_HEIGHT, and robot connection details.
"""

import sys
import time
import math
import threading

import pygame

import bosdyn.client
import bosdyn.client.util
import bosdyn.client.math_helpers as math_helpers
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand
from bosdyn.api import arm_command_pb2, robot_command_pb2, synchronized_command_pb2
from bosdyn.api.geometry_pb2 import SE3Pose, Vec3, Quaternion
from google.protobuf import duration_pb2

sys.path.insert(0, __file__.split("/src/")[0])
from config.robot_config import (ROBOT_IP, USERNAME, PASSWORD,
                                  CANVAS_WIDTH_M, CANVAS_HEIGHT_M,
                                  CANVAS_Z_HEIGHT, ARM_FRAME)

# --- Window settings ---
WIN_W = 600   # pixels
WIN_H = 600   # pixels
FPS   = 30

# Canvas real-world bounds in body frame (metres)
# Arm reaches forward (positive x) and left/right (y)
X_MAX =  0.75
X_MIN =  X_MAX - CANVAS_WIDTH_M
Y_MAX =  CANVAS_WIDTH_M / 2
Y_MIN = -CANVAS_WIDTH_M / 2
Z_DRAW  = CANVAS_Z_HEIGHT
Z_LIFT  = CANVAS_Z_HEIGHT + 0.06   # lift pen a bit when not drawing

# Minimum distance (metres) before we bother sending a new arm command
MOVE_THRESHOLD = 0.003   # 3 mm


def _duration(secs):
    return duration_pb2.Duration(seconds=int(secs),
                                  nanos=int((secs - int(secs)) * 1e9))


def pixel_to_robot(px, py):
    """Convert Pygame pixel coords to robot body-frame (x, y, z)."""
    # px=0 → left edge of canvas → robot's maximum +Y (robot's left)
    # px=WIN_W → right edge → robot's maximum -Y (robot's right)
    # py=0 → top of canvas → robot's maximum +X (furthest forward)
    # py=WIN_H → bottom of canvas → robot's minimum X (nearest)
    rx = X_MAX - (py / WIN_H) * CANVAS_HEIGHT_M
    ry = Y_MAX - (px / WIN_W) * CANVAS_WIDTH_M
    return rx, ry


def send_arm_pose(cmd_client, x, y, z, duration_s=0.15):
    """
    Command end-effector to (x, y, z) in body frame, camera pointing straight down
    (pitch = 90°, end-effector facing down — marker tip points down).
    """
    # Pitch 90° → hand/marker points downward
    cmd = robot_command_pb2.RobotCommand()
    arm_cart = cmd.synchronized_command.arm_command.arm_cartesian_command
    arm_cart.root_frame_name = ARM_FRAME
    point = arm_cart.pose_trajectory_in_task.points.add()
    point.pose.position.x = x
    point.pose.position.y = y
    point.pose.position.z = z
    point.pose.rotation.w = 0.7071068
    point.pose.rotation.x = 0.0
    point.pose.rotation.y = 0.7071068
    point.pose.rotation.z = 0.0
    point.time_since_reference.CopyFrom(_duration(duration_s))
    cmd_client.robot_command(cmd)


def run_canvas(robot):
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    cmd_client   = robot.ensure_client(RobotCommandClient.default_service_name)

    with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        robot.power_on(timeout_sec=20)
        blocking_stand(cmd_client, timeout_sec=10)

        # Move arm to canvas centre at lift height
        cx = (X_MAX + X_MIN) / 2
        cy = (Y_MAX + Y_MIN) / 2
        print("Moving arm to canvas centre…")
        send_arm_pose(cmd_client, cx, cy, Z_LIFT, duration_s=2.5)
        time.sleep(3)

        # --- Pygame setup ---
        pygame.init()
        screen    = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption("Canvas Draw — drag to draw, r=reset, s=save, ESC=quit")
        clock     = pygame.time.Clock()

        canvas_surf = pygame.Surface((WIN_W, WIN_H))
        canvas_surf.fill((255, 255, 255))   # white canvas

        last_rx, last_ry, last_rz = cx, cy, Z_LIFT
        prev_mouse = None
        pen_down   = False

        print("Canvas open. Hold left mouse button and drag to draw.")

        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_r:
                        # Lift pen and clear canvas
                        canvas_surf.fill((255, 255, 255))
                        send_arm_pose(cmd_client, last_rx, last_ry, Z_LIFT)
                        last_rz = Z_LIFT
                        prev_mouse = None
                        print("Canvas cleared, pen lifted.")
                    elif event.key == pygame.K_s:
                        fname = "canvas_output.png"
                        pygame.image.save(canvas_surf, fname)
                        print(f"Canvas saved to {fname}")

            buttons = pygame.mouse.get_pressed()
            mx, my  = pygame.mouse.get_pos()

            if buttons[0]:  # left button held → draw
                rx, ry = pixel_to_robot(mx, my)
                rz = Z_DRAW

                # Draw on canvas surface
                if prev_mouse is not None:
                    pygame.draw.line(canvas_surf, (0, 0, 0), prev_mouse, (mx, my), 2)
                else:
                    pygame.draw.circle(canvas_surf, (0, 0, 0), (mx, my), 2)

                # Only send command if arm moved enough
                dist = math.hypot(rx - last_rx, ry - last_ry)
                if dist > MOVE_THRESHOLD or rz != last_rz:
                    send_arm_pose(cmd_client, rx, ry, rz)
                    last_rx, last_ry, last_rz = rx, ry, rz

                prev_mouse = (mx, my)
                pen_down   = True

            else:
                if pen_down:
                    # Lift pen when mouse released
                    send_arm_pose(cmd_client, last_rx, last_ry, Z_LIFT)
                    last_rz = Z_LIFT
                    pen_down = False
                prev_mouse = None

            # Render
            screen.fill((200, 200, 200))   # grey border
            screen.blit(canvas_surf, (0, 0))

            # Cursor dot
            pygame.draw.circle(screen, (255, 0, 0), (mx, my), 5)

            pygame.display.flip()
            clock.tick(FPS)

        pygame.quit()

        # Clean exit
        stow = RobotCommandBuilder.arm_stow_command()
        cmd_client.robot_command(stow)
        time.sleep(3)
        sit = RobotCommandBuilder.synchro_sit_command()
        cmd_client.robot_command(sit)
        time.sleep(2)
        print("Arm stowed and robot sat. Goodbye.")


def main():
    sdk   = bosdyn.client.create_standard_sdk("CanvasDraw")
    robot = sdk.create_robot(ROBOT_IP)
    robot.authenticate(USERNAME, PASSWORD)
    robot.time_sync.wait_for_sync()
    run_canvas(robot)


if __name__ == "__main__":
    main()
