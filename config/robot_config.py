"""
Robot Configuration File
========================
Reads connection credentials from environment variables.

Set these before running any script (or add them to a .env file):
    export SPOT_IP=192.168.80.3
    export SPOT_USER=user
    export SPOT_PASSWORD=yourpassword

Usage:
    from config.robot_config import ROBOT_IP, USERNAME, PASSWORD
"""

import os

def _require(name):
    val = os.environ.get(name)
    if not val:
        raise EnvironmentError(f"Missing required environment variable: {name}")
    return val

# --- Robot Connection Settings ---
ROBOT_IP = _require("SPOT_IP")
USERNAME = _require("SPOT_USER")
PASSWORD = _require("SPOT_PASSWORD")

# --- Arm / Motion Tuning ---
ARM_VELOCITY_MAX = 0.5  # m/s, max end-effector cartesian speed
BASE_VELOCITY_MAX = 0.5  # m/s, max base linear speed
BASE_ROTATION_MAX = 0.5  # rad/s, max base yaw speed

# --- Canvas Draw Settings (canvas_draw.py) ---
CANVAS_WIDTH_M = 0.4  # real-world width of the drawing canvas (metres)
CANVAS_HEIGHT_M = 0.4  # real-world height of the drawing canvas (metres)
CANVAS_Z_HEIGHT = 0.05  # height of end-effector above the canvas (metres)

# Frame used for arm Cartesian commands ("hand", "body", "odom", etc.)
ARM_FRAME = "body"
