"""
Object Picker — Camera-Guided Grasp
====================================
1. Powers on the robot and stands it up.
2. Moves the arm to a top-down "survey" pose so the hand camera points at the
   ground plane in front of the robot.
3. Captures an image from the hand color camera and displays it with OpenCV.
4. You CLICK on the object you want to pick in the displayed window.
5. The script converts your pixel click to a 3-D ray and passes it to the
   Spot Manipulation API (PickObjectInImage), which autonomously plans and
   executes the grasp.
6. After grasping, it prints success/failure and optionally stows the arm.

Run from the repo root:
    python -m src.manipulation.pick_object

Controls (OpenCV window):
    Left-click   — select pick point, trigger grasp
    q            — quit without picking

Requirements:
    pip install bosdyn-client bosdyn-mission bosdyn-api opencv-python
    Edit config/robot_config.py with your robot's IP, username, and password.
"""

import sys
import time

import cv2
import numpy as np

# Module-level globals for the OpenCV mouse callback (required by Qt on macOS)
g_image_click = None
g_image_display = None

import bosdyn.client
import bosdyn.client.util
import bosdyn.client.math_helpers as math_helpers
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.api import manipulation_api_pb2, geometry_pb2
from bosdyn.api import arm_command_pb2, robot_command_pb2, synchronized_command_pb2
from bosdyn.api.geometry_pb2 import SE3Pose, Vec3, Quaternion
from google.protobuf import duration_pb2

sys.path.insert(0, __file__.split("/src/")[0])
from config.robot_config import ROBOT_IP, USERNAME, PASSWORD

# Hand camera source name
HAND_COLOR_SOURCE = "hand_color_image"

# Survey pose — arm stretched forward, camera pointing straight down
# (body frame, metres). Adjust z to change hover height.
SURVEY_POSE = dict(x=0.7, y=0.0, z=0.0,
                   roll=0.0, pitch=1.5708, yaw=0.0)   # pitch ≈ 90° → camera down


def _cv_mouse_callback(event, x, y, flags, param):
    global g_image_click, g_image_display
    clone = g_image_display.copy()
    if event == cv2.EVENT_LBUTTONUP:
        g_image_click = (x, y)
    else:
        # Draw crosshair while moving
        h, w = clone.shape[:2]
        cv2.line(clone, (0, y), (w, y), (30, 30, 30), 2)
        cv2.line(clone, (x, 0), (x, h), (30, 30, 30), 2)
        cv2.imshow('Click to grasp', clone)


def _duration(secs):
    return duration_pb2.Duration(seconds=int(secs),
                                  nanos=int((secs - int(secs)) * 1e9))


def move_arm_to_survey(cmd_client):
    """Move end-effector to top-down survey pose."""
    p = SURVEY_POSE
    import math as _math
    cr, sr = _math.cos(p['roll']/2),  _math.sin(p['roll']/2)
    cp, sp = _math.cos(p['pitch']/2), _math.sin(p['pitch']/2)
    cy, sy = _math.cos(p['yaw']/2),   _math.sin(p['yaw']/2)
    hand_pose = SE3Pose(
        position=Vec3(x=p['x'], y=p['y'], z=p['z']),
        rotation=Quaternion(
            w=cr*cp*cy + sr*sp*sy,
            x=sr*cp*cy - cr*sp*sy,
            y=cr*sp*cy + sr*cp*sy,
            z=cr*cp*sy - sr*sp*cy,
        ),
    )
    cmd = robot_command_pb2.RobotCommand()
    arm_cart = cmd.synchronized_command.arm_command.arm_cartesian_command
    arm_cart.root_frame_name = "body"
    point = arm_cart.pose_trajectory_in_task.points.add()
    point.pose.position.x = hand_pose.position.x
    point.pose.position.y = hand_pose.position.y
    point.pose.position.z = hand_pose.position.z
    point.pose.rotation.CopyFrom(hand_pose.rotation)
    point.time_since_reference.CopyFrom(_duration(2.5))
    cmd_client.robot_command(cmd)
    time.sleep(3)


def capture_image(image_client):
    """Capture one frame from the hand color camera. Returns (np image, ImageResponse)."""
    req  = build_image_request(HAND_COLOR_SOURCE, quality_percent=75)
    resp = image_client.get_image([req])[0]
    img_bytes = np.frombuffer(resp.shot.image.data, dtype=np.uint8)
    img = cv2.imdecode(img_bytes, cv2.IMREAD_COLOR)
    return img, resp


def pick_at_pixel(manip_client, image_response, px, py):
    """
    Call the Spot Manipulation API to grasp at pixel (px, py) in the given image.
    Returns the ManipulationApiResponse.
    """
    pick_request = manipulation_api_pb2.PickObjectInImage(
        pixel_xy=geometry_pb2.Vec2(x=px, y=py),
        transforms_snapshot_for_camera=image_response.shot.transforms_snapshot,
        frame_name_image_sensor=image_response.shot.frame_name_image_sensor,
        camera_model=image_response.source.pinhole,
    )
    grasp_request = manipulation_api_pb2.ManipulationApiRequest(
        pick_object_in_image=pick_request
    )
    response = manip_client.manipulation_api_command(
        manipulation_api_request=grasp_request
    )
    return response


def wait_for_grasp(manip_client, cmd_id, timeout=15.0):
    """Poll until the grasp command finishes or times out."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        feedback_req = manipulation_api_pb2.ManipulationApiFeedbackRequest(
            manipulation_cmd_id=cmd_id
        )
        fb = manip_client.manipulation_api_feedback_command(
            manipulation_api_feedback_request=feedback_req
        )
        state = fb.current_state
        print(f"  Grasp state: {manipulation_api_pb2.ManipulationFeedbackState.Name(state)}")
        if state in (
            manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED,
            manipulation_api_pb2.MANIP_STATE_GRASP_FAILED,
            manipulation_api_pb2.MANIP_STATE_GRASP_PLANNING_NO_SOLUTION,
        ):
            return state
        time.sleep(0.5)
    print("Grasp timed out.")
    return None


def run_picker(robot):
    print("[1/6] Acquiring clients…")
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    cmd_client   = robot.ensure_client(RobotCommandClient.default_service_name)
    img_client   = robot.ensure_client(ImageClient.default_service_name)
    manip_client = robot.ensure_client(ManipulationApiClient.default_service_name)
    estop_client = robot.ensure_client(EstopClient.default_service_name)
    print("[2/6] Setting up E-Stop…")
    estop_endpoint = EstopEndpoint(estop_client, name="pick_object_estop", estop_timeout=9.0)
    estop_endpoint.force_simple_setup()
    print("[3/6] E-Stop configured. Acquiring lease…")

    with EstopKeepAlive(estop_endpoint), \
         LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        print("[4/6] Lease acquired. Powering on motors…")
        robot.power_on(timeout_sec=20)
        print("[5/6] Motors on. Standing up…")
        blocking_stand(cmd_client, timeout_sec=10)
        print("[6/6] Standing. Ready.\n")

        print("Moving arm to survey position…")
        move_arm_to_survey(cmd_client)

        print("Capturing image…")
        img, img_response = capture_image(img_client)

        global g_image_click, g_image_display
        g_image_click = None
        g_image_display = img

        image_title = 'Click to grasp'
        cv2.namedWindow(image_title)
        cv2.setMouseCallback(image_title, _cv_mouse_callback)
        cv2.imshow(image_title, g_image_display)
        print("Click on the object you want to pick. Press q to cancel.")

        while g_image_click is None:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q'):
                cv2.destroyAllWindows()
                print("Cancelled.")
                return

        px, py = g_image_click
        cv2.destroyAllWindows()
        print(f"Picking at pixel ({px}, {py})…")
        resp = pick_at_pixel(manip_client, img_response, float(px), float(py))
        final_state = wait_for_grasp(manip_client, resp.manipulation_cmd_id)

        if final_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED:
            print("Grasp SUCCEEDED!")
        else:
            print(f"Grasp ended with state: {final_state}")

        # Stow arm after grasp attempt
        time.sleep(1)
        stow = RobotCommandBuilder.arm_stow_command()
        cmd_client.robot_command(stow)
        time.sleep(3)
        print("Arm stowed.")


def main():
    sdk   = bosdyn.client.create_standard_sdk("PickObject")
    robot = sdk.create_robot(ROBOT_IP)
    robot.authenticate(USERNAME, PASSWORD)
    robot.time_sync.wait_for_sync()
    run_picker(robot)


if __name__ == "__main__":
    main()
