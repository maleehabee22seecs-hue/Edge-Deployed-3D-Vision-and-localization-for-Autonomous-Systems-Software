#!/usr/bin/env python3
"""Smoke tests for ROS shim interfaces.

Checks:
1) Topic smoke: /camera/image_raw and /odom callbacks update internal cache.
2) API parity smoke: required methods/properties exist and are callable.
3) Command smoke: set_velocity publishes Twist on /cmd_vel.
4) Import smoke: shim module imports and constructs cleanly in ROS env.
"""

import argparse
import os
import sys
import threading

import rospy
from geometry_msgs.msg import Twist

# Ensure we import the source shim module, not the catkin relay wrapper from devel/lib.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from ros_shims import RosCameraStream, RosMotorController


def resolve_calib_file(calib_file: str):
    if calib_file is None:
        return None

    if os.path.isabs(calib_file) and os.path.exists(calib_file):
        return calib_file

    candidates = [
        os.path.abspath(calib_file),
        os.path.abspath(os.path.join(os.getcwd(), calib_file)),
    ]

    script_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_root = os.path.abspath(os.path.join(script_dir, "..", "..", ".."))
    candidates.append(os.path.join(workspace_root, "src", "fydp", calib_file))

    for path in candidates:
        if path and os.path.exists(path):
            return path

    return calib_file


def assert_api_parity(stream, motor):
    required_stream_attrs = [
        "K",
        "raw_K",
        "dist",
        "image_size",
        "undistorted_image_size",
        "capture",
        "stop",
    ]
    required_motor_methods = [
        "connect",
        "get_pose",
        "set_velocity",
        "stop",
        "reset_pose",
        "close",
        "keyboard_control_non_blocking",
    ]

    for attr in required_stream_attrs:
        if not hasattr(stream, attr):
            raise AssertionError(f"RosCameraStream missing attribute: {attr}")

    for name in required_motor_methods:
        if not hasattr(motor, name):
            raise AssertionError(f"RosMotorController missing method: {name}")
        if not callable(getattr(motor, name)):
            raise AssertionError(f"RosMotorController member is not callable: {name}")


def main():
    parser = argparse.ArgumentParser(description="ROS shim smoke test")
    parser.add_argument("--calib-file", default=None)
    parser.add_argument("--image-topic", default="/camera/image_raw")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--image-timeout", type=float, default=5.0)
    parser.add_argument("--odom-timeout", type=float, default=5.0)
    parser.add_argument("--cmd-timeout", type=float, default=2.0)
    args = parser.parse_args()

    args.calib_file = resolve_calib_file(args.calib_file)
    if args.calib_file is not None:
        print(f"Resolved calib file: {args.calib_file}")

    # Import smoke happens at module import time above.
    stream = RosCameraStream(calib_file=args.calib_file, image_topic=args.image_topic)
    motor = RosMotorController(cmd_topic=args.cmd_topic, odom_topic=args.odom_topic)
    motor.connect()

    assert_api_parity(stream, motor)

    if not stream.wait_for_image(timeout=args.image_timeout):
        raise AssertionError(
            f"Topic smoke failed: no image callback update from {args.image_topic} within {args.image_timeout}s"
        )

    if not motor.wait_for_odom(timeout=args.odom_timeout):
        raise AssertionError(
            f"Topic smoke failed: no odom callback update from {args.odom_topic} within {args.odom_timeout}s"
        )

    color = stream.capture(undistort=False, grayscale=False)
    gray = stream.capture(undistort=False, grayscale=True)
    if color is None or gray is None:
        raise AssertionError("Capture smoke failed: capture returned None")
    if len(color.shape) != 3:
        raise AssertionError(f"Capture smoke failed: color frame should be 3D, got {color.shape}")
    if len(gray.shape) != 2:
        raise AssertionError(f"Capture smoke failed: grayscale frame should be 2D, got {gray.shape}")

    cmd_event = threading.Event()
    received = {"msg": None}

    def _cmd_cb(msg):
        received["msg"] = msg
        cmd_event.set()

    cmd_sub = rospy.Subscriber(args.cmd_topic, Twist, _cmd_cb, queue_size=1)
    rospy.sleep(0.2)

    motor.set_velocity(42, -17)
    if not cmd_event.wait(timeout=args.cmd_timeout):
        raise AssertionError(
            f"Command smoke failed: no Twist observed on {args.cmd_topic} within {args.cmd_timeout}s"
        )

    msg = received["msg"]
    if msg is None:
        raise AssertionError("Command smoke failed: callback fired without a Twist payload")

    if int(round(msg.linear.x)) != 42 or int(round(msg.angular.z)) != -17:
        raise AssertionError(
            "Command smoke failed: published Twist does not match set_velocity input "
            f"(got linear.x={msg.linear.x}, angular.z={msg.angular.z})"
        )

    cmd_sub.unregister()
    motor.stop()
    motor.close()
    stream.stop()

    print("PASS: Topic smoke, API parity smoke, command smoke, and import smoke all passed.")


if __name__ == "__main__":
    main()
