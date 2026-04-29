#!/usr/bin/env python3
"""Run live_map_growth.py against ROS shims without modifying core script code."""

import os
import sys
import types
import importlib.util

import rospy


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)


def _workspace_root():
    return os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))


def _load_ros_shims_module():
    shim_path = os.path.join(SCRIPT_DIR, "ros_shims.py")
    spec = importlib.util.spec_from_file_location("fydp_ros_shims", shim_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to create import spec for shim module: {shim_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _inject_shim_modules():
    shim_module = _load_ros_shims_module()
    RosCameraStream = shim_module.RosCameraStream
    RosMotorController = shim_module.RosMotorController

    camera_module = types.ModuleType("camera_stream")
    camera_module.CameraStream = RosCameraStream
    sys.modules["camera_stream"] = camera_module

    # Create control package with shims
    control_pkg = types.ModuleType("control")
    motor_module = types.ModuleType("control.motor_controller")
    motor_module.MotorController = RosMotorController
    control_pkg.motor_controller = motor_module
    sys.modules["control"] = control_pkg
    sys.modules["control.motor_controller"] = motor_module

    # Load real navigator module into control package
    ws = _workspace_root()
    navigator_path = os.path.join(ws, "src", "fydp", "src", "control", "navigator.py")
    navigator_spec = importlib.util.spec_from_file_location("control.navigator", navigator_path)
    if navigator_spec and navigator_spec.loader:
        navigator_module = importlib.util.module_from_spec(navigator_spec)
        navigator_spec.loader.exec_module(navigator_module)
        control_pkg.navigator = navigator_module
        sys.modules["control.navigator"] = navigator_module


def _append_live_map_paths():
    ws = _workspace_root()
    vision_dir = os.path.join(ws, "src", "fydp", "src", "vision")
    control_parent_dir = os.path.join(ws, "src", "fydp", "src")

    if vision_dir not in sys.path:
        sys.path.insert(0, vision_dir)
    if control_parent_dir not in sys.path:
        sys.path.insert(0, control_parent_dir)


def main():
    rospy.init_node("live_map_growth_wrapper", anonymous=True, disable_signals=True)

    # Drop ROS remap arguments before delegating to argparse in live_map_growth.
    sys.argv = rospy.myargv(argv=sys.argv)
    os.environ.setdefault("FYDP_USE_ROS_SHIMS", "1")

    _inject_shim_modules()
    _append_live_map_paths()

    import live_map_growth

    live_map_growth.main()


if __name__ == "__main__":
    main()