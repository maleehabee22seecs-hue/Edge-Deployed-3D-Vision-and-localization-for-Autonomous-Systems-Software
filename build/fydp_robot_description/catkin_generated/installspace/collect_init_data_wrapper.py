#!/usr/bin/env python3
"""Run collect_init_data.py against ROS shims without modifying core script code."""

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

    control_pkg = types.ModuleType("control")
    motor_module = types.ModuleType("control.motor_controller")
    motor_module.MotorController = RosMotorController
    control_pkg.motor_controller = motor_module

    sys.modules["control"] = control_pkg
    sys.modules["control.motor_controller"] = motor_module


def _append_collect_paths():
    ws = _workspace_root()
    vision_dir = os.path.join(ws, "src", "fydp", "src", "vision")
    control_parent_dir = os.path.join(ws, "src", "fydp", "src")

    if vision_dir not in sys.path:
        sys.path.insert(0, vision_dir)
    if control_parent_dir not in sys.path:
        sys.path.insert(0, control_parent_dir)


def main():
    # Drop ROS remap arguments before delegating to argparse in collect_init_data.
    sys.argv = rospy.myargv(argv=sys.argv)
    help_only = any(arg in ("-h", "--help") for arg in sys.argv[1:])
    if not help_only:
        rospy.init_node("collect_init_data_wrapper", anonymous=True, disable_signals=True)

    os.environ.setdefault("FYDP_USE_ROS_SHIMS", "1")
    os.environ.setdefault("FYDP_DATASET_OUTPUT_DIR", _workspace_root())

    _inject_shim_modules()
    _append_collect_paths()

    import collect_init_data

    collect_init_data.main()


if __name__ == "__main__":
    main()