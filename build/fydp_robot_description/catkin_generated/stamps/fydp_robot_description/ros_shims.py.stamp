#!/usr/bin/env python3
"""ROS shim interfaces compatible with the hardware classes used by SLAM scripts.

This module mirrors the subset of APIs used by:
- collect_init_data.py
- live_map_growth.py

Camera shim:
- Subscribes to /camera/image_raw
- Provides capture() with grayscale/color behavior matching CameraStream

Motor shim:
- Subscribes to /odom and publishes /cmd_vel
- Returns odometry pose as (x_cm, y_cm, yaw_rad)
"""

import math
import os
import threading
from typing import Optional, Tuple

import cv2
import numpy as np
import yaml

try:
    import rospy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo
    from sensor_msgs.msg import Image
except Exception as exc:
    rospy = None
    CvBridge = None
    Twist = None
    Odometry = None
    CameraInfo = None
    Image = None
    _ROS_IMPORT_ERROR = exc
else:
    _ROS_IMPORT_ERROR = None


def _require_ros():
    if _ROS_IMPORT_ERROR is not None:
        raise RuntimeError(
            "ROS Python dependencies are unavailable. "
            "Source your ROS setup and ensure rospy, cv_bridge, sensor_msgs, nav_msgs, and geometry_msgs are installed."
        ) from _ROS_IMPORT_ERROR


def _ensure_ros_node(default_name: str):
    _require_ros()
    if not rospy.core.is_initialized():
        rospy.init_node(default_name, anonymous=True, disable_signals=True)


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class RosCameraStream:
    """ROS-backed camera stream with CameraStream-compatible API."""

    def __init__(
        self,
        calib_file: Optional[str] = None,
        size: Tuple[int, int] = (640, 480),
        rotate_180: bool = True,
        full_fov: bool = True,
        sports_mode: bool = False,
        sports_max_exposure_us: int = 12000,
        sports_frame_duration_us: int = 16666,
        image_topic: str = "/camera/image_raw",
        camera_info_topic: str = "/camera/camera_info",
        wait_for_image_timeout: float = 8.0,
        wait_for_camera_info_timeout: float = 3.0,
    ):
        _ensure_ros_node("ros_camera_stream")

        self.rotate_180 = bool(rotate_180)
        self.full_fov = bool(full_fov)
        self.sports_mode = bool(sports_mode)
        self.sports_max_exposure_us = int(sports_max_exposure_us)
        self.sports_frame_duration_us = int(sports_frame_duration_us)

        self._bridge = CvBridge()
        self._lock = threading.RLock()
        self._frame_event = threading.Event()
        self._stopped = False
        self._latest_rgb = None
        self._latest_stamp = None
        self._image_topic = image_topic
        self._camera_info_topic = camera_info_topic
        self._wait_for_image_timeout = float(wait_for_image_timeout)
        self._wait_for_camera_info_timeout = float(wait_for_camera_info_timeout)
        self._camera_info_event = threading.Event()
        self._warned_intrinsics_override = False

        self.image_size = (int(size[0]), int(size[1]))
        self.undistorted_image_size = None
        self.raw_K = None
        self.K = None
        self.dist = None

        self._roi = None
        self._map1 = None
        self._map2 = None
        self._maps_size = None

        if calib_file:
            self._load_calibration(calib_file)
            self._ensure_undistort_maps_locked(self.image_size)

        self._camera_info_sub = rospy.Subscriber(
            self._camera_info_topic,
            CameraInfo,
            self._camera_info_callback,
            queue_size=1,
        )
        self._image_sub = rospy.Subscriber(self._image_topic, Image, self._image_callback, queue_size=1)

        if self._wait_for_camera_info_timeout > 0.0:
            self._camera_info_event.wait(self._wait_for_camera_info_timeout)

    def _load_calibration(self, calib_file: str):
        if not os.path.exists(calib_file):
            raise FileNotFoundError(f"Calibration file not found: {calib_file}")

        with open(calib_file, "r", encoding="utf-8") as f:
            calib = yaml.safe_load(f)

        if not isinstance(calib, dict):
            raise ValueError(f"Calibration file did not parse as a mapping: {calib_file}")

        try:
            K_data = np.asarray(calib["camera_matrix"]["data"], dtype=np.float64).reshape(-1)
            dist_data = np.asarray(calib["distortion_coefficients"]["data"], dtype=np.float64).reshape(-1)
        except Exception as exc:
            raise ValueError(
                "Calibration YAML must contain camera_matrix.data and distortion_coefficients.data"
            ) from exc

        if K_data.size != 9:
            raise ValueError(f"camera_matrix.data must contain 9 values, got {K_data.size}")
        if dist_data.size not in (4, 5, 8, 12, 14):
            raise ValueError(
                "distortion_coefficients.data length must be one of 4, 5, 8, 12, or 14"
            )

        self.raw_K = K_data.reshape(3, 3)
        self.dist = dist_data

    def _ensure_undistort_maps_locked(self, image_size: Tuple[int, int]):
        if self.raw_K is None or self.dist is None:
            return

        if self._maps_size == image_size and self._map1 is not None and self._map2 is not None:
            return

        self.K, roi = cv2.getOptimalNewCameraMatrix(self.raw_K, self.dist, image_size, 0, image_size)
        self._roi = tuple(int(v) for v in roi)
        self.undistorted_image_size = (self._roi[2], self._roi[3])
        self._map1, self._map2 = cv2.initUndistortRectifyMap(
            self.raw_K, self.dist, None, self.K, image_size, cv2.CV_16SC2
        )
        self._maps_size = image_size

    def _camera_info_callback(self, msg: CameraInfo):
        K_vals = np.asarray(msg.K, dtype=np.float64).reshape(-1)
        if K_vals.size != 9:
            return
        if np.allclose(K_vals, 0.0):
            return

        dist_vals = np.asarray(msg.D, dtype=np.float64).reshape(-1)
        if dist_vals.size == 0:
            dist_vals = np.zeros(5, dtype=np.float64)
        elif dist_vals.size not in (4, 5, 8, 12, 14):
            if dist_vals.size > 5:
                dist_vals = dist_vals[:5]
            else:
                dist_vals = np.pad(dist_vals, (0, 5 - dist_vals.size), mode="constant")

        width = int(msg.width) if int(msg.width) > 0 else self.image_size[0]
        height = int(msg.height) if int(msg.height) > 0 else self.image_size[1]

        with self._lock:
            had_intrinsics = self.raw_K is not None
            self.raw_K = K_vals.reshape(3, 3)
            self.dist = dist_vals
            self.image_size = (width, height)
            self._ensure_undistort_maps_locked(self.image_size)

        if had_intrinsics and not self._warned_intrinsics_override:
            rospy.loginfo_once(
                f"RosCameraStream: using intrinsics from {self._camera_info_topic} "
                "(overriding calibration file values)"
            )
            self._warned_intrinsics_override = True

        self._camera_info_event.set()

    def _to_rgb(self, frame: np.ndarray, encoding: str) -> np.ndarray:
        if frame.ndim == 2:
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)

        if frame.ndim != 3:
            raise ValueError(f"Unsupported image shape from ROS topic: {frame.shape}")

        channels = frame.shape[2]
        enc = (encoding or "").lower()

        if channels == 3:
            if enc.startswith("rgb"):
                return frame
            if enc.startswith("bgr") or enc == "":
                return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            return frame

        if channels == 4:
            if enc.startswith("rgba"):
                return cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)

        raise ValueError(f"Unsupported channel count from ROS topic: {channels}")

    def _image_callback(self, msg: Image):
        if self._stopped:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            rgb = self._to_rgb(frame, msg.encoding)
            if self.rotate_180:
                rgb = cv2.rotate(rgb, cv2.ROTATE_180)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, f"RosCameraStream image conversion failed: {exc}")
            return

        with self._lock:
            h, w = rgb.shape[:2]
            self.image_size = (w, h)
            self._latest_rgb = rgb
            self._latest_stamp = msg.header.stamp
            self._ensure_undistort_maps_locked(self.image_size)
            self._frame_event.set()

    def wait_for_image(self, timeout: float = 8.0) -> bool:
        return self._frame_event.wait(float(timeout))

    def capture(self, undistort: bool = True, grayscale: bool = True):
        if self._stopped:
            raise RuntimeError("RosCameraStream has been stopped.")

        if not self._frame_event.wait(self._wait_for_image_timeout):
            raise RuntimeError(
                f"Timed out waiting for image on topic {self._image_topic} after {self._wait_for_image_timeout:.2f}s"
            )

        with self._lock:
            if self._latest_rgb is None:
                raise RuntimeError("No image available from ROS topic.")
            frame = self._latest_rgb.copy()
            roi = self._roi
            map1 = self._map1
            map2 = self._map2

        if undistort and map1 is not None and map2 is not None:
            frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
            if roi is not None:
                x, y, w_roi, h_roi = roi
                frame = frame[y : y + h_roi, x : x + w_roi]

        if grayscale and frame.ndim == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        return frame

    def stop(self):
        self._stopped = True
        if getattr(self, "_camera_info_sub", None) is not None:
            self._camera_info_sub.unregister()
            self._camera_info_sub = None
        if getattr(self, "_image_sub", None) is not None:
            self._image_sub.unregister()
            self._image_sub = None


class RosMotorController:
    """ROS-backed motor controller with MotorController-compatible API."""

    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baud: int = 115200,
        update_rate: int = 20,
        cmd_topic: str = "/cmd_vel",
        odom_topic: str = "/odom",
    ):
        _ensure_ros_node("ros_motor_controller")

        self.port = port
        self.baud = int(baud)
        self.update_rate = int(update_rate)

        self._cmd_topic = cmd_topic
        self._odom_topic = odom_topic

        self._lock = threading.RLock()
        self._odom_event = threading.Event()
        self._connected = False

        self.last_linear = 0
        self.last_angular = 0
        self.is_stopped = True

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self._cmd_pub = None
        self._odom_sub = None

    def connect(self):
        if self._connected:
            return

        self._cmd_pub = rospy.Publisher(self._cmd_topic, Twist, queue_size=10)
        self._odom_sub = rospy.Subscriber(self._odom_topic, Odometry, self._odom_callback, queue_size=10)
        self._connected = True

    def _odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = _quat_to_yaw(ori.x, ori.y, ori.z, ori.w)

        with self._lock:
            # Keep yaw in radians and convert x/y meters -> centimeters.
            self.x = float(pos.x) * 100.0
            self.y = float(pos.y) * 100.0
            self.theta = float(yaw)
            self._odom_event.set()

    def wait_for_odom(self, timeout: float = 2.0) -> bool:
        return self._odom_event.wait(float(timeout))

    def get_pose(self):
        with self._lock:
            return (self.x, self.y, self.theta)

    def set_velocity(self, linear, angular=0):
        linear = int(max(min(linear, 255), -255))
        angular = int(max(min(angular, 255), -255))

        with self._lock:
            self.last_linear = linear
            self.last_angular = angular
            self.is_stopped = linear == 0 and angular == 0

        if self._cmd_pub is None:
            return

        twist = Twist()
        twist.linear.x = float(linear)/10
        twist.angular.z = float(angular)/10
        self._cmd_pub.publish(twist)

    def stop(self):
        self.set_velocity(0, 0)

    def reset_pose(self):
        with self._lock:
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0

    def keyboard_control_non_blocking(self, linear_speed=120, angular_speed=120):
        import select
        import sys
        import termios
        import tty

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        def read_and_apply():
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch == "\x1b":
                    ch2 = sys.stdin.read(1)
                    if ch2 == "[":
                        ch3 = sys.stdin.read(1)
                        if ch3 == "A":
                            ch = "w"
                        elif ch3 == "B":
                            ch = "s"
                        elif ch3 == "C":
                            ch = "d"
                        elif ch3 == "D":
                            ch = "a"
                    else:
                        ch = "q"

                if ch.lower() == "w":
                    self.set_velocity(linear_speed, 0)
                elif ch.lower() == "s":
                    self.set_velocity(-linear_speed, 0)
                elif ch.lower() == "a":
                    self.set_velocity(0, angular_speed)
                elif ch.lower() == "d":
                    self.set_velocity(0, -angular_speed)
                elif ch == " ":
                    self.stop()
                elif ch.lower() == "q":
                    return False
            return True

        def cleanup():
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            self.stop()

        return read_and_apply, cleanup

    def close(self):
        self.stop()
        if self._odom_sub is not None:
            self._odom_sub.unregister()
            self._odom_sub = None
        self._cmd_pub = None
        self._connected = False


# Optional aliases for drop-in replacement when swapping imports.
CameraStream = RosCameraStream
MotorController = RosMotorController
