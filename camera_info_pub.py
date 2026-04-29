#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CameraInfo

rospy.init_node("camera_info_pub")

pub = rospy.Publisher("/camera/camera_info", CameraInfo, queue_size=10)

info = CameraInfo()

# Dummy pinhole calibration (good enough to START ORB-SLAM)
info.width = 640
info.height = 480

info.K = [525.0, 0.0, 320.0,
          0.0, 525.0, 240.0,
          0.0, 0.0, 1.0]

info.D = [0, 0, 0, 0, 0]
info.R = [1, 0, 0,
          0, 1, 0,
          0, 0, 1]

info.P = [525.0, 0.0, 320.0, 0.0,
          0.0, 525.0, 240.0, 0.0,
          0.0, 0.0, 1.0, 0.0]

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    info.header.stamp = rospy.Time.now()
    pub.publish(info)
    rate.sleep()

