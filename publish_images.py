#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import glob

rospy.init_node('image_folder_publisher')

pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
bridge = CvBridge()

# CHANGE PATH IF NEEDED
image_paths = sorted(glob.glob('/home/maleeha/catkin_ws/init_data_20260426_115003/images/*.png'))

rate = rospy.Rate(10)

rospy.loginfo(f"Publishing {len(image_paths)} images...")

for img_path in image_paths:
    frame = cv2.imread(img_path)

    if frame is None:
        rospy.logwarn(f"Failed to load {img_path}")
        continue

    msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
    pub.publish(msg)

    rate.sleep()

rospy.loginfo("Finished publishing images.")
