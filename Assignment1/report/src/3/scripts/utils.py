#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy

def convert_image_ros_to_cv(bridge, data, type="bgr8"):
	try:
		cv_image = bridge.imgmsg_to_cv2(data, type)
		return cv_image
	except CvBridgeError, error:
		raise error
	
	
def convert_image_cv_to_ros(bridge, data, type="bgr8"):
	try:
		ros_image = bridge.cv2_to_imgmsg(data, type)
	except CvBridgeError, error:
		rospy.logerr(error)
	return ros_image
