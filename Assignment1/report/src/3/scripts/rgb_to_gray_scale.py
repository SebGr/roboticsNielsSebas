#!/usr/bin/env python
import rospy
from basics.srv import * 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from utils import *
import cv2
import numpy

import utils

def rgb_to_gray_scale(req):
	bridge = CvBridge()
	cv_image = convert_image_ros_to_cv(bridge, req.rgb)
	cv_gray_scale = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	ros_gray_scale_image_msg = convert_image_cv_to_ros(bridge, cv_gray_scale, type="8UC1")
	return RGBtoGrayScaleResponse(ros_gray_scale_image_msg)

def convert_rgb_to_gray_scale_server():
	try:	
		rospy.init_node('convert_rgb_to_gray_scale_server')
	except rospy.exceptions.ROSInitException:
		rospy.logerr('init_node failed; initialization/registration failed.')
	except ValueError:
		rospy.logerr("init_node failed; wrong arguments.") 
	
	rospy.Service(
		name='convert_rgb_to_gray_scale', 
		service_class=RGBtoGrayScale, 
		handler=rgb_to_gray_scale
	)	
	rospy.spin()


if __name__ == "__main__":
	convert_rgb_to_gray_scale_server()
