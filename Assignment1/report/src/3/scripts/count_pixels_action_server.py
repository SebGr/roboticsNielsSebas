#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import actionlib

import utils
import basics.msg

class CountPixels:

	def count_pixels_in_ros_image(self, ros_image, type):
		try:
			cv_image = utils.convert_image_ros_to_cv(
				self.bridge, ros_image, type)
			height, width = cv_image.shape[:2]
			return height * width
		except CvBridgeError:
			#self.server.set_aborted()
			return -1;
	
	def __init__(self):
		self.server = actionlib.SimpleActionServer(
			'countPixels',
			basics.msg.countPixelsAction,
			execute_cb=self.execute,
			auto_start = False
		)
		self.bridge = CvBridge()
		self.server.start()

	def execute(self, goal):
		rgb_pixels = self.count_pixels_in_ros_image(goal.rgb, type="bgr8")

		gray_pixels = self.count_pixels_in_ros_image(goal.grayscale, type="8UC1")

		pixels = rgb_pixels + gray_pixels
		#rospy.loginfo(pixels)
		result = basics.msg.countPixelsResult()
		result.numberofpixels = pixels
		self.server.set_succeeded(result)

if __name__ == "__main__":
	rospy.init_node(name='countPixelsServer')
	server = CountPixels()
	rospy.spin()
