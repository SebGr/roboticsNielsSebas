#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import utils
from basics.msg import Images

class Image_Subscriber(object):


	def __init__(self):
		self.bridge = CvBridge()

	def spin(self):
		rospy.init_node('listener', anonymous=True)
		rospy.Subscriber("imagesAndPixelsum", Images, self.show_message)
		rospy.spin()			

	def show_image(self, image, window, type='bgr8'):
		cv_image = utils.convert_image_ros_to_cv(
					self.bridge, image, type)
		cv2.imshow(window, cv_image)


	def show_message(self, msg):
		self.show_image(msg.rgb, 'RGB')
		self.show_image(msg.grayscale, 'grayscale' ,'8UC1')
		rospy.loginfo(msg.numberofpixels)
		cv2.waitKey(1)

if __name__ == "__main__":
	sub = Image_Subscriber()
	sub.spin()