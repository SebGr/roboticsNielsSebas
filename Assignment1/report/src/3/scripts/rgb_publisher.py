#!/usr/bin/env python

import rospy
import actionlib
from sensor_msgs.msg import Image

from basics.srv import *
import basics.msg

def talker(grayscale, rgb, number_of_pixels):
	pub = rospy.Publisher('imagesAndPixelsum', basics.msg.Images, queue_size=1)
	msg = basics.msg.Images()
	msg.rgb = rgb
	msg.grayscale = grayscale
	msg.numberofpixels = number_of_pixels
	pub.publish(msg)

def call_to_count_pixels_server(grayscale, rgb):
	client = actionlib.SimpleActionClient('countPixels', basics.msg.countPixelsAction)
	client.wait_for_server()
	goal = basics.msg.countPixelsGoal(rgb, grayscale)
	client.send_goal(goal)
	client.wait_for_result()
	return client.get_result().numberofpixels

def call_to_rgb_to_gray_scale_server(image):
	rospy.wait_for_service('convert_rgb_to_gray_scale')
	try:
		convert_rgb_to_gray_scale = rospy.ServiceProxy(
			'convert_rgb_to_gray_scale', RGBtoGrayScale
		)
		resp = convert_rgb_to_gray_scale(image)
		return resp.grayscale
	except rospy.ServiceException, e:
		rospy.logerr("Service call failed: %s"%e)

def subscriber_callback(rgb_image):
	print 'callback'
	grayscale_image = call_to_rgb_to_gray_scale_server(rgb_image)
	number_of_pixels = call_to_count_pixels_server(grayscale_image, rgb_image)
	talker(grayscale_image, rgb_image, number_of_pixels)

def listener():
	rospy.Subscriber("/camera/rgb/image_raw", Image, subscriber_callback)	

if __name__ == "__main__":
	try:
		rospy.init_node(name='rgb_publisher')
	except ROSInitException:
		rospy.logerr("init_node failed; initialization/registration failed.")
	except ValueError:
		rospy.logerr("init_node failed; wrong arguments.") 
	listener()
	rospy.spin()
