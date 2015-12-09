#!/usr/bin/env python
import rospy
import actionlib
from collections import deque
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

import move_base_msgs.msg as mb_msg


import locations_io as io

location_files = {
	'static' 	: '/home/student/sudo/brain/data/locations/locations.dat',
	'dynamic' 	: '/home/student/sudo/brain/data/locations/locations-dynamic.dat'
}

class Navigator(object):

	def __init__(self, way_points):
		self.way_points = way_points

	def convert_angle_to_quaternions(self, angle):
		q_angle = quaternion_from_euler(0, 0, float(angle), axes='sxyz')
		return Quaternion(*q_angle)

	def create_goal(self, way_point):
		goal = mb_msg.MoveBaseGoal()
		goal.target_pose.pose.position.x = float(way_point.get('x'))
		goal.target_pose.pose.position.y = float(way_point.get('y'))
		goal.target_pose.pose.orientation = self.convert_angle_to_quaternions(way_point.get('angle'))

		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		print way_point
		print goal
		return goal

	def send_goal(self, goal):
		result = self.action_server.send_goal_and_wait(goal)
		# Handle result
		print result

	def spin(self):
		try:
			rospy.init_node('navigator', anonymous=False)
		except rospy.ROSInitException, e:
			rospy.logerr("Could not init navigator node:%s"%e)

		self.action_server = actionlib.SimpleActionClient('move_base', mb_msg.MoveBaseAction);
		self.action_server.wait_for_server()

		while(not rospy.is_shutdown() and self.way_points):
			self.send_goal(
				self.create_goal(
					self.way_points.popleft()
				)
			)
		rospy.spin()			

if __name__ == "__main__":
	way_points = io.read_locations_from_file(
		location_files.get('static')
	)
	rospy.loginfo('Launching the navigator')
	navigator = Navigator(way_points)
	navigator.spin()