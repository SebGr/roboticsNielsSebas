#!/usr/bin/env python

import rospy
import actionlib
from collections import deque
from geometry_msgs.msg import Quaternion,PoseStamped
from tf.transformations import quaternion_from_euler
from tf import TransformListener
import random
import move_base_msgs.msg as mb_msg
import nav_msgs.msg as nav_msg
import nav_msgs.srv as nav_srv
import geometry_msgs.msg as geo_msg
import numpy as np
import random

import locations_io as io

def convert_angle_to_quaternions(angle):
		q_angle = quaternion_from_euler(0, 0, float(angle), axes='sxyz')
		return Quaternion(*q_angle)

def create_goal(x, y, orientation=convert_angle_to_quaternions(random.uniform(0,360))):
	goal = mb_msg.MoveBaseGoal()
	goal.target_pose.pose.position.x = x  #Created random coordinates for the robot to go to
	goal.target_pose.pose.position.y = y #Created random coordinates for the robot to go to
	goal.target_pose.pose.orientation = orientation #Created random coordinates for the robot to go to
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
	return goal

def send_goal(goal, action_server):
	result = action_server.send_goal_and_wait(goal)


def neighbour(occupancyMap):
	for n in range(0,2):
		for m in range(0,2):
			if(occupancyMap[n,m]==-1):
				return True
	return False


def find_goals(occupancyMap, width, height):
	found = []

	potential_goals = zip(*np.where(occupancyMap==0))
	for n in potential_goals:
		x_value = n[0]
		y_value = n[1]
		if(neighbour(occupancyMap[x_value-1:x_value+1,y_value-1:y_value+1])==True):
			found.append(n) 
	return found

def transformation(occupancyMap, width, height):
	grid = np.reshape(occupancyMap, (width,height))
	return grid

def create_message(x,y,z,quaternion):
	msg = geo_msg.PoseStamped()
	msg.pose.position.x = x
	msg.pose.position.y = y
	msg.pose.position.z = z
	msg.pose.orientation = geo_msg.Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3]);
	return msg

def subscriber_callback(data):
	occupancyMap = transformation(data.data, data.info.width, data.info.height)
	way_points = find_goals(occupancyMap, data.info.width, data.info.height)
	result = random.choice(way_points)

	try:
		planMaker = rospy.ServiceProxy('move_base/make_plan', nav_srv.GetPlan)
		listener = TransformListener()
		listener.waitForTransform("base_link", "map", rospy.Time(), rospy.Duration(4.0))
		t = listener.getLatestCommonTime("base_link", "map")
		position, quaternion = listener.lookupTransform("base_link", "map", t)
		pos_x = position[0]
		pos_y = position[1]
		pos_z = position[2]
		goal_robot = create_goal((result[1]-2000)*data.info.resolution,(result[0]-2000)*data.info.resolution)
		#Make plan with 0.5 meter flexibility, from target pose and current pose (with same header)
		start_pose = create_message(pos_x,pos_y,pos_z,quaternion)

		plan = planMaker(
			start_pose,
			goal_robot.target_pose,
			0.5)
		action_server = actionlib.SimpleActionClient('move_base', mb_msg.MoveBaseAction);
		action_server.wait_for_server()
		send_goal(goal_robot, action_server)

	except rospy.ServiceException, e:
		print "plan service call failed: %s"%e

if __name__ == "__main__":
	rospy.init_node(name="goal_executioner")
	try:
		while(not rospy.is_shutdown()):
			mapGetter = rospy.ServiceProxy('dynamic_map', nav_srv.GetMap)
			newMap = mapGetter()
			subscriber_callback(newMap.map)
	except rospy.ServiceException, e:
		print "map service call failed: %s"%e
	#rospy.spin()