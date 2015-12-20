#!/usr/bin/env python

import rospy
import actionlib_msgs.msg as al_msg
import actionlib
import move_base_msgs.msg as mb_msg

errors = 0

def callback(data):
	global errors
	for goal in data.status_list:
		if goal.status == 3:
			print("Goal has been found! Rejoice! Carrying on.")
			break
		elif goal.status==4:
			errors = errors+1
			if errors == 3:
				errors = 0
				print("Goal unreachable!")
				action_server = actionlib.SimpleActionClient('move_base', mb_msg.MoveBaseAction);
				action_server.wait_for_server()
				action_server.cancel_goal()
				break
				#kill
	# print data


def listener():

	rospy.init_node('goal_status_listener', anonymous=True)

	rospy.Subscriber("move_base/status", al_msg.GoalStatusArray, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()