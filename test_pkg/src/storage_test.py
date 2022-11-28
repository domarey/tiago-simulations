#!/usr/bin/env python
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
import json, socket, ssl
from std_srvs.srv import Trigger
import rospy, actionlib, numpy

import sqlite3


if __name__ == '__main__':
	rospy.init_node('storage_test_node')
	rospy.loginfo("Testing the storage")
	
	rospy.wait_for_service('/ppm/storage/get_pose')
			
	try:
		res = rospy.ServiceProxy('/lio/core/tool_pick', Trigger).call()
		if res.success:
			rospy.loginfo("Closed gripper successfully")
		else:
			rospy.logerr(res.message)
	except rospy.ServiceException as e:
		rospy.logerr("Service call failed: %s"%e)
