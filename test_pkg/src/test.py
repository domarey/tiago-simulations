#!/usr/bin/env python
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
import json, socket, ssl
from std_srvs.srv import Trigger
import rospy, actionlib, numpy

import sqlite3

def active_cb():
    rospy.loginfo("Lio is still moving...")
#
#def feedback_cb(feedback):
#    rospy.loginfo("Current location: "+str(feedback))

def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Goal reached")
    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    if status == 4:
        rospy.loginfo("Goal aborted")

class BaseHandler(object):
    def __init__(self):
        rospy.loginfo("Initializing BaseHandler...")
        self.ac = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Connecting to /move_base ...")
        # If more than 10s pass by, we crash
        if not self.ac.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Could not connect to /move_base!")
            exit(0)
        rospy.loginfo("Connected!")
        
    def go_to(self, x, y, orientation):
    	goal = MoveBaseGoal()
    	goal.target_pose.header.frame_id = "map"
    	
    	goal.target_pose.header.stamp = rospy.Time.now()
    	goal.target_pose.pose.position.x = x
    	goal.target_pose.pose.position.y = y
    	
    	quaternion = quaternion_from_euler(0, 0,  numpy.deg2rad(orientation))
    	goal.target_pose.pose.orientation.x = quaternion[0]
    	goal.target_pose.pose.orientation.y = quaternion[1]
    	goal.target_pose.pose.orientation.z = quaternion[2]
    	goal.target_pose.pose.orientation.w = quaternion[3]
    	self.ac.send_goal(goal, done_cb, active_cb)
    	rospy.loginfo("Lio will move to position (" + str(x) + "," + str(y) + ") with orientation " + str(orientation))
    	finished  = self.ac.wait_for_result()

if __name__ == '__main__':
	rospy.init_node("test_node")
	
	


	rospy.loginfo("Domenico is testing")
	rospy.loginfo("Going to starting position...")
	
	rospy.wait_for_service('/lio/core/tool_pick')
			
	try:
		res = rospy.ServiceProxy('/lio/core/tool_pick', Trigger).call()
		if res.success:
			rospy.loginfo("Closed gripper successfully")
		else:
			rospy.logerr(res.message)
	except rospy.ServiceException as e:
		rospy.logerr("Service call failed: %s"%e)
	
	# To change the position of the robot's base
	base_handler = BaseHandler()
	rospy.loginfo("Going to first position")
	base_handler.go_to(0, 0, -90.0)
	
	'''
	to store:
	pppm storage poses
	
	javascript to generate the path from the positions
	ppm/flexgui_interface
	action play path
	
	core execute script
	
	to execute a script
	open a websocket channel to the robot and then 
	dropbox tcp protocol manual (similar to the websocket)
	'''
	
	
'''
	move_to_pose("attention", velocity=None, acceleration=None, block=True, offset=None, mode='deprecated', profile='trapezoidal')

	log_message("Going to second position")
	move_to_pose("ppm/drive", velocity=None, acceleration=None, block=True, offset=None, mode='deprecated', profile='trapezoidal')

	voice.say('Ciao come stai, Domenico?', language_code=None, voice_id=None, volume=50, block=True)

	#home_interface.show_video(video_name, quick=False, controls=False, background='')


	move_joint(ALL_KIN_JOINTS, 0, velocity=None, acceleration=None, block=True, relative=False, profile='trapezoidal')


	orientation = [180,0,90]

	if(is_reachable(0, -400, 950, orientation=orientation)):
		move_tool(0, -400, 950, orientation=orientation, velocity=None, acceleration=None, block=True, relative=False, frame='base', mode='deprecated', profile='trapezoidal')

	else:
		print("Not reachable!")
		
	print(mobile_platform.get_nearby_positions(max_distance=10, return_closest=False))

	log_message("Going to another position")
	move_to_pose("ppm/drive", velocity=None, acceleration=None, block=True, offset=None, mode='deprecated', profile='trapezoidal')
		
	voice.say('Going to the first location', language_code=None, voice_id=None, volume=50, block=True)
	mobile_platform.move_platform(0, 0, 0, block=True, relative=False, goal_importance=None, linear_speed=None, angular_speed=None)

	voice.say('Going to the second location', language_code=None, voice_id=None, volume=50, block=True)
	mobile_platform.move_platform(0, -0.5 , -45, block=True, relative=False, goal_importance=None, linear_speed=None, angular_speed=None)


	while not service_robot.is_robot_docked():
		voice.say('Docking', language_code=None, voice_id=None, volume=50, block=True)
		mobile_platform.move_to_position("charging_station", block=True, goal_importance=None, linear_speed=None, angular_speed=None)
		service_robot.docking(trig=True)

	voice.say('Charging', language_code=None, voice_id=None, volume=50, block=True)

	log_message("---------- End of Domenico's test ----------")
'''
