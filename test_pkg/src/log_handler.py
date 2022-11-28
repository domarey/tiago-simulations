#!/usr/bin/env python
from fp_core_msgs.srv import GetPose, PowerUp, SetString
import json, socket, ssl
from std_srvs.srv import Trigger
import rospy

if __name__ == '__main__':
	rospy.init_node("dome_log_node")
	
	
	while True:
		input_val = int(input("Press 1 to log in, 2 to log out, any other to ESC:\n"))
		if input_val == 1:
			rospy.loginfo("Logging in")
			ip_address = "10.0.0.203"
			username = "domenico"
			password = "domeLio22"
			ssl_cert = "/home/domenico/catkin_ws/src/test_pkg/ssl/certificate.crt"
			
			# create request to send
			request = {
		    	"request": "authentication", 
				"action": "login", 
				"data": {
					"username": username, 
					"password": password,
					},
				}
			ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
			ssl_context.load_verify_locations(ssl_cert)
			ssl_context.check_hostname = False
			ssl_context.verify_mode = ssl.CERT_NONE

			with socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0) as sock:
				with ssl_context.wrap_socket(sock, server_hostname=ip_address) as ssock:
					# connect to myP authentication socket
					ssock.connect((ip_address, 5001))
					# send request (and end character)
					message = json.dumps(request) + "\x03"
					ssock.send(message.encode("UTF-8"))
					# receive answer (before end character)
					message = ssock.recv(1024).decode("UTF-8")
					answer = json.loads(message.split("\x03")[0])

			token = answer["response"]["data"]["token"]

			rospy.wait_for_service('/lio/core/register_user')
			
			try:
				get_token = rospy.ServiceProxy('/lio/core/register_user', SetString)
				res = get_token(token)
				if res.success:
					rospy.loginfo("Logged in successfully")
				else:
					rospy.logerr(res.message)
			except rospy.ServiceException as e:
				rospy.logerr("Service call failed: %s"%e)
		elif input_val == 2:
			rospy.loginfo("Logging out")

			rospy.wait_for_service('/lio/core/logout')
			
			try:
				res = rospy.ServiceProxy('/lio/core/logout', Trigger).call()
				if res.success:
					rospy.loginfo("Logged out successfully")
					break
				else:
					rospy.logerr(res.message)
			except rospy.ServiceException as e:
				rospy.logerr("Service call failed: %s"%e)
		else:
			break
