#!/usr/bin/env python
from fp_core_msgs.srv import GetPose, PowerUp, SetString
from std_srvs.srv import Trigger
import socket, sqlite3
import json, socket, ssl, websocket, ujson
import rospy
import argparse


# define the robot ip address
IP = "10.0.0.203"

def on_message(msg):
    rospy.loginfo("Message received from LIO: %s" % msg)
    
def handler_request(obj):
    global ws
    try:
        rospy.loginfo("Sending data to LIO: %s", obj.value)
        ws.send(obj.value)
    
        return SetStringResponse(success = True)
    except Exception as e:
        return SetStringResponse(success = False, message = str(e))
    
    
def main():
	# create the tcp socket
	with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
	# connect to the serve
		client_socket.connect((IP, 38152))
		# receive a message, decode and split it by the end character
		for message in client_socket.recv(1024).decode("UTF-8").split("\x03"):
			# print the received message
			print(message)
	
	
if __name__ == "__main__":
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
		
	#rospy.wait_for_service(args.srv)
	#resp = rospy.ServiceProxy(args.srv, Token).call()

	ws = websocket.WebSocket(sslopt={"cert_reqs": ssl.CERT_NONE }, on_message = on_message)
	ws.connect("wss://192.168.100.110/socket?channel=DESKTOP")

	req = { "action": 'register_user',
	    "bridge": 'core',
	    "arguments": {
		"token": token
	    }}

	ws.send(ujson.dumps(req));

	name = 'ppm/domenico/test1'
	args = ''
	req = { "action": 'execute_script',
	    "bridge": 'core',
	    "arguments": {
		"script_name": name,
		"script_arguments": args
	    }}

	ws.send(ujson.dumps(req));
	rospy.loginfo("Logging out")

	rospy.wait_for_service('/lio/core/logout')
	
	res = rospy.ServiceProxy('/lio/core/logout', Trigger).call()
	if res.success:
		rospy.loginfo("Logged out successfully")
	else:
		rospy.logerr(res.message)	
	'''
	# Create a SQL connection to our SQLite database
	con = sqlite3.connect("/home/domenico/Downloads/robot_database.db")

	cur = con.cursor()

	res = cur.execute("PRAGMA table_info(poses)")
	#print(res.fetchall())
	paths = res.execute("SELECT name, actuator_angles, pose_data FROM poses")
	#print(paths.fetchall())
	
	paths = res.execute("SELECT * FROM poses WHERE name LIKE '%home%'")
	#print(paths.fetchall())
	for path in paths.description:
		print(path[0])
		
	query = res.execute("SELECT name, actuator_angles, pose_data FROM poses WHERE name LIKE '%home%'")
	print(query.fetchall())
	# Be sure to close the connection
	con.close()
	'''
