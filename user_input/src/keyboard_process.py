#!/usr/bin/python


import rospy
import numpy as np
import keyboard
from numpy.linalg import inv

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from std_msgs.msg import ColorRGBA, Float32, Bool, Int32
from user_input.msg import Velocity, JoyCmd
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math

class keyboard_obj(object):

	def __init__(self):
		rospy.init_node('keyboard', anonymous=True)
		self.joyconnPub= rospy.Publisher('/joy/connected', Bool, queue_size = 1)
		self.joycmdPub = rospy.Publisher('/joy/cmd', JoyCmd, queue_size = 10)

		r = rospy.Rate(60)
		self.cmd_msg = JoyCmd()
		self.conn_msg = Bool()
		self.conn_msg.data = True

		while not rospy.is_shutdown():
			self.cmd_msg = self.detectCmds()
			# Publish at a frequency of 60 Hz
			self.joyconnPub.publish(self.conn_msg)
			self.joycmdPub.publish(self.cmd_msg)
			r.sleep()

	def detectCmds(self):
		msg = JoyCmd()
		mult = 1.0/3.0
		#Detect forward and backward keys assigned to 'w' and 's'
		if keyboard.is_pressed('w'):
			msg.axis1 =  1.0
		if keyboard.is_pressed('s'):
			msg.axis1 = msg.axis1 - 1.0

		#Detect left and right keys 'a' and 'd'
		if keyboard.is_pressed('a'):
			msg.axis2 =  1.0
		if keyboard.is_pressed('d'):
			msg.axis2 = msg.axis2 - 1.0

		#Detect rotation keys 'q' for counterclockwise, 'e' for clockwise

		if keyboard.is_pressed('q'):
			msg.axis3 =  1.0
		if keyboard.is_pressed('e'):
			msg.axis3 = msg.axis3 - 1.0

		#Detect keys for multipliers

		if keyboard.is_pressed('alt'):
			mult = 2.0/3.0
		if keyboard.is_pressed('shift'):
			mult = 1.0

		msg.axis1 = msg.axis1*mult
		msg.axis2 = msg.axis2*mult
		msg.axis3 = msg.axis3*mult
		return msg



if __name__=='__main__':
	node = keyboard_obj()
	


# var joy_connected_topic = new ROSLIB.Topic({
#     ros: ros,
#     name: '/joy/connected',
#     messageType: 'std_msgs/Bool'
# })
# var joy_cmd_topic = new ROSLIB.Topic({
#     ros: ros,
#     name: '/joy/cmd',
#     messageType: 'user_input/JoyCmd'
# })