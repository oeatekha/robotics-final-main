#!/usr/bin/python
"""
Controlling both odrives
"""

import time
import math
import fibre
import serial
import struct
import signal
import sys
import pdb
import numpy 
from user_input.msg import Velocity, JoyCmd
import rospy 
from std_msgs.msg import ColorRGBA, Float32, Bool, Int32
from odrive_driver import OdrivePython
import numpy as np
from numpy import tan

MAX_ROBOT_VEL = 3 #m/s
MAX_ROBOT_ROT_VEL = 2 #radians/s
LINEAR = 0
ROTATION = 1
alpha = np.pi/4 # degrees of mechanum wheel offset angle
LENGTH = .2863
WIDTH = .225
CNT_PER_REV = 400000
class OdriveMaster(object):
	def __init__(self, odrive_list, axis_list):
		rospy.init_node('joybrain', anonymous=True)
		rospy.Subscriber('/joy/connected', Bool, self.joyconnection_cb)
		rospy.Subscriber('/joy/cmd', JoyCmd, self.velcmd_cb)
		self.vel_pub = rospy.Publisher('/robot/velocity/measured', Velocity, queue_size = 10)
		self.odrive_pub = rospy.Publisher('/robot/ready/', Bool, queue_size = 1)
		self.browser_pub = rospy.Publisher('/cmd2browser', Int32, queue_size = 1)
		self.odrives = []
		self.driveMode = LINEAR
		self.axis_list = axis_list
		self.axes = []
		self.wheelR = 0.1524 # wheel radius
		self.baseLength = LENGTH/2 #length of robot base, wheel to wheel
		self.baseWidth = WIDTH/2 #width of robot base, wheel to wheel
		self.connected = False #
		self.last_joy_cmd = None
		tana = np.tan(alpha)
		L1 = self.baseLength
		L1tana = self.baseLength*np.tan(alpha)
		L2 = self.baseWidth
		L1pL2 = L1 + L2
		#IK Jacobian Inverse
		self.Ji = 1./self.wheelR * np.array([[1., 1./tana, -(L1tana + L2)/tana], \
						[1., -1./tana, (L1tana + L2)/tana], \
						[-1., -1./tana, -(L1tana + L2)/tana], \
						[-1., 1./tana, (L1tana + L2)/tana]])
		#FK Jacobian
		self.J = self.wheelR/4 * np.array([[1., 1., 1., 1.], \
						[1., -1., -1., 1.], \
						[1./L1pL2, 1./L1pL2, 1./L1pL2, 1./L1pL2]])

		for ssn in odrive_list: 
			# ssn's are strings
			# list is: front, back
			self.odrives.append(OdrivePython(ssn, init_arg = False))

		for i in range(len(self.odrives)):
			# axis list is 0s and 1s
			# list is: left front, right front, left back, right back
			self.axes.append(self.odrives[i].axes[axis_list[2*(i)]])
			self.axes.append(self.odrives[i].axes[axis_list[2*(i)+1]])

		self.vel_msg = Velocity()
		self.rdy_msg = Bool()
		self.rdy_msg.data = True
		self.int_msg = Int32()
		self.int_msg.data = 0
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			# Publish at a frequency of 100 Hz
			# if not self.connected:
			# 	print('not connected')
			# 	self.last_joy_cmd.axis1 = 0
			# 	self.last_joy_cmd.axis2 = 0
			# 	self.last_joy_cmd.axis3 = 0
			if self.last_joy_cmd is not None:
				self.UpdateMotorVel()
			self.odrive_pub.publish(self.rdy_msg)
			self.browser_pub.publish(self.int_msg)
			self.vel_pub.publish(self.vel_msg)
			r.sleep()

	def joyconnection_cb(self, msg):
		#Makes sure a joystick is connected
		# print(self.connected)
		self.connected = msg.data

	def velcmd_cb(self, msg):
		#takes joystick values and publishes velocities
		print(msg)
		self.last_joy_cmd = msg

	def UpdateMotorVel(self, cmd = None):
		cmd = self.last_joy_cmd
		if cmd is not None:
			motorCmd = self.convertCmd(cmd)
			motorCmd[0] = -1*motorCmd[0]
			motorCmd[2] = -1*motorCmd[2]
			for i in range(2):
				self.odrives[i].VelMove(vel_setpt = self.rad2cnt(motorCmd[2*i]), axis_num = self.axis_list[2*i])
				self.odrives[i].VelMove(vel_setpt = self.rad2cnt(motorCmd[2*i+1]), axis_num = self.axis_list[2*i+1])

	def convertMotorVel(self):
		motor_vels = []
		for i in range(len(self.odrives)):
			axis = self.odrives[i].axes[self.axis_list[i]]
			motor_vels.append(axis.encoder.vel_estimate)

		wheelSpeeds = motor_vel*self.wheelR
		return self.fk(wheelSpeeds)	 #Center of robot velocity 

	def convertCmd(self, msg):
		#Convert joystick command into motor wheel velocity commands

		#Find magnitude
		mag = np.sqrt(msg.axis1 ** 2 + msg.axis2 ** 2)
		angle = np.arctan2(msg.axis1,msg.axis2)
		rot = self.remap(-msg.axis3, rotational = True)

		#Remap values to be based on maximum robot velocity
		vel = self.remap(mag)
		# print(vel)
		vx = vel*np.cos(angle)
		vy = vel*np.sin(angle)
		cmd = [[vx], [vy], [rot]]
		motorCmd = self.ik(cmd)
		self.lastbtn = msg.btn1
		return motorCmd

	def fk(self, speeds):
		#returns velocity of the center of mass
		vw = np.array([speeds])

		return self.J.dot(vw)

	def ik(self, cmd):
		#returns rotational velocity of the wheels
		return self.Ji.dot(cmd)


	def remap(self, value, rotational = False):
		# False is linear velocity, True is rotational
		if rotational:
			return MAX_ROBOT_ROT_VEL*value
		else:
			return MAX_ROBOT_VEL*value/np.sqrt(2)

	def rad2cnt(self, val):
		#converts radians to counts and then rounds
		return np.floor(CNT_PER_REV*val/(2*np.pi))



if __name__ == '__main__':

	# myargv = rospy.myargv(argv=sys.argv)
	# OdriveMaster(myargv[1], myargv[2]) # Feed in odrive SSNs, and list of axes
	ssn_list = ['205337853548', '20653884304E']
	axes_list = [0,1,1,0]
	OdriveMaster(ssn_list, axes_list)