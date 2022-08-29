#!/usr/bin/python
"""
Interface from Python to ODrive
Daniel J. Gonzalez - dgonz@mit.edu
2.12 Intro to Robotics Spring 2019
Edited by Rachel Hoffman-Bice and Jerry Ng, January 2020
"""

import odrive
from odrive.enums import *
import time
import math
import fibre
import serial
import struct
import signal
import sys
import pdb
import matplotlib.pyplot as plt
import numpy 
from user_input.msg import Velocity, JoyCmd
import rospy 
from std_msgs.msg import ColorRGBA, Float32, Bool, Int32

pi = 3.1415927
pi
in2mm = 25.4
mm2in = 1/in2mm
in2m = in2mm/1000

Nm2A = 0.00000604 

class OdrivePython:
	def __init__(self, usb_serial, init_arg = True):
		self.usb_serials=usb_serial
		self.axes = []
		self.zeroVec = [[[0,0],[0,0]]]
		self.thtDesired = [[[0,0],[0,0]]]
		self.velDesired = [[[0,0],[0,0]]]
		self.kP = [[[0,0],[0,0]]]
		self.kD = [[[0,0],[0,0]]]
		self.home_kp = [[[0,0],[0,0]]]
		self.home_kd = [[[0,0],[0,0]]]
		self.kPd = [[[0,0],[0,0]]]
		self.kDd = [[[0,0],[0,0]]]

		self.CPR2RAD = (2*math.pi/400000)
		self.connect_all()
		self.full_init(reset = init_arg)
		self.set_gains(axis_num = 0, kpp = 0.1,  kvp =0.0001)
		self.set_gains(axis_num = 1, kpp = 0.1,  kvp =0.0001)


	def connect_all(self):
		#Connects to odrives of specified serial ids
		print("Finding odrive: " + self.usb_serials+ "...")

		odrv = odrive.find_any(serial_number = self.usb_serials)

		print("Found odrive!")

		self.odrv=odrv
		self.axes.append(odrv.axis0)
		self.axes.append(odrv.axis1)

	#Error checking print functions
	def print_controllers(self):
		for axis in self.axes:
			print(axis.controller)

	def print_encoders(self):
		for axis in self.axes:
			print(axis.controller)

	def printErrorStates(self):
		i = 0
		for axis in self.axes:
			i = i + 1
			print('Axis ' + str(i))
			print(' axis error:',hex(axis.error))
			print( ' motor error:',hex(axis.motor.error))
			print( ' encoder error:',hex(axis.encoder.error))


	def printPos(self):
		i = 0
		for axis in self.axes:
			i = i + 1
			print('Axis ' + str(i))
			print(' pos_estimate: ', axis.encoder.pos_estimate)
			print(' count_in_cpr: ', axis.encoder.count_in_cpr)
			print(' shadow_count: ', axis.encoder.shadow_count)


	def print_all(self):
		self.printErrorStates()
		self.print_encoders()
		self.print_controllers()


	def reboot(self):
		#Reboot and reconnect function
		self.odrv.reboot()
		time.sleep(5)
		connect_all()
		print('Rebooted ')

	def trajMoveCnt(self, axis_num, posDesired = 10000, velDesired = 50000, accDesired = 50000):
		#Move to a position with a specified trajectory
		self.axes[axis_num].trap_traj.config.vel_limit = velDesired 
		self.axes[axis_num].trap_traj.config.accel_limit = accDesired 
		self.axes[axis_num].trap_traj.config.decel_limit = accDesired
		self.axes[axis_num].controller.move_to_pos(posDesired)

	def pos_test_one(self, axis_num, amt = 400000, mytime = 5):
		#Used for the position test
		print('Changing modes to position control.')
		self.axes[axis_num].requested_state = AXIS_STATE_IDLE

		self.axes[axis_num].controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
		self.axes[axis_num].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
		print('Moving in... 3')
		time.sleep(1)
		print('2')
		time.sleep(1)
		print('1')
		time.sleep(1)
		self.axes[axis_num].controller.pos_setpoint = amt
		time.sleep(mytime)

		self.axes[axis_num].controller.pos_setpoint = 0
		time.sleep(mytime)

		self.axes[axis_num].controller.pos_setpoint = amt
		time.sleep(mytime)

		self.axes[axis_num].controller.pos_setpoint = 0
		time.sleep(mytime)

		self.axes[axis_num].requested_state = AXIS_STATE_IDLE


	def vel_test_one(self, axis_num, amt = 400000, mytime = 5):
		#Used for the velocity test
		self.axes[axis_num].requested_state = AXIS_STATE_IDLE
		print('Changing modes to velocity control.')
		self.axes[axis_num].controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

		self.axes[axis_num].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
		self.axes[axis_num].controller.vel_setpoint = 0
		print('Moving in... 3')
		time.sleep(1)
		print('2')
		time.sleep(1)
		print('1')
		time.sleep(1)

		self.axes[axis_num].controller.vel_setpoint = amt
		time.sleep(mytime)

		self.axes[axis_num].controller.vel_setpoint = 0
		time.sleep(mytime)

		self.axes[axis_num].controller.vel_setpoint = -amt
		time.sleep(mytime)

		self.axes[axis_num].controller.vel_setpoint = 0
		time.sleep(mytime)

		self.axes[axis_num].requested_state = AXIS_STATE_IDLE


	def traj_test_one(self, axis_num, amt = 400000, vel_limit = 100000, accel_limit = 10000, mytime = 10):
		#Used for the trajectory test
		self.axes[axis_num].requested_state = AXIS_STATE_IDLE
		print('Changing modes to trajectory control.')

		self.axes[axis_num].trap_traj.config.vel_limit = vel_limit
		self.axes[axis_num].trap_traj.config.accel_limit = accel_limit
		self.axes[axis_num].trap_traj.config.decel_limit = accel_limit
		self.axes[axis_num].controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
		self.axes[axis_num].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

		print('Moving in... 3')
		time.sleep(1)
		print('2')
		time.sleep(1)
		print('1')
		time.sleep(1)
		self.axes[axis_num].controller.move_to_pos(amt)
		time.sleep(mytime)

		self.axes[axis_num].controller.move_to_pos(0)
		time.sleep(mytime)

		self.axes[axis_num].controller.move_to_pos(amt)
		time.sleep(mytime)

		self.axes[axis_num].controller.move_to_pos(0)
		time.sleep(mytime)

		self.axes[axis_num].requested_state = AXIS_STATE_IDLE

	def erase_and_reboot(self):
		#Erase the configuration of the system and reboots
		print('erasing config')
		self.odrv.erase_configuration()
		print('reboot')
		self.odrv.reboot()


	def startup_init(self, axis_num):
		print('Initializing encoder calibration sequence')
		self.axes[axis_num].requested_state = AXIS_STATE_IDLE
		time.sleep(1)
		self.axes[axis_num].requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
		time.sleep(10)
		self.axes[axis_num].requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
		time.sleep(10)
		self.axes[axis_num].requested_state = AXIS_STATE_IDLE
		time.sleep(1)
		self.initflag=1

	def full_init(self,reset = True):
		self.odrv.config.brake_resistance = 0
		for axis in self.axes:
			if(reset):
				axis.motor.config.pre_calibrated = False

				#pole pairs
				axis.motor.config.pole_pairs = 4
				axis.controller.config.vel_limit = 200000 

				axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
				axis.encoder.config.cpr = 4000
				axis.encoder.config.use_index = True
				axis.encoder.config.zero_count_on_find_idx = True
				axis.encoder.config.pre_calibrated = False

				#motor calibration current
				axis.motor.config.calibration_current = 4
				time.sleep(1)
				axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
				time.sleep(10)
			time.sleep(5)
			# self.printErrorStates()
			axis.requested_state = AXIS_STATE_IDLE
			axis.motor.config.pre_calibrated=True

			axis.config.startup_encoder_index_search = True
			axis.config.startup_encoder_offset_calibration = True
			axis.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
			# self.printErrorStates()
			kP_des = 0.1
			kD_des = 0.0001
			axis.controller.config.pos_gain = kP_des 
			axis.controller.config.vel_gain = kD_des
			axis.controller.config.vel_integrator_gain = 0
			axis.controller.pos_setpoint = 0
			time.sleep(1)

		self.odrv.save_configuration()
		print('Calibration completed')
		self.printErrorStates()

	def PosMove(self,pos_setpt, axis_num):

		self.axes[axis_num].requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL
		self.axes[axis_num].controller.config.control_mode=CTRL_MODE_POSITION_CONTROL
		self.axes[axis_num].controller.pos_setpoint=pos_setpt

	def set_vel(self,vel_setpt, axis_num):
		#100000 = quarter rev per second
		self.axes[axis_num].requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL
		self.axes[axis_num].controller.config.control_mode=CTRL_MODE_VELOCITY_CONTROL
		self.axes[axis_num].controller.vel_setpoint=vel_setpt
	
		
	def PosMoveTuning(self,pos_setpt, axis_num):
			

		self.axes[axis_num].requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL
		self.axes[axis_num].controller.config.control_mode=CTRL_MODE_POSITION_CONTROL
		starting_pos = self.axes[axis_num].encoder.pos_estimate
		pos_setpt += starting_pos
		self.axes[axis_num].controller.pos_setpoint=pos_setpt
		
		##Initialize Figure
		timevar=10
		plt.ion()
		fig=plt.figure()
		plot_time = numpy.array([])
		plot_timesetpt=numpy.arange(0,timevar,.1)
		f=plot_timesetpt.size
		plot_setpt=numpy.ones(f)*pos_setpt
		plot_enc = numpy.array([])
		##Real Time Plotting
		start_time = time.time()
		i = self.axes[axis_num].motor.current_control.Iq_measured
		elapsed_time=0
		plt.plot(plot_timesetpt,plot_setpt-starting_pos,color = 'g')
		plt.xlabel('Time [s]')
		plt.ylabel('Motor Shaft Position [count]')
		while elapsed_time<timevar:            
			elapsed_time = time.time() - start_time
			plot_time = numpy.append(plot_time,elapsed_time)
			plot_enc = numpy.append(plot_enc,self.axes[axis_num].encoder.pos_estimate - starting_pos)
			plt.plot(plot_time,plot_enc);
			i+=1;
			plt.show()
			plt.pause(0.0001) #Note this correction
			i = self.axes[axis_num].motor.current_control.Iq_measured
		
	def VelMove(self,vel_setpt, axis_num):
		#100000 = quarter rev per second
		self.axes[axis_num].requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL
		self.axes[axis_num].controller.config.control_mode=CTRL_MODE_VELOCITY_CONTROL
		self.axes[axis_num].controller.vel_setpoint=vel_setpt

	def VelMoveTuning(self,vel_setpt, axis_num):
		#100000 = quarter rev per second
		self.axes[axis_num].requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL
		self.axes[axis_num].controller.config.control_mode=CTRL_MODE_VELOCITY_CONTROL
		self.axes[axis_num].controller.vel_setpoint=vel_setpt
		
		##Initialize Figure
		plt.ion()
		fig=plt.figure()
		plot_time = numpy.array([])
		timevar=10
		plot_enc = numpy.array([])
		plot_timesetpt=numpy.arange(0,timevar,.1)
		f=plot_timesetpt.size
		plot_setpt=numpy.ones(f)*vel_setpt
		##Real Time Plotting
		start_time = time.time()
		i = self.axes[axis_num].motor.current_control.Iq_measured
		plt.plot(plot_timesetpt,plot_setpt,color='g')
		plt.xlabel('Time [s]')
		plt.ylabel('Motor Shaft Velocity [count/s]')
		elapsed_time=0
		while elapsed_time<timevar:            
			elapsed_time = time.time() - start_time
			plot_time = numpy.append(plot_time,elapsed_time)
			plot_setpt = numpy.append(plot_time,elapsed_time)
			plot_enc = numpy.append(plot_enc,self.axes[axis_num].encoder.vel_estimate)
			plt.plot(plot_time,plot_enc);
			i+=1;
			plt.show()
			plt.pause(0.0001) #Note this correction
			i = self.axes[axis_num].motor.current_control.Iq_measured
			print(i)

	def make_perm(self):
		self.odrv.save_configuration()

	def set_gains(self,axis_num,kpp,kvp,kvi = 0):
		self.axes[axis_num].requested_state=AXIS_STATE_IDLE
		self.axes[axis_num].controller.config.pos_gain = kpp
		self.axes[axis_num].controller.config.vel_gain = kvp
		self.axes[axis_num].controller.config.vel_integrator_gain = kvi
		time.sleep(1)



	def set_closed_loop_state(self):
		self.axes[axis_num].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL


if __name__ == '__main__':

	myargv = rospy.myargv(argv=sys.argv)
	my_node(myargv[1])