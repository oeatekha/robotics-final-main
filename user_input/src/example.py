#! /usr/bin/env python
#testing various python functions and ros communication via rosbridge
import os
import rospy
import navigator
import math
import intera_interface 
from intera_interface import (
    SimpleClickSmartGripper,
    get_current_gripper_interface,
    Lights,
    Cuff,
    RobotParams,
)
import go_to
import waypoint
import time
import threading
import zeroG
import gripper_cuff
import pexpect
from std_msgs.msg import String
from std_msgs.msg import Int32
file_save = True
module_number = 2
int_msg = Int32()
int_msg.data = 0
grip_msg = Int32()
grip_msg.data = 1
unique_msg = String()
unique_msg.data = ''	
z_table = 0.127974138485
gripper_to_base_length = .08
box1_height = 0.110
box2_height = 0.110
box3_height = 0.110
bin_depth = 0.005
bin_height = 0.085
half_gripper_width = 0.015
gripper_width = 0.03
lift_position = z_table + box1_height - gripper_width
#rostopic pub -r 10 /robot/limb/right/suppress_cuff_interaction std_msgs/Empty

class cartPose(object):
	def __init__(self, pos_x=None, pos_y=None, pos_z=None, ori_x=None, ori_y=None, ori_z=None, ori_w = None):
		self.position_x = pos_x
		self.position_y = pos_y
		self.position_z = pos_z
		self.orientation_x = ori_x
		self.orientation_y = ori_y
		self.orientation_z = ori_z
		self.orientation_w = ori_w

class jointAngles(object):
	def __init__(self, j0 = None, j1 = None, j2 =None, j3=None, j4 = None, j5 = None, j6 = None):
		self.j0 = j0
		self.j1 = j1
		self.j2 = j2
		self.j3 = j3
		self.j4 = j4
		self.j5 = j5
		self.j6 = j6

class listener():
	def __init__(self):
		self.old_time = time.time()
		self.learner_quotient = 0
		self.interaction_counter = 0
		self.total_interactions = 0
		self.time_elapsed = 0
		self.learner_interaction = False
		self.old_time = time.time()
		self.new_time = time.time()
		self.waypoints = []
		self.waypoints_back = []
		self.waypoint_limit = 99
		self.effort = []
		self.new_angle = 0
		self.old_angle = 0
		self.changed_angle = 0
		self.eventnumber = 0
		self.iframe_sequence_pub = rospy.Publisher('/iframe_sequence', Int32, queue_size = 10)
		self.unique_input_pub = rospy.Publisher('/unique_input', String, queue_size = 10)
		self.zeroG_pub = rospy.Publisher('/zeroG_topic', String, queue_size = 10)
		self.gripper_pub = rospy.Publisher('/gripper_control', Int32, queue_size = 10)
		self.lights_pub = rospy.Publisher('/cuff_light', String, queue_size = 10)
		self.interaction_pub = rospy.Publisher('/interaction_topic', Int32, queue_size = 10)
		rospy.Subscriber('/upper_cuff_button', Int32, self.upper_cuff_button_callback)
		rospy.Subscriber('/lower_cuff_button', Int32, self.lower_cuff_button_callback)
		rospy.Subscriber('/bot_sequence', Int32, self.callback)
		rospy.Subscriber('/save_info', String, self.save_info_callback)
		rospy.Subscriber('/right_navigator_button', String, self.navigator_callback)
		rospy.init_node('Sawyer_Sparrow_comm_node', anonymous=True)
		self.limb = intera_interface.Limb('right')

		self._gripper = get_current_gripper_interface()
		self._is_clicksmart = isinstance(self._gripper, SimpleClickSmartGripper)

		self.startPose = self.limb.endpoint_pose() 
		self.startPose_container = cartPose()
		self.startPose_arg = go_to.cartesian_pose_arg()
		self.startJointAngles = self.limb.joint_angles()
		self.newJointAngles_arg = go_to.joint_angle_arg()
		self.newCartPose_arg = go_to.cartesian_pose_arg()
		self.newCartPose_arg2 = go_to.cartesian_pose_arg()
		self.newCartPose = cartPose()
		self.newJointAngles = jointAngles()
		self.waypoint_init_container = cartPose()
		self.waypoint_init_arg = go_to.cartesian_pose_arg()
		#Check if gripper is attached properly
		self.grip = gripper_cuff.GripperConnect()
		self.savedLocations = []
		#Open gripper
		grip_msg.data = 1
		self.gripper_pub.publish(grip_msg)

		#set constrained mode
		
		zeroG.constrained(zeroG_all_constraints)

		# self.startPose = self.limb.endpoint_pose()
		# print(self.limb.endpoint_pose())
		# self.startPose_container = self.pose_to_cartclass(pose = self.startPose, cartclass = self.startPose_container)
		# print(self.startPose_container.position_z)
		# print(self.startPose_container.position_z)
		# self.startPose_arg = self.cartclass_to_cartarg(cartclass = self.startPose_container, cartarg = self.startPose_arg)
		# print(self.startPose_container.position_z)
		# go_to.cartesian_pose(self.startPose_arg)
		#Check files and set text file to save to
		if(file_save):
			self.home = os.path.expanduser("~")
			self.dir_path = self.home + '/Learner_Responses/module' + str(module_number) + '/'
			if not os.path.exists(self.dir_path):
				os.makedirs(self.dir_path)

			self.file_number = 0
			self.file_path = self.dir_path + 'test_' + str(self.file_number) + '.txt'
			while(os.path.exists(self.file_path)):
				self.file_number += 1
				self.file_path = self.dir_path + 'test_' + str(self.file_number) + '.txt'

			self.file = open(self.file_path, 'w')
			self.file.write('pretest \r\n' + str(self.old_time) + '\r\n')
			self.file.close()

		
		go_to.joint_angles(empty_angle_arg)

		# #Complete grip
		# grip_msg.data = 0
		# self.gripper_pub.publish(grip_msg)
		#Test code here
		#effort testing
		# self.waypoints = []
		# self.waypoint_limit = 5
		#initialize navigator for user inputs
		navigator.right()

	def compare_efforts(self, effort1, effort2, tol_effort = 2.8):
		return abs(effort1['right_j6']-effort2['right_j6'])<=tol_effort \
			   and abs(effort1['right_j5']-effort2['right_j5'])<=tol_effort \
			   and abs(effort1['right_j4']-effort2['right_j4'])<=tol_effort \
			   and abs(effort1['right_j3']-effort2['right_j3'])<=tol_effort \
			   and abs(effort1['right_j2']-effort2['right_j2'])<=tol_effort \
			   and abs(effort1['right_j1']-effort2['right_j1'])<=tol_effort \
			   and abs(effort1['right_j0']-effort2['right_j0'])<=tol_effort

	def check_pickup(self, lift_position = z_table + box1_height - gripper_width):
		del self.effort[:]

		self.startPose = self.limb.endpoint_pose()
		self.startPose_container = self.pose_to_cartclass(pose = self.startPose, cartclass = self.startPose_container)

		self.startPose_container.position_z = lift_position + .1
		self.startPose_arg = self.cartclass_to_cartarg(cartclass = self.startPose_container, cartarg = self.startPose_arg)

		self.newCartPose = self.pose_to_cartclass(pose = self.startPose, cartclass = self.newCartPose)
		self.newCartPose.position_z = lift_position
		self.newCartPose_arg = self.cartclass_to_cartarg(cartclass = self.newCartPose, cartarg = self.newCartPose_arg)
		
		go_to.cartesian_pose(self.startPose_arg)
		self.effort.append(self.limb.joint_efforts())


		go_to.cartesian_pose(self.newCartPose_arg)
		grip_msg.data = 0
		self.gripper_pub.publish(grip_msg)
		rospy.sleep(2)
		go_to.cartesian_pose(self.startPose_arg)
		self.effort.append(self.limb.joint_efforts())
		if(not self.compare_efforts(effort1 = self.effort[0], effort2 = self.effort[1])):
			go_to.cartesian_pose(self.newCartPose_arg)

	def endpoints_equal(self, pose1, pose2, tol=0.01):
		return abs(pose1['position'].x-pose2['position'].x)<=tol \
			   and abs(pose1['position'].y-pose2['position'].y)<=tol \
			   and abs(pose1['position'].z-pose2['position'].z)<=tol \
			   and abs(pose1['orientation'].x-pose2['orientation'].x)<=tol \
			   and abs(pose1['orientation'].y-pose2['orientation'].y)<=tol \
			   and abs(pose1['orientation'].z-pose2['orientation'].z)<=tol \
			   and abs(pose1['orientation'].w-pose2['orientation'].w)<=tol

	def pose_to_cartclass(self, pose, cartclass):
		cartclass.position_x = pose['position'].x
		cartclass.position_y = pose['position'].y
		cartclass.position_z = pose['position'].z
		cartclass.orientation_x = pose['orientation'].x
		cartclass.orientation_y = pose['orientation'].y
		cartclass.orientation_z = pose['orientation'].z
		cartclass.orientation_w = pose['orientation'].w
		return cartclass

	def cartclass_to_cartarg(self, cartclass, cartarg):
		cartarg = go_to.cartesian_pose_arg(position = [cartclass.position_x, cartclass.position_y, cartclass.position_z],
			orientation = [cartclass.orientation_x, cartclass.orientation_y, cartclass.orientation_z, cartclass.orientation_w])
		return cartarg

	def limbangles_to_angleclass(self, limbangles, angleclass):
		angleclass.j0 = limbangles['right_j0']
		angleclass.j1 = limbangles['right_j1']
		angleclass.j2 = limbangles['right_j2']
		angleclass.j3 = limbangles['right_j3']
		angleclass.j4 = limbangles['right_j4']
		angleclass.j5 = limbangles['right_j5']
		angleclass.j6 = limbangles['right_j6']
		return angleclass

	def angleclass_to_anglearg(self, angleclass, anglearg):
		anglearg = go_to.joint_angle_arg(joint_angles = [angleclass.j0, angleclass.j1, angleclass.j2, angleclass.j3, angleclass.j4, angleclass.j5, angleclass.j6])
		return anglearg


	def navigator_callback(self, data):
		self.new_time = time.time()
		self.time_elapsed = self.new_time - self.old_time
		self.file = open(self.file_path, 'a')
		self.file.write('time stamp: ' + str(self.time_elapsed) + '\r\n' + 'Navigator: ' + str(data.data) + '\r\n')
		self.file.close()
		if(self.learner_interaction and data.data == 'Button \'OK\': OFF'):
			self.learner_interaction = False
			zeroG.constrained(zeroG_all_constraints)
			self.interaction_counter = self.interaction_counter + 1
			self.total_interactions = self.total_interactions + 1
			int_msg.data = self.interaction_counter
			self.interaction_pub.publish(int_msg)
			rospy.sleep(0.1)
				
	def upper_cuff_button_callback(self, data):
		#white button
		#switch to zeroG, orientation mode
		self.new_time = time.time()
		self.time_elapsed = self.new_time - self.old_time
		self.file = open(self.file_path, 'a')
		self.file.write('time stamp: ' + str(self.time_elapsed) + '\r\n' + 'upper_cuff_button callback: ' + str(data.data) + '\r\n')
		self.file.close()
		if(self.learner_interaction):
			zeroG.constrained(zeroG_pos_constraints)
			rospy.sleep(0.1)

	def lower_cuff_button_callback(self, data):
		#gray button
		#switch to zeroG, Position mode
		self.new_time = time.time()
		self.time_elapsed = self.new_time - self.old_time
		self.file = open(self.file_path, 'a')
		self.file.write('time stamp: ' + str(self.time_elapsed) + '\r\n' + 'lower_cuff_button callback: ' + str(data.data) + '\r\n')
		self.file.close()
		if(self.learner_interaction):
			zeroG.constrained(zeroG_ori_constraints)
			rospy.sleep(0.1)

	def save_info_callback(self, data):
		if data.data == 'time_start':
			self.old_time = time.time()

		elif data.data == 'time_end':
			self.new_time = time.time()
			self.time_elapsed = self.new_time - self.old_time
			self.file = open(self.file_path, 'a')
			self.file.write(str(self.time_elapsed) + '\r\n')
			self.file.close()


		else:
			self.file = open(self.file_path, 'a')
			self.file.write(str(data.data) + '\r\n')
			self.file.close()

	def callback(self, data):
		self.new_time = time.time()
		self.time_elapsed = self.new_time - self.old_time
		self.file = open(self.file_path, 'a')
		self.file.write('time stamp: ' + str(self.time_elapsed) + '\r\n' + 'callback: ' + str(data.data) + '\r\n')
		self.file.close()
		rospy.loginfo(rospy.get_caller_id() + "I heard" + str(data.data))
		if (data.data == 0):
			grip_msg.data = 1
			self.gripper_pub.publish(grip_msg)
			go_to.joint_angles(empty_angle_arg)
			int_msg.data = 0
			self.iframe_sequence_pub.publish(int_msg)


		if (data.data == 1):
			#To begin with, let me try to grab it with my hands
			#Fail to grip the object
			#Open Gripper
			grip_msg.data = 1
			self.gripper_pub.publish(grip_msg)
			go_to.joint_angles(empty_angle_arg)

			#Initial move
			go_to.joint_angles(init_joint_arg)

			#Go to gripping position
			go_to.joint_angles(fail_init_joint_arg)
			go_to.cartesian_pose(fail_pickup_cart_arg)

			#Grip
			grip_msg.data = 0
			self.gripper_pub.publish(grip_msg)

			#fail and then lift up
			#Open gripper
			rospy.sleep(1)
			go_to.cartesian_pose(fail_above_pickup_cart_arg)
			grip_msg.data = 1
			self.gripper_pub.publish(grip_msg)
			go_to.joint_angles(init_joint_arg)
			go_to.joint_angles(joint_buttons)

			int_msg.data = 1
			self.iframe_sequence_pub.publish(int_msg)
			self.learner_interaction = True

		elif (data.data == 2):
			#The first lesson of the day
			#Properly grip the object
			go_to.joint_angles(empty_angle_arg)
			go_to.joint_angles(init_joint_arg)
			go_to.joint_angles(fail_init_joint_arg)
			go_to.joint_angles(success_init_joint_arg)
			int_msg.data = 2
			self.iframe_sequence_pub.publish(int_msg)
			go_to.cartesian_pose(success_pickup_cart_arg)
			
			#Complete grip
			grip_msg.data = 0
			self.gripper_pub.publish(grip_msg)

			#Show off the proper gripping
			go_to.cartesian_pose(success_above_pickup_cart_arg)
			go_to.joint_angles(success_init_joint_arg)
			go_to.joint_angles(init_joint_arg)

			#put it back down
			go_to.joint_angles(success_init_joint_arg)
			go_to.cartesian_pose(success_pickup_cart_arg)
			grip_msg.data = 1
			self.gripper_pub.publish(grip_msg)
			go_to.cartesian_pose(success_above_pickup_cart_arg)

			#Get ready for next exercise
			go_to.joint_angles(success_init_joint_arg)


		elif(data.data == 3):
			#To get a feeling for what I mean
			self.learner_interaction = True

		elif(data.data == 4):
			#Let's try an example.
			go_to.joint_angles(success_init_joint_arg)
			self.learner_interaction = True
			if int_msg.data !=4:
				int_msg.data = 3
				self.iframe_sequence_pub.publish(int_msg)


		elif(data.data == 5):
			#alright let's see
			self.check_pickup()
			if(self.compare_efforts(effort1 = self.effort[0], effort2 = self.effort[1])):
				#did not lift the shoe. Efforts are the same.
				grip_msg.data = 1
				self.gripper_pub.publish(grip_msg)
				int_msg.data = 4
				self.iframe_sequence_pub.publish(int_msg)
				rospy.sleep(1)
			else:
				#did lift the shoe.
				int_msg.data = 5
				self.iframe_sequence_pub.publish(int_msg)
				go_to.cartesian_pose(self.startPose_arg)

		elif(data.data == 6):
			#For example, now that I've picked up the shoe
			go_to.cartesian_pose(above_bin_cart_arg)

			self.startPose = self.limb.endpoint_pose()
			self.newCartPose = self.pose_to_cartclass(pose = self.startPose, cartclass = self.newCartPose)
			
			self.newCartPose.position_z = z_table + bin_height + box2_height - gripper_width
			self.newCartPose_arg = self.newCartPose_arg = self.cartclass_to_cartarg(cartclass = self.newCartPose, cartarg = self.newCartPose_arg)
			go_to.cartesian_pose(self.newCartPose_arg)
			rospy.sleep(1)
			go_to.cartesian_pose(above_bin_cart_arg)
			int_msg.data = 6
			self.iframe_sequence_pub.publish(int_msg)

		
		elif(data.data == 7):
			#... it won't fit. So, before I can put the shoe in the shoebox...
			self.startJointAngles = self.limb.joint_angles()
			self.newJointAngles = self.limbangles_to_angleclass(limbangles = self.startJointAngles, angleclass = self.newJointAngles)

			self.newJointAngles.j6 = self.startJointAngles['right_j6']+1.57
			self.newJointAngles_arg = self.angleclass_to_anglearg(angleclass = self.newJointAngles, anglearg = self.newJointAngles_arg)

			go_to.joint_angles(self.newJointAngles_arg)
			int_msg.data = 7
			self.iframe_sequence_pub.publish(int_msg)


		elif(data.data == 8):
			#Now I can package the box
			#Update current pose
			self.startPose = self.limb.endpoint_pose()
			self.startPose_container = self.pose_to_cartclass(pose = self.startPose, cartclass = self.startPose_container)
			self.startPose_arg = self.cartclass_to_cartarg(cartclass = self.startPose_container, cartarg = self.startPose_arg)

			self.newCartPose = self.pose_to_cartclass(pose = self.startPose, cartclass = self.newCartPose)

			self.newCartPose.position_z = z_table + bin_depth + box2_height - gripper_width
			self.newCartPose_arg = self.cartclass_to_cartarg(cartclass = self.newCartPose, cartarg = self.newCartPose_arg)
			go_to.cartesian_pose(self.newCartPose_arg)
			grip_msg.data = 1
			self.gripper_pub.publish(grip_msg)

			go_to.cartesian_pose(self.startPose_arg)
			go_to.joint_angles(init_joint_arg)
			int_msg.data = 8
			self.iframe_sequence_pub.publish(int_msg)

		elif(data.data == 9):
			#Now let's try moving this 3rd box into the 2nd bin.
			go_to.joint_angles(success_init_joint_arg)
			self.learner_interaction = True

		elif(data.data == 10):
			self.check_pickup()
			if(self.compare_efforts(effort1 = self.effort[0], effort2 = self.effort[1])):
				#did not lift the shoe. Efforts are the same.
				grip_msg.data = 1
				self.gripper_pub.publish(grip_msg)
				int_msg.data = 9
				self.iframe_sequence_pub.publish(int_msg)
				rospy.sleep(1)
			else:
				#did lift the shoe.
				int_msg.data = 10
				self.iframe_sequence_pub.publish(int_msg)


		elif(data.data == 11):
			#correct answer
			int_msg.data = 11
			self.iframe_sequence_pub.publish(int_msg)


		elif(data.data == 12):
			#correct answer continued
			self.startPose = self.limb.endpoint_pose()
			self.startPose_container = self.pose_to_cartclass(pose = self.startPose, cartclass = self.startPose_container)
			self.startPose_arg = self.cartclass_to_cartarg(cartclass = self.startPose_container, cartarg = self.startPose_arg)

			grip_msg.data = 0
			self.gripper_pub.publish(grip_msg)

			go_to.cartesian_pose(into_second_bin_cart_arg)
			go_to.cartesian_pose(self.startPose_arg)

			int_msg.data = 12
			self.iframe_sequence_pub.publish(int_msg)


		elif(data.data == 13):
			self.startPose = self.limb.endpoint_pose()
			self.newCartPose = self.pose_to_cartclass(pose = self.startPose, cartclass = self.startPose_container)
			self.newCartPose.position_z = z_table + box2_height - gripper_width + bin_height + .2
			self.newCartPose_arg = self.cartclass_to_cartarg(cartclass = self.startPose_container, cartarg = self.startPose_arg)

			
			go_to.cartesian_pose(self.newCartPose_arg)
			int_msg.data = 13
			self.iframe_sequence_pub.publish(int_msg)

		elif(data.data == 14):
			go_to.cartesian_pose(above_second_bin_cart_arg)
			int_msg.data = 14
			self.iframe_sequence_pub.publish(int_msg)


		elif(data.data == 15):
			self.startPose = self.limb.endpoint_pose()
			self.newCartPose = self.pose_to_cartclass(pose = self.startPose, cartclass = self.startPose_container)
			self.newCartPose.position_z = z_table + box2_height + bin_depth - gripper_width
			self.newCartPose_arg = self.cartclass_to_cartarg(cartclass = self.newCartPose, cartarg = self.newCartPose_arg)
			go_to.cartesian_pose(self.newCartPose_arg)

			grip_msg.data = 1
			self.gripper_pub.publish(grip_msg)

			go_to.cartesian_pose(above_second_bin_cart_arg)
			#skip incorrect answer
			int_msg.data = 17
			self.iframe_sequence_pub.publish(int_msg)


		elif(data.data == 16):
			#incorrect answer
			int_msg.data = 15
			self.iframe_sequence_pub.publish(int_msg)

		elif(data.data == 17):
			#same as 12, moving box into the bin. 
			self.startPose = self.limb.endpoint_pose()
			self.startPose_container = self.pose_to_cartclass(pose = self.startPose, cartclass = self.startPose_container)
			self.startPose_arg = self.cartclass_to_cartarg(cartclass = self.startPose_container, cartarg = self.startPose_arg)

			grip_msg.data = 0
			self.gripper_pub.publish(grip_msg)

			go_to.cartesian_pose(into_second_bin_cart_arg)
			go_to.cartesian_pose(self.startPose_arg)

			int_msg.data = 12
			self.iframe_sequence_pub.publish(int_msg)

		elif(data.data == 18):
			#go back and grip the box
			grip_msg.data = 1
			self.gripper_pub.publish(grip_msg)

			self.startPose = self.limb.endpoint_pose()
			self.startPose_container = self.pose_to_cartclass(pose = self.startPose, cartclass = self.startPose_container)
			self.startPose_container.position_z = z_table + box2_height - gripper_width + 0.01
			self.startPose_arg = self.cartclass_to_cartarg(cartclass = self.startPose_container, cartarg = self.startPose_arg)
			go_to.cartesian_pose(self.startPose_arg)
			grip_msg.data = 0
			self.gripper_pub.publish(grip_msg)

			int_msg.data = 18
			self.iframe_sequence_pub.publish(int_msg)

		elif(data.data == 19):
			#Lift the box and then move back and then set it down
			self.startPose = self.limb.endpoint_pose()
			self.startPose_container = self.pose_to_cartclass(pose = self.startPose, cartclass = self.startPose_container)
			self.startPose_container.position_z = self.startPose_container.position_z + box2_height + 0.01
			self.startPose_arg = self.cartclass_to_cartarg(cartclass = self.startPose_container, cartarg = self.startPose_arg)

			go_to.cartesian_pose(self.startPose_arg)
			go_to.cartesian_pose(above_third_box_cart_arg)

			self.startPose = self.limb.endpoint_pose()
			self.startPose_container = self.pose_to_cartclass(pose = self.startPose, cartclass = self.startPose_container)
			self.startPose_container.position_z = z_table + box2_height - gripper_width
			self.newCartPose_arg = self.cartclass_to_cartarg(cartclass = self.startPose_container, cartarg = self.newCartPose_arg)

			go_to.cartesian_pose(self.newCartPose_arg)
			grip_msg.data = 1
			self.gripper_pub.publish(grip_msg)
			int_msg.data = 19
			self.iframe_sequence_pub.publish(int_msg)


		elif(data.data == 20):
			go_to.joint_angles(success_init_joint_arg)
			self.learner_interaction = True

		elif(data.data == 21):
			self.check_pickup()
			if(self.compare_efforts(effort1 = self.effort[0], effort2 = self.effort[1])):
				#did not lift the shoe. Efforts are the same.
				int_msg.data = 20
				self.iframe_sequence_pub.publish(int_msg)
				grip_msg.data = 1
				self.gripper_pub.publish(grip_msg)
				rospy.sleep(1)
			else:
				#did lift the shoe.
				int_msg.data = 31
				self.iframe_sequence_pub.publish(int_msg)
				grip_msg.data = 1
				self.gripper_pub.publish(grip_msg)
				go_to.cartesian_pose(self.startPose_arg)
				go_to.joint_angles(empty_angle_arg)
		


		elif(data.data==32):
			go_to.joint_angles(empty_angle_arg)
			zeroG.constrained(zeroG_truly_no_constraints)
			int_msg.data = 32
			self.iframe_sequence_pub.publish(int_msg)

		elif(data.data==33):
			zeroG.constrained(zeroG_all_constraints)
			rospy.sleep(0.1)
			int_msg.data = 33
			self.iframe_sequence_pub.publish(int_msg)


		elif(data.data==34):
			go_to.joint_angles(self.savedLocations[0])
			int_msg.data = 34
			self.iframe_sequence_pub.publish(int_msg)


		elif(data.data==35):
			go_to.joint_angles(self.savedLocations[1])
			int_msg.data = 35
			self.iframe_sequence_pub.publish(int_msg)


		elif(data.data==36):
			go_to.joint_angles(self.savedLocations[2])
			int_msg.data = 36
			self.iframe_sequence_pub.publish(int_msg)


		elif(data.data==37):
			go_to.joint_angles(self.savedLocations[3])
			int_msg.data = 37
			self.iframe_sequence_pub.publish(int_msg)


		elif(data.data==38):
			go_to.joint_angles(self.savedLocations[4])
			int_msg.data = 38
			self.iframe_sequence_pub.publish(int_msg)


		elif(data.data==100):
			JointAnglesLimb = self.limb.joint_angles()
			JointAnglesClass = jointAngles()
			JointAnglesClass = self.limbangles_to_angleclass(JointAnglesLimb, JointAnglesClass)
			JointAngleArg = go_to.joint_angle_arg()
			JointAngleArg = self.angleclass_to_anglearg(JointAnglesClass, JointAngleArg)
			self.savedLocations.append(JointAngleArg)

		# 	zeroG.constrained(zeroG_ori_constraints)

		# 	del self.waypoints[:]
		# 	self.waypoint_limit = 5
		# 	self.waypoint_init_pose = self.limb.endpoint_pose()
		# 	self.waypoint_init_container = self.pose_to_cartclass(pose = self.waypoint_init_pose, cartclass = self.waypoint_init_container)
		# 	self.waypoint_init_arg = self.cartclass_to_cartarg(cartclass = self.waypoint_init_container, cartarg = self.waypoint_init_arg)
		# 	#set first waypoint to be the start point
		# 	self.waypoints.append(self.waypoint_init_arg)
		# 	self.eventnumber = 35

		# elif(data.data == 35):
		# 	#move to the beginning of the obstacle course and then check waypoints.
		# 	go_to.joint_angles(second_course_arg)
		# 	unique_msg.data = str(self.check_intersect(waypoint_list = self.waypoints, line_segment_list = second_obstacle_segments))
		# 	if(unique_msg.data == []):
		#  		#publish that no errors were found and begin movement.
		#  		unique_msg.data = 'no errors'
		#  		self.unique_input_pub.publish(unique_msg)
		#  		for x in range(0,len(self.waypoints)):
		#  			go_to.cartesian_pose(self.waypoints[x])
		#  		#Now let's try another course
		#  		int_msg.data = 37
		#  		self.iframe_sequence_pub.publish(int_msg)

		#  	else:
		#  		#[i,j] where i is the waypoint line segment and j is the obstacle line segment
		#  		unique_msg.data = str(self.check_intersect(waypoint_list = self.waypoints, line_segment_list = second_obstacle_segments))
		#  		self.unique_input_pub.publish(unique_msg)

		# elif(data.data == 36):
		#  	#Wrong waypoints were sent.
		# 	for x in range(0, int(unique_msg.data[0])+1):
		#  			#goes one additional waypoint more than the waypoint of intersection
		#  			go_to.cartesian_pose(self.waypoints[x])

		#  	del self.waypoints[:]
		#  	int_msg.data = 36
		#  	self.iframe_sequence_pub.publish(int_msg)

		# elif(data.data == 37):
		# 	#the last activity is to make it back to the beginning
		# 	go_to.joint_angles(second_course_bckwrds_arg)
		# 	rospy.sleep(2)
		# 	zeroG.constrained(zeroG_ori_constraints)

		# 	del self.waypoints[:]
		# 	self.waypoint_limit = 5
		# 	self.waypoint_init_pose = self.limb.endpoint_pose()
		# 	self.waypoint_init_container = self.pose_to_cartclass(pose = self.waypoint_init_pose, cartclass = self.waypoint_init_container)
		# 	self.waypoint_init_arg = self.cartclass_to_cartarg(cartclass = self.waypoint_init_container, cartarg = self.waypoint_init_arg)
		# 	#set first waypoint to be the start point
		# 	self.waypoints.append(self.waypoint_init_arg)
		# 	self.eventnumber = 38

		# elif(data.data == 38):
		# 	#move to the beginning of the obstacle course and then check waypoints.
		# 	go_to.joint_angles(second_course_bckwrds_arg)
		# 	unique_msg.data = str(self.check_intersect(waypoint_list = self.waypoints, line_segment_list = second_obstacle_segments))
		# 	if(unique_msg.data == []):
		#  		#publish that no errors were found and begin movement.
		#  		unique_msg.data = 'no errors'
		#  		self.unique_input_pub.publish(unique_msg)
		#  		for x in range(0,len(self.waypoints)):
		#  			go_to.cartesian_pose(self.waypoints[x])
		#  		#Now let's try another course
		#  		int_msg.data = 40
		#  		self.iframe_sequence_pub.publish(int_msg)

		#  	else:
		#  		#[i,j] where i is the waypoint line segment and j is the obstacle line segment
		#  		unique_msg.data = str(self.check_intersect(waypoint_list = self.waypoints, line_segment_list = second_obstacle_segments))
		#  		self.unique_input_pub.publish(unique_msg)

		# elif(data.data == 39):
		#  	#Wrong waypoints were sent.
		# 	for x in range(0, int(unique_msg.data[0])+1):
		#  			#goes one additional waypoint more than the waypoint of intersection
		#  			go_to.cartesian_pose(self.waypoints[x])

		#  	del self.waypoints[:]
		#  	int_msg.data = 39
		#  	self.iframe_sequence_pub.publish(int_msg)


		# elif(data.data == 40):
		#  	rospy.sleep(10)





if __name__ == '__main__':
	empty_angle_arg = go_to.joint_angle_arg()
	empty_cartesian_arg = go_to.cartesian_pose_arg()
	child = pexpect.spawn('rostopic pub -r 10 /robot/limb/right/suppress_cuff_interaction std_msgs/Empty')

	#variables to be used
	init_cartesian_arg = go_to.cartesian_pose_arg(joint_angles = [-0.438147460938,-0.678481445312,-0.433721679687,1.33986621094,-0.763953125,-1.132484375,0.959416015625])
	init_joint_arg = go_to.joint_angle_arg(joint_angles =[-0.438147460938,-0.678481445312,-0.433721679687,1.33986621094,-0.763953125,-1.132484375,0.959416015625] )
	fail_init_joint_arg = go_to.joint_angle_arg(joint_angles = [-0.33585546875,-0.379596679687,-1.60570117188,1.12274511719,-1.9501328125,-1.73130761719,1.83596582031])
	fail_pickup_cart_arg = go_to.cartesian_pose_arg(joint_angles = [-0.335752929687,0.188111328125,-1.50376367188,1.44372558594,-1.40977148437,-1.44762792969,1.51276660156])
	fail_above_pickup_cart_arg = go_to.cartesian_pose_arg(joint_angles = [-0.33585546875,-0.379596679687,-1.60570117188,1.12274511719,-1.9501328125,-1.73130761719,1.83596582031])

	joint_buttons = go_to.joint_angle_arg(joint_angles = [-0.0393427734375,-0.71621875,0.022359375,1.811921875,-0.116017578125,2.77036035156,-4.48708789063], speed_ratio = 0.9, accel_ratio = 0.5)

	success_init_joint_arg = go_to.joint_angle_arg(joint_angles = [-0.391934570313,-0.355940429688,-1.49954101562,1.22303125,-1.92679003906,-1.65482226563,3.24194140625])
	success_pickup_joint_arg = go_to.joint_angle_arg(joint_angles = [-0.383508789063,-0.348434570313,-1.62057226562,1.12419726563,-1.88032324219,-1.72695605469,3.28075195312])
	success_pickup_cart_arg = go_to.cartesian_pose_arg(joint_angles =[-0.86590625,0.877251953125,-2.45294628906,1.81320800781,-0.65857421875,-2.26748535156,2.956015625])
	success_above_pickup_cart_arg = go_to.cartesian_pose_arg(joint_angles =[-0.391934570313,-0.355940429688,-1.49954101562,1.22303125,-1.92679003906,-1.65482226563,3.24194140625])

	above_bin_cart_arg = go_to.cartesian_pose_arg(joint_angles = [-0.429515625,-0.492512695313,-1.25894921875,1.58936425781,-2.12879492187,-1.21486914063,2.99199902344])
	rotated_above_bin_cart_arg = go_to.cartesian_pose_arg(joint_angles = [-0.239127929687,-0.440267578125,-0.618359375,2.16329882812,1.75038769531,0.603390625,0.984127929687])
	into_bin_cart_arg = go_to.cartesian_pose_arg(joint_angles = [-2.04193945312,0.905513671875,-2.16442285156,-2.058984375,0.59423828125,1.62292871094,-1.11868945313])

	into_second_bin_cart_arg = go_to.cartesian_pose_arg(joint_angles = [-0.89046875,0.309813476563,-1.65483398437,1.14237109375,-1.32817382812,-1.48642578125,2.85633007813])
	above_second_bin_cart_arg = go_to.cartesian_pose_arg(joint_angles = [-0.941543945312,0.193456054687,-1.71885546875,1.12710351562,-1.39799609375,-1.59924121094,2.80375683594])

	above_second_box_cart_arg = go_to.cartesian_pose_arg(joint_angles = [-0.729661132813,0.12510546875,-1.78256054688,1.094546875,-1.35895410156,-1.71990722656,4.66472753906])
	above_third_box_cart_arg = go_to.cartesian_pose_arg(joint_angles = [-0.988412109375,0.25142578125,-1.95062207031,0.73998046875,-1.17322949219,-1.695453125,3.15728417969])
	above_fourth_box_cart_arg = go_to.cartesian_pose_arg(joint_angles = [-1.482765625,0.17649609375,-2.27079101562,0.363586914062,-0.855272460937,-1.6052578125,1.5651328125])
	first_obstacle_start = [0.399860507274,-0.644030013122,0.133852047821]
	first_obstacle_end = [-0.227945308794,-0.647373646171,0.140652495049]
	first_obstacle_segments = []
	course1_seg1 = []
	course1_seg1.append([0.320707086597,-0.891440388143,0.136162922161])
	course1_seg1.append([0.302925566947,-0.708980883519,0.133541603958])
	course1_seg2 = []
	course1_seg2.append([0.231502942421,-0.728939803477,0.13452432824])
	course1_seg2.append([0.224147857193,-0.643210997585,0.136544420308])
	course1_seg3 = []
	course1_seg3.append([0.161667533248,-0.477170744228,0.134025520215])
	course1_seg3.append([0.164681443128,-0.252524045326,0.133679336876])
	course1_seg4 = []
	course1_seg4.append([0.0345295589078,-0.885604841201,0.13814561457])
	course1_seg4.append([0.0398246570335,-0.569510742333,0.1373215404])
	course1_seg5 = []
	course1_seg5.append([-0.111881056524,-0.724872638964,0.138292304304])
	course1_seg5.append([-0.0920161573037,-0.547839190658,0.137311740722])
	course1_seg6 = []
	course1_seg6.append([-0.141950763931,-0.300855779926,0.140039691397])
	course1_seg6.append([-0.154090762813,-0.546484156226,0.138445238244])
	first_obstacle_segments.append(course1_seg1)
	first_obstacle_segments.append(course1_seg2)
	first_obstacle_segments.append(course1_seg3)
	first_obstacle_segments.append(course1_seg4)
	first_obstacle_segments.append(course1_seg5)
	first_obstacle_segments.append(course1_seg6)

	second_obstacle_segments = []
	course2_seg1 = []
	course2_seg1.append([0.338111105248,-0.899843206302,0.134451215272])
	course2_seg1.append([0.336691579842,-0.562921265174,0.131890953155])
	course2_seg2 = []
	course2_seg2.append([0.227897743673,-0.268608051256,0.135133008047])
	course2_seg2.append([0.223305588404,-0.466743142449,0.132906918834])
	course2_seg3 = []
	course2_seg3.append([0.160416241091,-0.282563500869,0.131372411175])
	course2_seg3.append([0.164132153151,-0.558348752997,0.134540383954])
	course2_seg4 = []
	course2_seg4.append([0.0304143008842,-0.821931764334,0.137179538331])
	course2_seg4.append([0.032920236144,-0.590739671509,0.136797206157])
	course2_seg5 = []
	course2_seg5.append([-0.106923794599,-0.986124134397,0.140403383516])
	course2_seg5.append([-0.101176875497,-0.694803147479,0.139127759436])
	course2_seg6 = []
	course2_seg6.append([-0.147813245505,-0.512607912465,0.138083224809])
	course2_seg6.append([-0.169295354875,-0.285453111457,0.140002718868])
	second_obstacle_segments.append(course2_seg1)
	second_obstacle_segments.append(course2_seg2)
	second_obstacle_segments.append(course2_seg3)
	second_obstacle_segments.append(course2_seg4)
	second_obstacle_segments.append(course2_seg5)
	second_obstacle_segments.append(course2_seg6)

	#course1_seg2 = 
	init_course_corners1 = []
	init_course_corners1.append([0.715563023545,-0.499392455919,-0.0293937940559])
	init_course_corners1.append([0.327523936334,-0.457078649427,-0.030030885182])
	init_course_corners1.append([0.278365976057,-0.909909631392,-0.0271018425405])
	init_course_corners1.append([0.143122958412,-0.899859163685,-0.0241534678283])
	init_course_corners1.append([0.159813919642,-0.7359369119,-0.0259437455369])
	init_course_corners1.append([0.0131915019168,-0.736565979572,-0.0200761952404])
	init_course_corners1.append([0.0495882998644,-0.9534279047,-0.110072730306])
	init_course_corners1.append([0.408943993412,-0.932134783456,-0.11195746082])
	init_course_corners1.append([0.411540471994,-0.633150074684,-0.030064641006])
	init_course_corners1.append([0.65999189727,-0.641701715204,-0.0277294285236])
	init_course_corners1.append([0.715563023545,-0.499392455919,-0.0293937940559])
	
	init_course_corners2 = []
	init_course_corners2.append([0.65999189727,-0.641701715204,-0.0277294285236])
	init_course_corners2.append([0.411540471994,-0.633150074684,-0.030064641006])
	init_course_corners2.append([0.408943993412,-0.932134783456,-0.11195746082])
	init_course_corners2.append([0.0495882998644,-0.9534279047,-0.110072730306])
	init_course_corners2.append([0.0131915019168,-0.736565979572,-0.0200761952404])

	init_course_arg = go_to.joint_angle_arg(joint_angles = [-0.984604492188,0.0925224609375,-2.038609375,0.634586914063,-1.15381152344,-1.73773730469,1.73496386719])
	init_course_bckwrds_arg = go_to.joint_angle_arg(joint_angles = [-1.73605566406,0.14494140625,-2.03089355469,0.863700195312,-1.16269433594,-1.78790917969,0.785041015625])

	second_course_arg = go_to.joint_angle_arg(joint_angles = [-0.984604492188,0.0925224609375,-2.038609375,0.634586914063,-1.15381152344,-1.73773730469,1.73496386719])
	second_course_bckwrds_arg = go_to.joint_angle_arg(joint_angles = [-1.73605566406,0.14494140625,-2.03089355469,0.863700195312,-1.16269433594,-1.78790917969,0.785041015625])

	zeroG_endpoint_constraints = zeroG.constrained_arg(orientation_z=True, in_endpoint_frame = True)
	zeroG_all_constraints = zeroG.constrained_arg(orientation_x = False, orientation_y = False, orientation_z = False, position_x = False, position_y = False, position_z = False)
	zeroG_ori_constraints = zeroG.constrained_arg(orientation_x=False, orientation_y=False, orientation_z=False, position_x=True, position_y=True, position_z=True)
	zeroG_pos_constraints = zeroG.constrained_arg(orientation_x=True, orientation_y=True, orientation_z=True, position_x=False, position_y=False, position_z=False)
	zeroG_truly_no_constraints = zeroG.constrained_arg(orientation_x=True, orientation_y=True, orientation_z=True, position_x=True, position_y=True, position_z=True)
	zeroG_xyplane = zeroG.constrained_arg(plane_horizontal = True)

	listener()

#go_to.joint_angles(["-q -0.2 0.1 0.1 0.2 -0.3 0.2 0.4 -s 0.9 -a 0.1"])
#waypoint.playback(["-s 0.1 -a 0.1"])
#navigator.right()