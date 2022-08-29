#! /usr/bin/env python

# 2.12 Final Project
# Phillip Daniel April 2021

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

# These global variables are the pose of the mobile robot
x= 0.0
y=0.0
theta=0.0

def newOdom(msg):
	global x
	global y
	global theta

	x=msg.pose.pose.position.x
	y=msg.pose.pose.position.y


	rot_q=msg.pose.pose.orientation
	(roll, pitch, theta)=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])



def searchForAprilTags(x,y,theta):
	
	class myClass(object):
		pass
	
	# Tag one
	# Visibility information
	tag1Vis=myClass()
	tag1Vis.label='tag1'
	# Tag Position Relative to Global (0,0)
	tag1Vis.x=-3.85
	tag1Vis.y=-1.79
	# Robot Position Bounding Box
	tag1Vis.xMin=tag1Vis.x
	tag1Vis.xMax=10
	tag1Vis.yMin=-100
	tag1Vis.yMax=100

	
	# Tag one
	# Visibility information
	tag2Vis=myClass()
	tag2Vis.label='tag2'
	# Tag Position Relative to Global (0,0)
	tag2Vis.x=-3.3
	tag2Vis.y=-7.0
	# Robot Position Bounding Box
	tag2Vis.xMin=-5.5
	tag2Vis.xMax=10
	tag2Vis.yMin=-100
	tag2Vis.yMax=100

	
	# Tag one
	# Visibility information
	tag3Vis=myClass()
	tag3Vis.label='tag3'
	# Tag Position Relative to Global (0,0)
	tag3Vis.x=-6.66
	tag3Vis.y=-6.72
	# Robot Position Bounding Box
	tag3Vis.xMin=tag3Vis.x
	tag3Vis.xMax=0
	tag3Vis.yMin=tag2Vis.y
	tag3Vis.yMax=-4.5

	
	# Tag one
	# Visibility information
	tag4Vis=myClass()
	tag4Vis.label='tag4'
	# Tag Position Relative to Global (0,0)
	tag4Vis.x=-6.23
	tag4Vis.y=-0.19
	# Robot Position Bounding Box
	tag4Vis.xMin=-10
	tag4Vis.xMax=-5.83
	tag4Vis.yMin=-100
	tag4Vis.yMax=100

	
	# Tag one
	# Visibility information
	tag5Vis=myClass()
	tag5Vis.label='tag5'
	# Tag Position Relative to Global (0,0)
	tag5Vis.x=-9
	tag5Vis.y=-4.88
	# Robot Position Bounding Box
	tag5Vis.xMin=-10
	tag5Vis.xMax=-5.83
	tag5Vis.yMin=-4.88
	tag5Vis.yMax=0.0

	

	def computeLineOfSight(tagVis, x, y, theta): #Pass in the class that describes visibility informaiton of each of the April tag. This returns information from each tag that is visible.
		# Robot's blind spot half-angle (Rad)
		cameraFOV=1.12
		thetaBlind=(2*np.pi-cameraFOV)/2
		
		# Compute relative angle between the April Tag and the robot (reference notes)
		V=np.array([tagVis.x-x, tagVis.y-y])
		xHat=np.array([np.cos(theta),np.sin(theta)])
		yHat=np.array([-np.sin(theta),np.cos(theta)])
		alpha=atan2(np.dot(V,yHat), np.dot(V, xHat))


		# Check if the robot is pointed at the tag, and in the proper bounding box to see the tag
		class relativePositionFromAprilTag(object):
			pass

		relPos=relativePositionFromAprilTag() 


		if x>tagVis.xMin and x<tagVis.xMax and y>tagVis.yMin and y<tagVis.yMax and ( alpha > thetaBlind or alpha < -thetaBlind ):
			relPos.label=tagVis.label
			relPos.x=-V[0]
			relPos.y=-V[1]
		else:
			relPos.label='no tag'
		return relPos

	relPos = myClass()

	relPos.relPos1=computeLineOfSight(tag1Vis, x, y, theta)
	relPos.relPos2=computeLineOfSight(tag2Vis, x, y, theta)
	relPos.relPos3=computeLineOfSight(tag3Vis, x, y, theta)
	relPos.relPos4=computeLineOfSight(tag4Vis, x, y, theta)
	relPos.relPos5=computeLineOfSight(tag5Vis, x, y, theta)

	return relPos

	
def setSpeeds(xDot, yDot, thetaDot):
	def clamp(num):
		return max(min(num,1),-1)

	xDot=clamp(xDot)
	yDot=clamp(yDot)
	thetaDot=clamp(thetaDot)

	maxSpeed=1.4 #Maximum Linear Speed m/s
	maxRot=.5 #Maximum Angular Velocit rad/s

	speed.linear.x = xDot*maxSpeed
	speed.linear.y = yDot*maxSpeed
	speed.angular.z = thetaDot*maxRot

	return speed



rospy.init_node("speed_controller")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

speed = Twist()

r = rospy.Rate(4)


# This is a way to specify a location with respect to the environment's coordinate system
goal = Point()
goal.x=-1
goal.y=-1

# This is the main loop
while not rospy.is_shutdown():

	# This is how to set the speed of the mobile robot. Choose a value between -1 < x < 1 for each speed. 1 Corresponds to the maximum speed.
	xDot=0
	yDot=0
	thetaDot=.75
	speed=setSpeeds(xDot, yDot, thetaDot)

	# This function searches for April tags based on the global pose of the robot
	tagInfo = searchForAprilTags(x,y,theta)


	if tagInfo.relPos1.label != 'no tag':
		# The relative position of the mobile robot with respect to each April tag that it sees is given by 'tagInfo.relPos#.x/y'
		print('Robot\'s relative position: ',tagInfo.relPos1.x, tagInfo.relPos1.y)

		# If a tag is visible, this will return 'tag#' otherwise it returns 'no tag'
		print('Tag label = ', tagInfo.relPos1.label)

	# You will also be able to compute the orientation of the mobil robot.
	print('Robot orientation = ', theta)

	# This sends the speed command to the mobile robot
	pub.publish(speed)

	# This sets the loop rate
	r.sleep()
