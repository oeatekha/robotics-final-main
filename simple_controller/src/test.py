#!/usr/bin/env python

# 2.12 Final Project
# Team R2

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Twist
from math import atan2
import sys

from user_input.msg import Velocity, JoyCmd

def cback(msg):
    print(msg.data, not msg.data)

rospy.init_node('TagChaser', anonymous=True)

handoff_pub = rospy.Publisher("/robot_arrived", Bool, queue_size=1)
handoff_sub = rospy.Subscriber("/robot_arrived", Bool, cback)

if __name__ == '__main__':
    test = Bool()
    test.data = False
    print(test)
    handoff_pub.publish(test)
    while not rospy.is_shutdown():
        handoff_pub.publish(test)
