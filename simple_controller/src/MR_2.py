import rospy
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
import sys
import os

from apriltag_ros.msg import AprilTagDetectionArray
from user_input.msg import Velocity, JoyCmd
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# These global variables are the pose of the mobile robot
x = 0.0
y = 0.0

# ROTATION STATES FOR MAP BUILDING TELLS WHICH SHOULD BE SUBTRACTED OR ADDED TO LAST FRAME
rightTurn_x = [0, 1]
rightTurn_z = [-1, 0]
leftTurn_x = [0, -1]
leftTurn_z = [1, 0]

# identity matrix
turn_x = [1, 0]
turn_z = [0,1]

theta = 0.0
tagPose = None
tagPoseInitial = PoseStamped()
tagEuler = None
tagID = None
relPos = None
currPose = None
path = Path()
timeOfFlight = 0.0

## EXTRA STUFF
x_cordinates = []
y_cordinates = []
breathing_count = 0
distance_OA = 2
toClose = 0 
pose_count = 0

prevPose = PoseStamped()
prevPose.pose.position.x = 0.0
prevPose.pose.position.y = 0.0
prevPose.pose.position.z = 0.0
prevPose.pose.orientation.x = 0.0
prevPose.pose.orientation.y = 0.0
prevPose.pose.orientation.z = 0.0
prevPose.pose.orientation.w = 1.0

# Turn Rate Max
turn_rate = 0.3

# Version number
rev = 'v.6.0'
print('Starting revision: ', rev)

# Teleop only
teleop_only = "teleop" in sys.argv

# Obstacle Avoidace Tag 2
obstacle_only = "obstAvoid2" in sys.argv
# Don't move, just range
no_move = "nomove" in sys.argv

# Tighten Up only
tighten_only = "tighten" in sys.argv

# Tighten Up (Fast) only
tighten_fast_only = "tightfast" in sys.argv

# Select state to start at (else, start at 0)
if "state" in sys.argv:
    if len(sys.argv) > sys.argv.index("state")+1:
        robot_state = int(sys.argv[sys.argv.index("state")+1])
    else:
        print('No State number entered')
        robot_state = 0
else:
    robot_state = 0

# State Machine
'''
-1 - Stop
0 - Begin, search for Tag 1
1 - Move to Tag 1
2 - Search for Tag 2
3 - Move to Tag 2
4 - Search for Tag 3
5 - Move to Tag 3
6 - Search for Tag 4    
7 - Move to Tag 4
8 - Search for Tag 5
9 - Move to Tag 5
10 - Wait for Bottle
11 - Backup from Dock
12 - Search for Tag 4
13 - Move away from Tag 4
14 - Search for Tag 3
15 - Move away from Tag 3
'''

# Command message
jcv = JoyCmd()
jcv.axis1 = 0.0
jcv.axis2 = 0.0
jcv.axis3 = 0.0
jcv.btn1 = 0.0
jcv.btn2 = 0.0
jcv.btn3 = 0.0

# Teleop request
teleop = JoyCmd()
teleop.axis1 = 0.0
teleop.axis2 = 0.0
teleop.axis3 = 0.0
teleop.btn1 = 0.0
teleop.btn2 = 0.0
teleop.btn3 = 0.0

# Autonomous request
auton = JoyCmd()
auton.axis1 = 0.0
auton.axis2 = 0.0
auton.axis3 = 0.0
auton.btn1 = 0.0
auton.btn2 = 0.0
auton.btn3 = 0.0

# Robot Docked Message
dockedMsg = Bool()
dockedMsg.data = False

# Rotation direction
## 1 = turn right; -1 = turn left
rot_dir = 1

# Indicators for manual control
teleopTransRequest = False
teleopRotRequest = False

# Robot Docked
robotDocked = False

# Arm task complete
armDone = False

def teleopTranslationCallback(msg):
    '''
    Get user input commands for front/back and side to side movement
    Request manual control
    '''
    global teleopTransRequest

    teleop.axis2 = msg.linear.x
    teleop.axis1 = msg.angular.z

    if msg.linear.x == 0 and msg.angular.z == 0:
        teleopTransRequest = False
    else:
        teleopTransRequest = True

def DistanceDepthCallback(msg):
    "Get Depth from Intel Real Sense"
    global timeOfFlight
    timeOfFlight = float(msg.data)
    # resets the timeofFLight reading

def teleopRotationCallback(msg):
    '''
    Get user input commands for rotational movement
    Request manual control
    '''
    global teleopRotRequest

    teleop.axis3 = msg.angular.z * -1

    if msg.angular.z == 0 and msg.linear.x == 0:
        teleopRotRequest = False
    else:
        teleopRotRequest = True

def handoffCallback(msg):
    '''
    Set ArmDone when arm publishes "finished" message
    '''
    global armDone
    armDone = msg.data

def pointAtTag(targetTagID):
    # Description: Search for the specified tag
    # Return
        # void - Prints a message when we are pointing at a tag
    # Argument
        # targetTagID - The ID of the tag that we wish to point the robot's camera towards

    global rot_dir, turn_rate
    global auton

    rate = turn_rate
    relX = 9999
    pointed = False
    thetaDot = rate * rot_dir
    
    global tagPose
    global tagID

    auton.axis1 = 0.0
    auton.axis2 = 0.0
    auton.axis3 = 0.0
    auton.btn1 = 0.0
    auton.btn2 = 0.0
    auton.btn3 = 0.0

    xHigh = 0.05
    xLow = -0.05

    print(' -- Tag Point --')

    # Spin around and look for the tag 'locatedTag.label.' Stop once we are pointing at it within a small window for error
    if targetTagID == tagID and tagPose != None: # We have found the tag that we were looking for.
        relX=tagPose.pose.pose.position.x

        if relX < xHigh and relX > xLow: # If we are pointing at the tag
            thetaDot = 0
            pointed = True
        else:
            thetaDot = rate * rot_dir
            pointed = False
    else:
        thetaDot = rate * rot_dir
        pointed = False

    print('Target: ', targetTagID, 'TagID: ', tagID, " Relative X: ", round(relX,3), " Pointed: ", pointed)

    auton.axis3 = thetaDot

    if pointed:
        rot_dir = rot_dir * -1
        return True

    return False

def viewedTagRelPos(data):
    # Description: Relative position of the tag 'tagID,' if it is in view
        # Return
            # locatedTag.label - The label for the visible April tags (e.g. 'tag1')
            # locatedTag.relZ - The x-axis position of the April tag with respect to the coordinate system that is attached to the mobile robot
            # locatedTag.relX - The y-axis position of the April tag with respect to the coordinate system that is attached to the mobile robot
        # Argument
            # tagID - The ID of the tag that we wish to point the robot's camera towards

    global tagPose
    global tagID
    global tagEuler
    global relPos
    global turn_x, turn_z


    tagPose = None
    tagID = None
    tagEuler = None
    relPos = None

    print(' -- Tag View --')

    for detections in data.detections:

        #TODO: global distance?

        for idseen in detections.id:
            if idseen == 0 or idseen == 1 or idseen == 2 or idseen == 3 or idseen == 4 or idseen == 5 :
                tagID = idseen
                tagPose = detections.pose

                res = euler_from_quaternion([tagPose.pose.pose.orientation.x, tagPose.pose.pose.orientation.y, tagPose.pose.pose.orientation.z, tagPose.pose.pose.orientation.w])
                tagEuler = res[1]
                print('TagID: ', tagID, ' Position: ', tagPose.pose.pose.position, ' Angle: ', tagEuler)
            else: 
                print('TagID: ', 'Unknown', ' Position: ', tagPose.pose.pose.position, ' Angle: ', tagEuler)

    if tagID is None:
        print('TagID: ', 'None Visible', ' Position: ', '----', ' Angle: ', '----')\
## ADDED
def newPose(ps, ps_init):

    try:
        places_pub = rospy.Publisher('/move_base_simple/places', PoseStamped, queue_size=5)
        
        places = PoseStamped()
        places.header.frame_id = "world"

        global path, prevPose, currPose
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "world"
        #
        
        places.pose.position.x = (tagPose.pose.pose.position.z*int(turn_x[1]) - tagPoseInitial.pose.pose.position.z*int(turn_x[1])) + (tagPose.pose.pose.position.x*int(turn_x[0]) - tagPoseInitial.pose.pose.position.x*int(turn_x[0])) + prevPose.pose.position.x
        places.pose.position.y = 0
        places.pose.position.z = (tagPose.pose.pose.position.z*int(turn_z[1]) - tagPoseInitial.pose.pose.position.z*int(turn_z[1])) + (tagPose.pose.pose.position.x*int(turn_z[0]) - tagPoseInitial.pose.pose.position.x*int(turn_z[0])) + prevPose.pose.position.x
        #truthfully orientation only changes when turning, this is where the theta-dot should potentially be used. 
        places.pose.orientation.x = 0.0
        places.pose.orientation.y = 0.0
        places.pose.orientation.z = 0.0
        places.pose.orientation.w = 1.0
        
        path.poses.append(places)
        places_pub.publish(places)
        path_pub.publish(path)
        currPose = places
    except:
        pass

def checkSize(dist):
    global tooClose

    if(dist < 100):
        tooClose = 1
    else:
        tooClose = 0

    return tooClose

def obstacleAvoidance(distanceObstacle):
    Unexpected = checkSize(distanceObstacle)
    if Unexpected == 1:
        print("SOMETHING IS IN FRONT OF ME")
    #    rospy.sleep(15)
    
## ADDED

def approach(target_dist=0.5, dir=0):
    # Description: Drive within a certian distace of the tag 'targetTagID'
        # Return
            # null
        # Argument
            # targetTagID - The ID of the tag that we wish to point the robot's camera towards
            # dir: 0 = forward, 1 = reverse
    global tagPose
    global auton
    global distance_OA
    global pose_count
    global timeOfFlight
    global tagPoseInitial 
    # ADDED
    

    approached=False

    auton.axis1 = 0.0
    auton.axis2 = 0.0
    auton.axis3 = 0.0
    auton.btn1 = 0.0
    auton.btn2 = 0.0
    auton.btn3 = 0.0

    #DistanceDepthCallback() # call back for depth sub # gives us a float
    relX = 0
    relZ = 0

    lost_tag = False

    print(' -- Tag Approach --')

    if tagPose != None:
        relX=tagPose.pose.pose.position.x
        relZ=tagPose.pose.pose.position.z

        if (pose_count < 1):
        # Basically establish initial pose
            tagPoseInitial = tagPose 
            print("Declared Initial Transformation")
            pose_count =  pose_count + 1
        # if abs(relZ) < 2 meters we will run obstacle avoidance to check if there is anything near us. 
        # we dont have to block fiducial we can just put something in the field of view not blocking the April Tag
        newPose(tagPose, tagPoseInitial) # Callback function to publish map with position of robot. 

        vRel=np.array([relZ,relX])
        relPosNorm=np.linalg.norm(vRel)
        relPosUnitVec=vRel/relPosNorm
        
            
        # Go Forward
        if dir == 0:
            if relPosNorm > target_dist:
                # If closer than half a meter, slow to half speed
                zDot=relPosUnitVec[0] * (1 - 0.5*(relPosNorm < 0.6))
                xDot=relPosUnitVec[1] * (1 - 0.5*(relPosNorm < 0.6))
            else:
                zDot=0
                xDot=0
                approached=True
        # Go Backward
        else:
            if relPosNorm < target_dist:
                zDot=relPosUnitVec[0] * -1
                xDot=relPosUnitVec[1] * -1
            else:
                zDot=0
                xDot=0
                approached=True
	
    if 'relPosNorm' in locals():
        if (abs(relPosNorm) > 2):
            print("OBSTACLE AVOIDANCE OCCURING")
            obstacleAvoidance(timeOfFlight)
            # check to see if there is an obstacle to stop
            # WAIT 15 SECONDS STOP, THEN LOOK FOR A TAG
            # if not lost keep waiting there.

        print('RelX: ', round(relX,3), ' RelZ: ', round(relZ,3), ' RelPosNorm:', round(relPosNorm,3), ' Approached: ',approached) #relPosNorm measured in meters
    else:
        zDot=0
        xDot=0
        lost_tag = True
        print('RelX: ', '----', ' RelZ: ', '----', ' RelPosNorm:', '----', ' Approached: ',approached) #relPosNorm measured in meters

    if not no_move:
        auton.axis1 = xDot
        auton.axis2 = zDot
    else:
        auton.axis1 = 0
        auton.axis2 = 0

##### ADDED  

    # Return if approached, or if not, if the tag is lost  
    if approached:
        pose_count = 0
        prevPose = currPose 

##### ADDED
    return (approached, approached or lost_tag)

def tighten_up():
    # Description: Drive within a certian distace of the tag 'targetTagID'
        # Return
            # null
        # Argument
            # targetTagID - The ID of the tag that we wish to point the robot's camera towards
    global tagPose
    global turn_rate
    global auton
    global relPos

    approached = False

    aligned = False
    pointed = False

    auton.axis1 = 0.0
    auton.axis2 = 0.0
    auton.axis3 = 0.0
    auton.btn1 = 0.0
    auton.btn2 = 0.0
    auton.btn3 = 0.0

    rate = turn_rate

    relX = 0

    xDot = 0
    thetaDot = 0

    lost_tag = False

    print(' -- Tag Tightening Up --')

    if (tagPose is not None) and (tagEuler is not None):

        relX=tagPose.pose.pose.position.x
        relTheta = tagEuler

        xHigh = 0.05
        xLow = -0.05

        tHigh = 0.075
        tLow = -0.075

        #if relPos is not None:
        #    if relPos > 1:
        #        xHigh = 0.1
        #        xLow = -0.1

        ###
        # Turn before strafe
        if relTheta < tHigh and relTheta > tLow: # Turn until square with tag
            thetaDot = 0
            pointed = True

            if relX < xHigh and relX > xLow: # Move sideways until in front of tag
                xDot = 0
                aligned = True
            else:
                xDot = 0.5 * rate * (1*(relX < 0) + -1*(relX > 0))
                thetaDot = 0
                aligned = False

        else:
            thetaDot = 0.5 * rate * (-1*(relTheta < 0) + 1*(relTheta > 0))
            pointed = False

        ###

        ###
        #  Strafe before turn
        '''
        if relX < xHigh and relX > xLow: # Move sideways until in front of tag
            xDot = 0
            aligned = True

            if relTheta < tHigh and relTheta > tLow: # Turn until square with tag
                thetaDot = 0
                pointed = True
            else:
                thetaDot = 0.5 * rate * (-1*(relTheta < 0) + 1*(relTheta > 0))
                pointed = False

        else:
            xDot = 0.5 * rate * (1*(relX < 0) + -1*(relX > 0))
            thetaDot = 0
            aligned = False
        '''
        ### 

        print('RelX: ', round(relX,3), ' RelTheta: ', round(relTheta,3), 'Aligned: ', aligned, 'Pointed: ', pointed)

    else:
        lost_tag = True
        thetaDot = 0
        xDot = 0
        print('RelX: ', '----', ' RelTheta: ', '----', 'Aligned: ', aligned, 'Pointed: ', pointed)

    auton.axis3 = thetaDot
    auton.axis1 = xDot

    # If lined up and pointed are true, evolution is complete
    approached = aligned and pointed

    if no_move:
        auton.axis1 = 0
        auton.axis2 = 0
        auton.axis3 = 0

    # Return approached state, or if not approached, if the tag is lost
    return (approached, approached or lost_tag)

def tighten_up_fast(slide_only=False):
    # Description: Drive within a certian distace of the tag 'targetTagID'
        # Return
            # null
        # Argument
            # targetTagID - The ID of the tag that we wish to point the robot's camera towards
    global tagPose
    global turn_rate
    global auton

    approached = False

    aligned = False
    pointed = False

    if not slide_only:
        auton.axis1 = 0.0
        auton.axis2 = 0.0
    auton.axis3 = 0.0
    auton.btn1 = 0.0
    auton.btn2 = 0.0
    auton.btn3 = 0.0

    rate = turn_rate

    xDot = 0
    thetaDot = 0

    lost_tag = False

    print(' -- Tag Tightening Up --')

    if (tagPose is not None) and (tagEuler is not None):

        relX=tagPose.pose.pose.position.x
        relTheta = tagEuler

        if relX < 0.05 and relX > -0.05: # Move sideways until in front of tag
            xDot = 0
            aligned = True
        else:
            xDot = 0.5 * rate * (1*(relX < 0) + -1*(relX > 0))
            thetaDot = 0
            aligned = False

        if relTheta < 0.05 and relTheta > -0.05: # Turn until square with tag
            thetaDot = 0
            pointed = True
        else:
            thetaDot = 0.5 * rate * (-1*(relTheta < 0) + 1*(relTheta > 0))
            pointed = False

        print('RelX: ', round(relX,3), ' RelTheta: ', round(relTheta,3), 'Aligned: ', aligned, 'Pointed: ', pointed)

    else:
        lost_tag = True
        thetaDot = 0
        xDot = 0
        print('RelX: ', '----', ' RelTheta: ', '----', 'Aligned: ', aligned, 'Pointed: ', pointed)

    if not slide_only:
        auton.axis3 = thetaDot
    auton.axis1 = xDot

    # If lined up and pointed are true, evolution is complete
    approached = aligned and pointed

    if no_move:
        auton.axis1 = 0
        auton.axis2 = 0
        auton.axis3 = 0

    # Return approached state, or if not approached, if the tag is lost
    return (approached, approached or lost_tag)

def robot_stop():
    jcv.axis1 = 0.0
    jcv.axis2 = 0.0
    jcv.axis3 = 0.0
    jcv.btn1 = 0.0
    jcv.btn2 = 0.0
    jcv.btn3 = 0.0
    virtualJoy_pub.publish(jcv)

rospy.init_node('TagChaser', anonymous=True)

apriltag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, viewedTagRelPos)
translation_sub = rospy.Subscriber("/teleop_trans", Twist, teleopTranslationCallback)
rotational_sub = rospy.Subscriber("/teleop_rot", Twist, teleopRotationCallback)
handoff_sub = rospy.Subscriber("/arm_done", Bool, handoffCallback)
print("Subscriber setup")

virtualJoy_pub = rospy.Publisher("/joy/cmd", JoyCmd, queue_size=1)
handoff_pub = rospy.Publisher("/robot_arrived", Bool, queue_size=1)
handoff_pub.publish(dockedMsg)
path_pub = rospy.Publisher('/path', Path, queue_size = 5) #ps = (transformPose(target_frame, ps))
print("Publisher setup")

r = rospy.Rate(30)
print("ROS rate setup")

print("Start loop")

# This is the main loop
try:

    # Latch variable to indicate when a partial state has been completed
    state_latch = False

    while not rospy.is_shutdown():

        #depthDistance = depth_sub() #Produces the Global Variable from the depth_sub program to be used for obstacle avoidance

        if teleop_only or teleopRotRequest or teleopTransRequest:
            print("== Teleoperated ==")

            # Send manual commands to robot
            jcv.axis1 = teleop.axis1
            jcv.axis2 = teleop.axis2
            jcv.axis3 = teleop.axis3
            jcv.btn1 = teleop.btn1
            jcv.btn2 = teleop.btn2
            jcv.btn3 = teleop.btn3

        elif tighten_only:
            print("== Tightening Up ==")
            (tight, lost) = tighten_up()
            print("Tightened: ", tight, " Lost: ", lost)

            # Send autonomous commands to robot
            jcv.axis1 = auton.axis1
            jcv.axis2 = auton.axis2
            jcv.axis3 = auton.axis3
            jcv.btn1 = auton.btn1
            jcv.btn2 = auton.btn2
            jcv.btn3 = auton.btn3

        elif tighten_fast_only:
            print("== Tightening Up (Fast) ==")
            (tight, lost) = tighten_up_fast()
            print("Tightened: ", tight, " Lost: ", lost)

            # Send autonomous commands to robot
            jcv.axis1 = auton.axis1
            jcv.axis2 = auton.axis2
            jcv.axis3 = auton.axis3
            jcv.btn1 = auton.btn1
            jcv.btn2 = auton.btn2
            jcv.btn3 = auton.btn3

        else:
            print("== Autonomous ==")
            print('State: ', robot_state, ' Latch: ', state_latch, ' Docked: ', robotDocked, ' Arm Done: ', armDone)

            if robot_state == -1: # -1 - Stop robot
                robot_stop()

            elif robot_state == 0: #0 - Begin, search for Tag 1
                pointed = pointAtTag(1)
                if pointed:
                    state_latch = True
                if pointed or state_latch:
                    (tighten, lost) = tighten_up()
                    if tighten:
                        robot_state += 1
                        state_latch = False
                        

            elif robot_state == 1: #1 - Move to Tag 1
                (approached, lost) = approach(0.6)
                #tighten_up_fast(slide_only=True)
                if approached:
                    robot_state += 1
                    rot_dir = 1
                    turn_x = rightTurn_x
                    turn_z = rightTurn_z
                elif lost:
                    robot_state -= 1

            elif robot_state == 2: #2 - Search for Tag 2
                pointed = pointAtTag(2)
                if pointed:
                    state_latch = True
                if pointed or state_latch:
                    (tighten, lost) = tighten_up()
                    if tighten:
                        robot_state += 1
                        state_latch = False

            elif robot_state == 3: #3 - Move to Tag 2
                (approached, lost) = approach(0.6)
                #tighten_up_fast(slide_only=True)
                if approached:
                    robot_state += 1
                    rot_dir = 1
                    turn_x = rightTurn_x
                    turn_z = rightTurn_z
                elif lost:
                    robot_state -= 1

            elif robot_state == 4: #4 - Search for Tag 3
                pointed = pointAtTag(3)
                if pointed:
                    state_latch = True
                if pointed or state_latch:
                    (tighten, lost) = tighten_up()
                    if tighten:
                        robot_state += 1
                        state_latch = False

            elif robot_state == 5: #5 - Move to Tag 3
                (approached, lost) = approach(0.6)
                #tighten_up_fast(slide_only=True)
                if approached:
                    robot_state += 1
                    rot_dir = -1
                    turn_x = leftTurn_x
                    turn_z = leftTurn_z
                elif lost:
                    robot_state -= 1

            elif robot_state == 6: #6 - Search for Tag 4
                pointed = pointAtTag(4)
                if pointed:
                    state_latch = True
                if pointed or state_latch:
                    (tighten, lost) = tighten_up()
                    if tighten:
                        robot_state += 1
                        state_latch = False

            elif robot_state == 7: #7 - Move to Tag 4
                (approached, lost) = approach(1)
                #tighten_up_fast(slide_only=True)
                if approached:
                    robot_state += 1
                    rot_dir = -1
                    turn_x = leftTurn_x
                    turn_z = leftTurn_z
                elif lost:
                    robot_state -= 1

            elif robot_state == 8: #8 - Search for Tag 5
                pointed = pointAtTag(5)
                if pointed:
                    state_latch = True
                if pointed or state_latch:
                    (tighten, lost) = tighten_up()
                    if tighten:
                        robot_state += 1
                        state_latch = False

            elif robot_state == 9: #9 - Move to Tag 5
                (approached, lost) = approach(0.3)
                #tighten_up_fast(slide_only=True)
                if approached:
                    robot_state += 1
                    turn_x = leftTurn_x
                    turn_z = leftTurn_z
                elif lost:
                    robot_state -= 1

            elif robot_state == 10: # 10 - Wait for Bottle
                if not robotDocked:
                    robotDocked = True
                    dockedMsg.data = True
                if armDone:
                    robot_state += 1

            elif robot_state == 11: # 11 - Backup
                (approached, lost) = approach(1.5, 1)
                #tighten_up_fast(slide_only=True)
                if approached:
                    robot_state += 1
                    rot_dir = 1
                elif lost:
                    robot_state -= 1

            elif robot_state == 12: # 12 - Search for Tag 4
                pointed = pointAtTag(4)
                if pointed:
                    state_latch = True
                if pointed or state_latch:
                    (tighten, lost) = tighten_up()
                    if tighten:
                        robot_state += 1
                        state_latch = False

            elif robot_state == 13: # 13 - Backup from Tag 4
                (approached, lost) = approach(3.5, 1)
                #tighten_up_fast(slide_only=True)
                if approached:
                    robot_state += 1
                    rot_dir = 1
                    turn_x = rightTurn_x
                    turn_z = rightTurn_z
                elif lost:
                    robot_state -= 1

            elif robot_state == 14: # 14 - Search for Tag 3
                pointed = pointAtTag(3)
                if pointed:
                    state_latch = True
                if pointed or state_latch:
                    (tighten, lost) = tighten_up()
                    if tighten:
                        robot_state += 1
                        state_latch = False

            elif robot_state == 15: # 15 - Backup from Tag 3
                (approached, lost) = approach(2.5, 1)
                #tighten_up_fast(slide_only=True)
                if approached:
                    robot_state += 1
                elif lost:
                    robot_state -= 1

            else:
                robot_stop()

            # Send autonomous commands to robot
            jcv.axis1 = auton.axis1
            jcv.axis2 = auton.axis2
            jcv.axis3 = auton.axis3
            jcv.btn1 = auton.btn1
            jcv.btn2 = auton.btn2
            jcv.btn3 = auton.btn3

        # Publish request to robot
        virtualJoy_pub.publish(jcv)
        handoff_pub.publish(dockedMsg)

        # Pause step
        r.sleep()

        # Clear screen
        os.system('clear')
except KeyboardInterrupt:
    robot_stop()

# Stop robot when ROS shuts down
jcv.axis1 = 0.0
jcv.axis2 = 0.0
jcv.axis3 = 0.0
jcv.btn1 = 0.0
jcv.btn2 = 0.0
jcv.btn3 = 0.0
virtualJoy_pub.publish(jcv)

'''
Possible things to change
- Max Turn Rate

'''