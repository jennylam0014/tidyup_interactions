#!/usr/bin/env python
from logging import INFO
import rospy, sys, actionlib, tf
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from control_msgs.msg import PointHeadAction, PointHeadGoal, GripperCommandAction, GripperCommandGoal
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import pi
from moveit_msgs.msg import MoveItErrorCodes
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest
import threading, collections, Queue, os, os.path
from interaction_functions import * 
import time, logging
from datetime import datetime
import deepspeech 
import pyaudio
import wave
import webrtcvad
from halo import Halo
from scipy import signal

rospy.loginfo("Fetch is starting up...")

# Initialise the sound client so Fetch can speak
sc = SoundClient(blocking=True)
bc = SoundClient()
rospy.sleep(0.5)

# Fetch starts in a not tidying, providing assistance state
tidy = False
assistance = True


# Variable for Fetch to continue when stopped
cont = False

# Scaling factor for manipulation speed
velocity_scale = 0.5

# Initialise Tables and Storage Containers
class destination:
    def __init__(self, location, name, objects =1):
        self.location = location
        self.name = name
        self.objects = objects
        self.empty = False

# For Fetch1077 Map5
#Table1 = destination([7.2,11.3,pi-0.3],'The First Table')
#Table2 = destination([7.16,9.5,-pi/1.2],'The Second Table')
#Tables = [Table1,Table2]
#RedStorage = destination([8.05,11.96,pi/2],'The Red Storage',1)
#BlueStorage = destination([8.59,12,pi/2],'The Blue Storage',1)
#home = destination([-10.326, 0.236,pi-0.3],'My Home')

# For Fetch1080 Map5

Table1 = destination([2.15,1.05,-pi/4],'The First Table')
Table2 = destination([2.80,2.35,0],'The Second Table')
Tables = [Table1, Table2]
RedStorage = destination([-0.70,1.3,-3*pi/4],'The Red Storage',2)
BlueStorage = destination([-0.09,0.398,-3*pi/4],'The Blue Storage',1)
YellowStorage = destination([0.30,0.14,-3*pi/4],'The Yellow Storage',1)
GreenStorage = destination([-0.33,0.80,-3*pi/4],'The Green Storage',1)

home = destination([1.25, 2.7,-pi/4],'My Home')


# Function that controls the direction the head/camera is looking
# Inputs: 'x,y,z', floats that are the coordinates relative to the base that Fetch will look
def head_tilt(x,y,z):
    rospy.loginfo("Fetch is looking at: " + str(x) + ', ' + str(y) + ', ' + str(z))
    head_client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
    rospy.sleep(0.5)
    head_client.wait_for_server()
    goal = PointHeadGoal()
    goal.target.header.stamp = rospy.Time.now()
    goal.target.header.frame_id = "base_link"
    goal.target.point.x = x
    goal.target.point.y = y
    goal.target.point.z = z
    head_client.send_goal(goal)
    head_client.wait_for_result()
    rospy.sleep(0.5)



# Function that controls the gripper
# Input: 'position', float ranging from 0-0.1 and is the distance the gripper will open in meters
def gripper(position):
    rospy.loginfo("Fetch is adjusting gripper to position: " + str(position))
    gripper = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
    rospy.sleep(0.5)
    gripper.wait_for_server()
    goal = GripperCommandGoal()
    goal.command.max_effort = 50
    goal.command.position = position
    gripper.send_goal(goal)
    gripper.wait_for_result()
    rospy.sleep(0.5)



# Function that controls the torso, arm tuck and initiates collision geometry 
# Input: 'height', float indicating the desired height of the torso in meters
# Input: 'pick', boolean value representing if the collision geometry for picking an object should be initiated
def torso(height,state):
    rospy.sleep(0.5)
    # Create move group interface for Fetch
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    planning_scene = PlanningSceneInterface("base_link")
    rospy.sleep(0.5)
    # Define collision geometry in relation to base_link
    rospy.loginfo("Setting up collision geometry")
    # Ground
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    # Table
    planning_scene.removeCollisionObject("table")
    # Account for objects on table
    planning_scene.removeCollisionObject("upper_table")
    # Account for slight rotation variance in navigation
    planning_scene.removeCollisionObject("LT_rotation")
    planning_scene.removeCollisionObject("RT_rotation")
    # Wall behind table
    #planning_scene.removeCollisionObject("back_wall")
    planning_scene.removeCollisionObject("place_wall")
    # Fetch base
    planning_scene.removeCollisionObject("base")
    planning_scene.removeCollisionObject("base_s1")
    planning_scene.removeCollisionObject("base_s2")


    planning_scene.removeCollisionObject("torso")
    planning_scene.removeCollisionObject("torso_s1")
    planning_scene.removeCollisionObject("torso_s2")



    planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)
    #planning_scene.addBox("back_wall", 0.1,3,3,1,0,1.25)
    planning_scene.addBox("base", 0.34,0.50,0.05,0.13,0,0.36)
    planning_scene.addBox("base_s1", 0.50,0.001,0.35,0,0.265,0.25)
    planning_scene.addBox("base_s2", 0.50,0.001,0.35,0,-0.265,0.25)


    #planning_scene.addBox("torso", 0.001,0.33,0.5,-0.1,0,0.5)
    planning_scene.addBox("torso_s1", 0.20,0.002,0.3,-0.15,-0.2,0.3)
    planning_scene.addBox("torso_s2", 0.20,0.002,0.3,-0.15,0.2,0.3)

    # Collision geometry relevant for picking an object
    if state == 1:
        planning_scene.addBox("table", 1,3,0.75,0.8,0,0.38) 
        planning_scene.addBox("LT_rotation", 0.03,1.25,0.8,0.285,-0.875,0.4) 
        planning_scene.addBox("RT_rotation", 0.03,1.25,0.8,0.285,0.875,0.4) 
        planning_scene.addBox("upper_table", 1,3,0.03,0.8,0,0.8)
    elif state == 2: 
        planning_scene.addBox("table", 1,3,0.75,0.8,0,0.38) 
        planning_scene.addBox("LT_rotation", 0.03,1.25,0.8,0.285,-0.875,0.4) 
        planning_scene.addBox("RT_rotation", 0.03,1.25,0.8,0.285,0.875,0.4) 
    elif state == 3:
        #planning_scene.addBox("back_wall", 0.3,3,3,1,0,1.25)
    rospy.sleep(0.5)

    # Arm tuck joint positions
    rospy.loginfo("Fetch is adjusting its arm and/or torso")
    joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    pose = [height, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]

    # Plans the joints in joint_names to angles in pose
    move_group.moveToJointPosition(joints, pose, wait=True, max_velocity_scaling_factor=velocity_scale)
    rospy.sleep(0.5)



# Function that performs the identification and picking of an object
# Output: 'empty', boolean value indicating if a table has objects on it
# Output: 'colour', string indicating the colour of the object
def pick(colour_fetch):


    rospy.sleep(1)
    # Ensure gripper is open and raise torso slightly
    torso(0.05,state = 2)
    gripper(0.1)

    # Search for an object
    rospy.loginfo("Fetch is searching for an object")
    angles = [0,0.22,0,-0.22]#,-0.15,0]

    for angle in angles:
        head_tilt(1,angle,0.45)
        rospy.sleep(4)


        #for x in range(2):

        # Inform marker node to look for objects
        pick_pub.publish(colour_fetch)
        
        try:
            # Request marker information 
            
            info = rospy.wait_for_message("markers", String, 10)
            info = info.data
            num_markers = int(info[0])
            ID = int(info[2])


        except:
            # No information recieved 
            return True, 'N/A'

        # If Fetch has found a marker stop searching
        if num_markers:
            break

     
    #If the table is empty return status
    if (num_markers == 0):
        gripper(0.1)
        #torso(0.05,state == 1)
        head_tilt(1,0,1)
        return True, 'N/A'
    # Removed because it may have not seen others
    #elif (num_markers == 1):
    #    empty = True               
    else:
        empty = False 
    
    if (ID==1):
        colour = 'red'
        RedStorage.objects -= 1
        rospy.loginfo("Fetch has found a red object")
    elif (ID==2):
        colour = 'blue'
        BlueStorage.objects -= 1
        rospy.loginfo("Fetch has found a blue object")
    elif(ID ==3):
        colour = 'yellow'
        YellowStorage.objects -= 1
        rospy.loginfo("Fetch has found a yellow object")
    elif(ID == 4):
        colour = 'green'
        GreenStorage.objects -= 1
        rospy.loginfo("Fetch has found a green object")


    # Create move group interface for Fetch
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    planning_scene = PlanningSceneInterface("base_link")
    rospy.sleep(0.5)
    # Get gripper to object transform
    rospy.loginfo("Fetch requested object transforms")
    listener = tf.TransformListener()
    time_now=rospy.Time()
    listener.waitForTransform("object","base_link", time_now,rospy.Duration(1))
    (trans, rot) = listener.lookupTransform("base_link","object", time_now)
    rot = euler_from_quaternion(rot)

    # Pick object from above (1.5707 ~ 90 degree rotation of gripper in y axis)
    q = quaternion_from_euler(0,1.5707,rot[2])
    gripper_poses = [Pose(Point(trans[0], trans[1], trans[2]+0.1),Quaternion(q[0],q[1],q[2],q[3])),Pose(Point(trans[0]-0.005, trans[1], trans[2]+0.01),Quaternion(q[0],q[1],q[2],q[3]))]    

    # Construct a "pose_stamped" message as required by moveToPose
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'

    # Move gripper frame to the object
    rospy.loginfo("Fetch has started picking the object up")
    for pose in gripper_poses:
        gripper_pose_stamped.header.stamp = rospy.Time.now()
        gripper_pose_stamped.pose = pose
        move_group.moveToPose(gripper_pose_stamped, 'gripper_link', max_velocity_scaling_factor=velocity_scale)
        planning_scene.removeCollisionObject("upper_table")
        rospy.sleep(1)

    # Close gripper on object
    gripper(0)
    rospy.sleep(2)
    rospy.loginfo("Fetch has the object")

    # Tuck arm
    #rospy.loginfo("Fetch is adjusting its arm and/or torso")
    #joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                #  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    #pose = [0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
    #move_group.moveToJointPosition(joints, pose, wait=True, max_velocity_scaling_factor=velocity_scale)
    #rospy.sleep(2)


    gripper_pose_stamped.header.stamp = rospy.Time.now()
    gripper_pose_stamped.pose = Pose(Point(trans[0], trans[1], trans[2]+0.15),Quaternion(q[0],q[1],q[2],q[3]))#moving fetch up
    move_group.moveToPose(gripper_pose_stamped, 'gripper_link', max_velocity_scaling_factor=velocity_scale)
    rospy.sleep(1)
    torso(trans[2]+0.20,state = 1)
    rospy.sleep(2)


    #gripper_pose_stamped.header.stamp = rospy.Time.now()
    #gripper_pose_stamped.pose = Pose(Point(trans[0], trans[1], trans[2]+0.15),Quaternion(q[0],q[1],q[2],q[3]))
    #move_group.moveToPose(gripper_pose_stamped, 'gripper_link', max_velocity_scaling_factor=velocity_scale)
    # planning_scene.attachCube("object",0.05,0,0,0,'gripper_link','gripper_link')
    torso(0.05,state = 1)
    head_tilt(1,0,1)
    rospy.loginfo("The object is ready for transportation")

    return empty, colour
 

# Function that places an object in the storage container
def place():

    torso(0.05,state = 3)
    rospy.sleep(0.1)


    #head_tilt(1,0,0.5)

    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    planning_scene = PlanningSceneInterface("base_link")

    joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    pose =  [0.05, 1.06, 1.40, -1.77, 1.52, 0.0, 0.0, 0.0]
    move_group.moveToJointPosition(joints, pose, wait=True, max_velocity_scaling_factor=velocity_scale)
    rospy.sleep(0.5)

    #q = quaternion_from_euler(0,1.5707,0)    rospy.loginfo("callback")
    #gripper_pose_stamped.header.stamp = rospy.Time.now()
    #gripper_pose_stamped.pose = pose

    # Move gripper frame to the pose specified
    #move_group.moveToPose(gripper_pose_stamped, 'gripper_link', max_velocity_scaling_factor=velocity_scale)
    #ospy.sleep(2)

    #rospy.sleep(1)
    gripper(0.1)
    rospy.sleep(1)

    #planning_scene.removeAttachedObject("object")
    #planning_scene.removeCollisionObject("object")
    pose =  [0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
    move_group.moveToJointPosition(joints, pose, wait=True, max_velocity_scaling_factor=velocity_scale)
    rospy.sleep(0.5)
    #head_tilt(1,0,1)
    

# Function that allows Fetch to navigate to a location 
# Input: 'destination', class containing the requested location to navigate to
# Output: boolean value indicating if Fetch reached its location
def navigate(destination):
    sc.say('I am moving to ' + destination.name,'voice_us1_mbrola')
    rospy.loginfo("Fetch is moving to " + str(destination.name))

    # Define a client to send goal requests to the move_base server through a SimpleActionClient
    global ac
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    ac.wait_for_server(rospy.Duration(5))

    # Set up the frame parameters
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Moving towards goal
    goal.target_pose.pose.position =  Point(destination.location[0],destination.location[1],0)
    orientation = Quaternion(*quaternion_from_euler(0,0,destination.location[2]))
    goal.target_pose.pose.orientation = orientation
    ac.send_goal(goal)
    ac.wait_for_result(rospy.Duration(30))

    # Return goal status
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("Fetch reached " + destination.name)
        return True
    else:
        rospy.loginfo("Fetch could not reach " + str(destination.name))
        return False



def conversation_pub(): 
    convo_pub.publish("start") 
    colour_flag = True 
    info = 'null'

    while colour_flag == True:

        # Request marker information        
        try:
            info = rospy.wait_for_message("colour", String, 10)
            colour_fetch = info.data[0]
            colour_human = info.data[1]

        except:
            if info != 'null':
                colour_flag = False 
                
    rospy.loginfo("Fetch is picking up: %s" %colour_fetch)
    rospy.loginfo("Human is picking up: %s" %colour_human)

    return colour_fetch
         
# Function that controls the tidying sequence
def clean():
    # Loop through tables
    for table in Tables:
        # If the table is not empty go to it
        while not table.empty:
            response1 = navigate(table)

            # Stop tidying if requested
            if not assistance:
                break

            # If table has been reached find and/or pick object up
            if response1 == True:
                empty, colour = pick(colour_fetch)
                table.empty = empty

                # Navigate to appropriate storage container
                if colour == 'red':
                    response2 = navigate(RedStorage)
                    colour_objects = RedStorage.objects
                elif colour == 'blue':
                    response2 = navigate(BlueStorage)   
                    colour_objects = BlueStorage.objects
                elif colour == 'yellow':
                    response2 = navigate(YellowStorage)
                    colour_objects = YellowStorage.objects
                elif colour == 'green':     
                    response2 = navigate(GreenStorage)
                    colour_objects = GreenStorage.objects
                else:
                    sc.say('I could not find any objects here','voice_us1_mbrola')
                    colour_objects = True
                    break

                # If storage container has been reached, place object in it
                if response2 == True:
                    place()
                    rospy.loginfo("Fetch placed object")

                # If storage container not reached, ask for someone to take the object
                else:
                    sc.say('Sorry I can not reach the container, please take the object from me','voice_us1_mbrola')
                    ac.cancel_goal()
                    gripper(0.1)
                    sc.say('Press continue once you have taken it','voice_us1_mbrola')
                    while not cont:
                        rospy.sleep(1)
                    global cont
                    cont = False
                break
            # If table can not be reached, try next one
            else:
                sc.say('Sorry I can not reach the table','voice_us1_mbrola')
                colour_objects = True 
                # Check table again later
                if len(Tables) < 5:
                    Tables.append(table)
                    # If its the second table return home before trying again
                    if table.name == 'The Second Table':
                        navigate(home)
                break

            # Update status on tidying if a set of colours is complete
            if not colour_objects:
                sc.say('All %s objects are complete' %colour,'voice_us1_mbrola')                              
            break
        
        # Stop tidying if requested
        if not assistance:
            break

        # All objects are tidied, break loop
        if (not colour_objects):
            break

    rospy.sleep(1)
    sc.say('I have finished tidying','voice_us1_mbrola')
    rospy.sleep(1)

    # Tidying has completed, return to home and turn off
    navigate(home)
    close_pub.publish('close')
    rospy.sleep(1)
    rospy.loginfo("Fetch has finished tidying")
    rospy.signal_shutdown("Finished Tidying")
    sys.exit(0)


# Callback function for user input node (button pressed on the GUI)
# Input: 'data', string indicating which button has been pressed
def exception_action(data):
    # Tell Fetch to stop moving
    if (data.data == "stop emergency"):
        rospy.loginfo("User clicked the emergency stop")
        rospy.signal_shutdown("Emergency Stop Requested")
        sys.exit(0)

    # Tell Fetch to continue tidying
    elif (data.data == "continue"):
        rospy.loginfo("User asked Fetch to continue")
        global cont
        cont = True


if __name__ == '__main__':
    # Setup node, publishers and subscribers
    rospy.init_node('Main_Control', disable_signals=True)
    pick_pub = rospy.Publisher('pick', String, queue_size=10)
    close_pub = rospy.Publisher('close', String, queue_size=10)
    rospy.Subscriber("user_input", String, exception_action)
    convo_pub = rospy.Publisher('convo', String, queue_size = 10)
    rospy.Subscriber("button", String, exception_action)
    rospy.sleep(4)


    # Start Fetch in default position
    #navigate(home)
    #rospy.sleep(2)
    torso(0.05, state =3)
    #head_tilt(1,0,1)
    #gripper(0.1)
    #sc.say('Which colour would you like to collect?','voice_us1_mbrola')
    ##rospy.sleep(1)
   # sc.say('Would you like to clean together?','voice_us1_mbrola')

    
    #Calling conversation function 
    #global colour_fetch, colour_human, colour_ID
    #colour_fetch = conversation_pub() 
    #rospy.loginfo('Fetch is ready to start tidying')
    # Start tidying when prompted
    #while not rospy.is_shutdown():
    #    clean()
