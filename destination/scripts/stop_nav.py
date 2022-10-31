#!/usr/bin/env python
import rospy, actionlib, sys
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point


# Function that stops navigation of Fetch
# Input: 'data', msg containing the request of a button press from the user input node
def stop(data):
    # initiate shutdown dequence if the emergency stop button is pressed
    if (data.data == "stop emergency"):
        rospy.loginfo("Emergency navigation stop node recieved request")

        # Stop navigation
        # Define a client to send goal requests to the move_base server through a SimpleActionClient
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Wait for the action server to come up
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
                rospy.loginfo("Waiting for the move_base action server to come up")

        goal = MoveBaseGoal()

        # Set up the frame parameters
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Moving towards goal
        goal.target_pose.pose.position =  Point(0,0,0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Stoping Navigation")
        ac.send_goal(goal)

        ac.wait_for_result(rospy.Duration(30))

        # Turn off
        rospy.loginfo("Emergency navigation stop node finished request")
        rospy.signal_shutdown("Emergency Navigation Stop Selected")
        sys.exit(0)



if __name__ == '__main__':
    rospy.init_node('stop_navigation', anonymous=True)
    
    # Wait for a button press
    while not rospy.is_shutdown():
        rospy.Subscriber("user_input", String, stop)
