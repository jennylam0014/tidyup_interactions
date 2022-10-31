#!/usr/bin/env python
import rospy, sys
from std_msgs.msg import String
from actionlib_msgs.msg import *
from moveit_python import MoveGroupInterface


# Function that stops manipulation of Fetch
# Input: 'data', msg containing the request of a button press from the user input node
def stop(data):
    # initiate shutdown dequence if the emergency stop button is pressed
    if (data.data == "stop emergency"):
        rospy.loginfo("Emergency manipulation stop node recieved request")

        # Stop manipulation
        move_group = MoveGroupInterface("arm_with_torso", "base_link")
        rospy.loginfo("Stopping Manipulation")
        rate = rospy.Rate(5.0)

        for x in range(0,100):
            move_group.get_move_action().cancel_all_goals()
            rate.sleep()

        # Turn off
        rospy.loginfo("Emergency manipulation stop node finished request")
        rospy.signal_shutdown("Emergency Manipulation Stop Selected")
        sys.exit(0)



if __name__ == '__main__':
    rospy.init_node('stop_manipulation', anonymous=True)
    
    # Wait for a button press
    while not rospy.is_shutdown():
        rospy.Subscriber("user_input", String, stop)
