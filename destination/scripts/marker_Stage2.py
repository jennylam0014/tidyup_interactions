#!/usr/bin/env python
import rospy, tf
from std_msgs.msg import String
from fiducial_msgs.msg import FiducialTransformArray


# Whether this is a simulation 
simulation = False
# Whether its been asked to pick an object
pick_status = False
count = 0


# Marker coords for simulation
transx = 0.00714426165428
transy = 0.204051632032
transz = 0.55055134137
rotx = 0.00188216955915
roty = 0.864120680408
rotz = -0.498701155419
rotw = -0.0677426358516


# Function to broadcast marker transform and information
# Input: 'msg' is a FiducialTransformArray containing marker information
def pick(msg):
    global pick_status

    # Ensure we catch the updated pick status
    for x in range(5):
        # If we've been notified to pick up an object
        if pick_status:
            # How many markers Fetch can see
            colours = ['red','blue','yellow','green']
            colour_ID = str(colours.index(colour_fetch)+1)
            
            markers = str(len(msg.transforms))
            rospy.loginfo("Detected " + markers + " objects")
            rate = rospy.Rate(10.0)
            # If there is a marker detected
            if len(msg.transforms) > 0:
                for i in range(len(msg.transforms)):
                    ID = str(msg.transforms[i].fiducial_id)
                    if ID == str(colour_ID):
                        marker = msg.transforms[i]
                        trans = marker.transform.translation
                        rot = marker.transform.rotation
                        rospy.loginfo("Broadcasting transforms for ID " + ID)
                         # Broadcast transform and info for 20 seconds
                        br = tf.TransformBroadcaster()
                        for x in range(0,20):
                            markers_pub.publish(markers + " " + ID)
                            br.sendTransform((trans.x,trans.y,trans.z),(rot.x,rot.y,rot.z,rot.w),rospy.Time.now(),"object","head_camera_rgb_optical_frame")
                            rate.sleep()
                            pick_status = False

                        break
                     
                if ID != str(colour_ID):
                    for x in range(0,50):
                        markers = "0"
                        markers_pub.publish(markers + " 0")
                        rate.sleep()
                        pick_status = False
            # If no makers detected
            elif x == 4:
                for x in range(0,50):
                    markers_pub.publish(markers + " 0")
                    rate.sleep()

    # Reset pick status   
    global pick_status
    pick_status = False

# def pick(msg):
#     global count
#     global pick_status
#     # Ensure we catch the updated pick status
#     for x in range(5):
#         # If we've been notified to pick up an object
#         if pick_status:
#             # How many markers Fetch can see
#             markers = str(len(msg.transforms))
#             rospy.loginfo("Detected " + markers + " objects")
#             rate = rospy.Rate(10.0)
            
#             # If there is a marker detected
#             if len(msg.transforms) > 0:
#                 # Take the first marker we see and get its information
#                 marker = msg.transforms[0]   
#                 trans = marker.transform.translation
#                 rot = marker.transform.rotation
#                 ID = str(marker.fiducial_id)
#                 rospy.loginfo("Broadcasting transforms for ID " + ID)

#                 # Broadcast transform and info for 20 seconds
#                 br = tf.TransformBroadcaster()
#                 for x in range(0,20):
#                     markers_pub.publish(markers + " " + ID)
#                     br.sendTransform((trans.x,trans.y,trans.z),(rot.x,rot.y,rot.z,rot.w),rospy.Time.now(),"object","head_camera_rgb_optical_frame")
#                     rate.sleep()
#                 pick_status = False
#                 count = 0

#             # If no makers detected
#             elif count:
#                 for x in range(0,50):
#                     markers_pub.publish(markers + " 0")
#                     rate.sleep()
            
#             # Try Aruco Detect again
#             count += 1


# Callback function to initiate marker broadcast
def callback(msg):
    global pick_status
    pick_status = True

    global colour_fetch
    colour_fetch = msg.data 



# Function to broadcast a simulated marker
def pick_sim(msg):
    rate = rospy.Rate(10.0)
    br = tf.TransformBroadcaster()

    for x in range(0,100):
        markers_pub.publish("1 1")
        br.sendTransform((transx,transy,transz),(rotx,roty,rotz,rotw),rospy.Time.now(),"object","head_camera_rgb_optical_frame")
        rate.sleep()



if __name__ == '__main__':
    # Initiate
    rospy.init_node('marker_sub', anonymous=True)
    markers_pub = rospy.Publisher('markers', String, queue_size=10)

    # Wait for a pick object request
    while not rospy.is_shutdown():
        if simulation:
            rospy.Subscriber('pick', String, pick_sim) 
        else:
            rospy.Subscriber('pick', String, callback)
            rospy.Subscriber("fiducial_transforms", FiducialTransformArray, pick)
        rospy.sleep(0.1) 
