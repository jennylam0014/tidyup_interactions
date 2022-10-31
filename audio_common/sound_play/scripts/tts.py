#!/usr/bin/env python
import rospy
import actionlib
from actionlib_msgs.msg import *
from sound_play.libsoundplay import SoundClient

def talk():
    rospy.init_node('tts', anonymous=False)
    sc = SoundClient()
    rospy.sleep(1)
    #rate = rospy.Rate(1) # Wait for sound client to initialise
    #rate.sleep()
    #path_to_sounds = "/home/jake/jsan_ws/src/audio_common/sound_play/sounds"
    #sc.playWave(path_to_sounds+"say-beep.wav")
    sc.say('Hello World')
    #rate.sleep()

if __name__ == '__main__':
    try:
        talk() # call talk function
        rospy.spin() # Repeat

    except rospy.ROSInterruptException:
        pass
