#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import String

def callback(data):
    os.system('rosrun sound_play say.py "{}"'.format(data.data))


def listener():
    rospy.init_node('quori_sound', anonymous=True)
    rospy.Subscriber("quori_sound", String, callback)

    rospy.spin()

if __name__ == '__main__':

    listener()