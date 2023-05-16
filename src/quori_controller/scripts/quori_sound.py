#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import String
import pyttsx3

def callback(data):
    engine.say(data.data)
    engine.runAndWait()


def listener():
    rospy.init_node('quori_sound', anonymous=True)

    engine = pyttsx3.init()

    rospy.Subscriber("quori_sound", String, callback)

    rospy.spin()

if __name__ == '__main__':

    listener()