#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray

def talker():
    pub = rospy.Publisher("quori_body_face", Float64MultiArray, queue_size=1)
    rospy.init_node('quori_neutral', anonymous=True)
    rate = rospy.Rate(0.25)
    body_face_msg = Float64MultiArray()
    body_face_msg.data = [0, 1, 2]
    while not rospy.is_shutdown():
        pub.publish(body_face_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass