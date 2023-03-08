#! /usr/bin/env python3
import rospy

# import numpy as np
from rospy.numpy_msg import numpy_msg

# from tf2_msgs.msg import {TFMessage}
# from nav_msgs.msg import {Odometry}
# from sensor_msgs.msg import {PointCloud2, LaserScan, PointField}
from geometry_msgs.msg import Twist, TransformStamped, Polygon, PolygonStamped, Point32
from std_msgs.msg import String, Float64

class test:
    def __init__(self):
        # self.args = args

        # rospy.Subscriber("{topic_name}", {message_type}, self.callback_function)

        self.publisher =  rospy.Publisher("quori/base_controller/cmd_vel", Twist, queue_size = 10)

    #def callback_function(self, message):
    #    return None

    def publishing_function(self, speedx, speedy):
        new_message = Twist()
        #new_message.header.stamp = rospy.Time.now()
        # fill in rest of values

        new_message.linear.x = speedx
        new_message.linear.y = speedy
        new_message.linear.z = 0
        new_message.angular.x = 0
        new_message.angular.y = 0
        new_message.angular.z = 0

        self.publisher.publish(new_message)

if __name__ == '__main__':
    rospy.init_node('test')

    args = None
    node = test()

    # if only triggered by callback functions
    # rospy.spin()

    # otherwise
    rate = rospy.Rate(100)
    startTime = rospy.Time.now()
    i = 0
    while not rospy.is_shutdown():
        #if(rospy.Time.now() >= startTime + rospy.Duration.from_sec(2)):
        #  node.publishing_function(0)
        #else:
        if(rospy.Time.now() >= startTime + rospy.Duration.from_sec(i+6)):
            node.publishing_function(0, -0.5)
            i += 8
        elif(rospy.Time.now() >= startTime + rospy.Duration.from_sec(i+4)):
            node.publishing_function(-0.5, 0)
        elif(rospy.Time.now() >= startTime + rospy.Duration.from_sec(i+2)):
            node.publishing_function(0, 0.5)
        else:
            node.publishing_function(0.5, 0)
        rate.sleep()
        # startTime = rospy.Time.now()
