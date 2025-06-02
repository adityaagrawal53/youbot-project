#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import math

def callback(msg):
    # Extract orientation quaternion
    z = msg.pose.pose.orientation.z
    w = msg.pose.pose.orientation.w

    # Convert quaternion to yaw
    yaw = math.atan2(2.0 * (w * z), 1.0 - 2.0 * (z * z))
    yaw_degrees = math.degrees(yaw)

    print("Yaw (heading): {:.2f} degrees".format(yaw_degrees))

rospy.init_node('yaw_printer')
rospy.Subscriber('/odom', Odometry, callback)
rospy.spin()

