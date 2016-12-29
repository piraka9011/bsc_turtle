#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import tf_conversions

def odomCB(msg):
    orientation = msg.pose.pose.orientation
    transform = tf_conversions.transformations
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    (roll, pitch, yaw) = transform.euler_from_quaternion(quaternion)
    rospy.loginfo("Yaw: %f", yaw)

rospy.init_node('driftTest')
rospy.Subscriber('/odom', Odometry, odomCB)
rospy.spin()
