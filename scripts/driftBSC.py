#!/usr/bin/env python

# ROS Lib
from geometry_msgs.msg import Twist
import rospy
from nav_msgs.msg import Odometry

# Py Lib
from itertools import product
import numpy as np
import math
import tf_conversions
import threading

class driftBSC():

    def __init__(self):
        # Create node
        rospy.init_node('drift_bsc', anonymous=True)

        # Init vars
        self.posX = 0.0;        self.posY = 0.0
        self.posZ = 0.0;        self.posW = 1.0
        self.userVelX = 0.0;    self.userVelZ = 0.0
        self.yaw = 0
        self.noiseMsg = Twist()  # Create twist/velocity object
        self.linearDrift = True;           self.whirlDrift = False   # Set what kind of drift we want
        self.tbotAng = 0.5
        self.tbotLin = rospy.get_param('/bsc/drift_value', 0.1)            # Default Tbot velocity cmds
        self.threshDist = 0.75   # Used for radius of whirlpool drift

        # Whirlpool grid
        gridX = np.linspace(-5.5, 2.5, 5)
        gridY = [-5, -2, 0, 2]
        self.coords = list(product(gridX, gridY))

        # Publishers
        self.velPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=100)

        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odomCB)
        rospy.Subscriber('/cmd_vel_mux/input/teleop', Twist, self.teleopCB)

        # Demons drifting >8)
        daemon = threading.Thread(name='driftDaemon', target=self.driftFunction)
        daemon.setDaemon(True)
        daemon.start()

        # Spin thru callbacks
        rospy.loginfo("Drifting...")
        rospy.spin()

    def odomCB(self, msg):
        # Position vars
        orientation = msg.pose.pose.orientation
        self.posX = msg.pose.pose.position.x
        self.posY = msg.pose.pose.position.y
        self.posZ = orientation.z
        self.posW = orientation.w
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        (roll, pitch, self.yaw) = tf_conversions.transformations.euler_from_quaternion(quaternion)
        # rospy.loginfo("Yaw: %f", self.yaw)

    def teleopCB(self, msg):
        # User cmd_vel
        self.userVelX = msg.linear.x
        self.userVelZ = msg.angular.z

    def driftFunction(self):
        while not rospy.is_shutdown():
            # Check drift type
            if self.linearDrift:
                # Check if we need to drift
                if -math.pi <= self.yaw <= -0.000001:
                    self.noiseMsg.angular.z = self.userVelZ - (self.tbotAng)
                elif 0.000001 <= self.yaw <= math.pi:
                    self.noiseMsg.angular.z = self.userVelZ + (self.tbotAng)

                # rospy.loginfo("User: %f, Noise: %f", self.userVelZ, self.noiseMsg.angular.z)
                # Publish drift
                self.noiseMsg.linear.x = self.userVelX
                self.velPub.publish(self.noiseMsg)

            elif self.whirlDrift:
                for x, y in self.coords:    # Loop through coordinates
                    currentDist = math.hypot(x - self.posX, y - self.posY)  # Get distance from whirlpool coords
                    # rospy.loginfo("Current Dist: %f", currentDist)
                    if currentDist <= self.threshDist:  # Check if inside whirlpool radius
                        self.noiseMsg.angular.z = self.userVelZ + (self.tbotAng)
                        self.noiseMsg.linear.x = self.userVelX + (self.tbotLin)
                        self.velPub.publish(self.noiseMsg)  # SPPPPPPPPPPIIIIIIIIIINNNNNNNIIIIINNNNNGGGGGGG

if __name__ == '__main__':
    try:
        driftBSC()
    except rospy.ROSInterruptException:
        pass
