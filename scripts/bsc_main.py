#!/usr/bin/env python

# Python Lib
from __future__ import division
from functools import partial
import math
import threading

# ROS Lib
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

'''
    This class implements the following:
        Thread 1: BSC Thread
        - Blending of inputs, outputting to proper cmd_vel_mux topic
        Thread 2: Spin Thread
        - Getting turtlebot position
        - Getting user cmd_vel
        - Getting the navigation stack's cmd_vel
        - Delay user input if userDelay set to True
'''

# Input string to boolean
def str2bool(s):
    return s in "y"

class BSCFun:

    def __init__(self):

        # Init Node
        rospy.init_node('bscMain', anonymous=True)

        # Init variables
        self.optHeading = 0.0;  self.newHeading = 0.0   # Headings
        self.goalX = 0.0;       self.goalY = 0.0        # Goal position
        self.currentX = 0.0;    self.currentY = 0.0     # Turtlebot position
        self.xVelUser = 0.0;    self.wzVelUser = 1.0    # User cmd_vel
        self.xVel = 0.0;        self.wzVel = 0.0        # Navi cmd_vel
        self.distToGoal = 0.0   # Dist. to goal as hypotenuse of x/y positions
        alpha = 1       # Alpha variable, don't mess with it...
        self.userDelay = False;  self.delay = 1 * alpha  # Delay variables

        self.userDelay = str2bool(raw_input("User Delay? (y/n)"))
        if self.userDelay:
            alpha = raw_input("Alpha Delay (Default: 1):")

        # ROS Publishers
        self.pub = rospy.Publisher('/cmd_vel_mux/input/bsc', Twist, queue_size=100)

        # DEEEEMONS :O
        daemon = threading.Thread(name='bscDaemon', target=self.bscDaemon)
        daemon.setDaemon(True)
        daemon.start()

        # ROS Subscribers
        rospy.Subscriber('/odom', Odometry, self.odomCallback)
        rospy.Subscriber('/move_base/current_goal', PoseStamped, self.goalCallback)
        rospy.Subscriber('/cmd_vel_mux/input/navi', Twist, self.optCmdCallback)
        rospy.Subscriber('/cmd_vel_mux/input/teleop', Twist, self.userCmdCallback)
        rospy.loginfo("BSC Ready")
        rospy.spin()

    def bscDaemon(self):
        # BSC Daemon conducts necessary blending while ROS spins
        # Create Twist object
        vMsg = Twist()
        while not rospy.is_shutdown():
            # Get distance from goal
            xDist = self.goalX - self.currentX
            yDist = self.goalY - self.currentY
            self.distToGoal = math.hypot(xDist, yDist)
            # NC rospy.loginfo("Dist to Goal: %f", self.distToGoal)

            # Calculate difference in commands
            deltaOpZ = self.wzVelUser - self.wzVel

            # BSC
            dMax1 = 8    # Max distance blending occurs at
            optMax1 = 3  # Max difference in cmds allowed
            # Math stuff, see paper on BSC parameter
            bscParamZ = max(0, (1 - (self.distToGoal / dMax1))) * \
                        max(0, (1 - math.pow((deltaOpZ / optMax1), 2)))
            # NC rospy.loginfo("BscParam: %f", bscParamZ)
            vMsg.angular.z = self.wzVelUser - (bscParamZ * deltaOpZ)
            vMsg.linear.x = self.xVelUser
            # NC rospy.loginfo("Z: %f, X: %f", vMsg.angular.z, vMsg.linear.x)
            # Publish blended velocity cmd
            self.pub.publish(vMsg)

    def odomCallback(self, msg):
        # Get current position
        position = msg.pose.pose.position
        self.currentX = position.x
        self.currentY = position.y

    def goalCallback(self, msg):
        # Get goal position
        position = msg.pose.position
        self.goalX = position.x
        self.goalY = position.y

    def delayedUserCmdCB(self, msg, event):
        # Delayed user vel cmd callback. See userCmdCallback for timer CB
        self.xVelUser = msg.linear.x
        self.wzVelUser = msg.angular.z

    def userCmdCallback(self, msg):
        # Check if we want to have delay
        if self.userDelay:
            # Use ROS timer to call the delayed callback
            # Delayed call back changes the user cmds only once called
            rospy.Timer(rospy.Duration(self.delay),
                        partial(self.delayedUserCmdCB, msg))
        elif not self.userDelay:
            # Get x and y velocity and determine the heading for the user
            self.xVelUser = msg.linear.x
            self.wzVelUser = msg.angular.z

    def optCmdCallback(self, msg):
        # Get x and y velocity and determine the heading for the plan
        self.xVel = msg.linear.x
        self.wzVel = msg.angular.z

BSCFun()
