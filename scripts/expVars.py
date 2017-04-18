#!/usr/bin/env python

# ROS Lib
import csv
import math
import os
import signal
import time
from multiprocessing import Process

import rospy
from move_base_msgs.msg import *
from nav_msgs.msg import Odometry

# Global variables: These variables are passed to other scripts as configs
userDelay = False       # bsc_main.py
linearDrift = False     # driftBSC.py
whirlDrift = False      # driftBSC.py
alphaDelay = 1          # bsc_main.py
alphaDriftLin = 0.5     # driftBSC.py
alphaDriftAng = 1       # driftBSC.py

# ----------------------------------Data Collection----------------------------------

class ExpVariables:

    def __init__(self):

        # -----------------------------------HouseKeeping---------------------------------
        # Init Node
        rospy.init_node('bscData', anonymous=True)

        # Init Variables
        self.timeStart = rospy.Time();      self.timeEnd = rospy.Time()
        self.totalTime = rospy.Time();      self.distToGoal = 100.0
        self.goalX = 2.5;                   self.goalY = -2.5
        self.pathLength = 0.0;              self.navResult = 0
        self.lastPoseX = 0.0;               self.lastPoseY = 0.0

        # ROS Params
        rospy.set_param('/move_base/recovery_behavior_enabled', False)
        rospy.set_param('/move_base/clearing_rotation_allowed', False)
        rospy.set_param('/move_base/oscillation_timeout', 0.0)
        rospy.set_param('/move_base/DWAPlannerROS/xy_goal_tolerance', 0.5)
        rospy.set_param('/move_base/DWAPlannerROS/yaw_goal_tolerance', 0.9)
        rospy.set_param('/move_base/planner_patience', 15.0)

        # Capture Ctrl-C
        signal.signal(signal.SIGINT, self.sigintHandler)

        # ROS Subscribers
        rospy.Subscriber('/odom', Odometry, self.getDistance)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.resultCallback)
        # --------------------------------------------------------------------------------
        # ------------------------------------User Info-----------------------------------
        # Get User Name
        self.subjName = raw_input("Enter Subject Name: ")
        csvName = self.subjName + '.csv'

        # CSV File
        self.csvFile = open(csvName, 'w')
        self.csvWriter = csv.writer(self.csvFile)
        self.csvWriter.writerow(('Name:', 'Time:', 'Distance:'))
        # --------------------------------------------------------------------------------
        # ------------------------------------Processes-----------------------------------
        # Send the goal position
        startSend = raw_input("Send goal position? (y/n)")
        if startSend == "y":
            sendStart = Process(target=os.system, args=["rosrun bsc_turtle sendStart.py"])
            sendStart.start()
            time.sleep(1)
        '''
        # Run BSC Node
        print("Running BSC")
        runBSC = Process(target=os.system, args=["rosrun bsc_turtle bsc_main.py"])
        runBSC.start()
        time.sleep(1)
        '''
        # Begin
        raw_input("Press enter to start experiment...")

        # ROSBag record for position traveled
        print("Recording to bag...")
        runROSBag = Process(target=os.system,
                            args=["rosbag record -O %s.bag "
                                  "/tf /move_base/DWAPlannerROS/global_plan" % self.subjName])
        runROSBag.start()
        time.sleep(1)
        # --------------------------------------------------------------------------------
        # Get start time
        self.timeStart = rospy.get_rostime()

        # SPIIIIIIIIIIIINININNIIIINNNIINNNNING
        rospy.loginfo("Spinning")
        rospy.spin()

    def getDistance(self, msg):
        posX = msg.pose.pose.position.x
        posY = msg.pose.pose.position.y
        xDist = self.goalX - posX
        yDist = self.goalY - posY

        # Calculate length of trip (euclidean distance from last->current position
        self.pathLength += math.hypot((posX - self.lastPoseX), (posY - self.lastPoseY))
        self.lastPoseX = posX;  self.lastPoseY = posY

        # Calculate distance to goal
        self.distToGoal = math.hypot(xDist, yDist)

        # Check if reached tolerance
        if self.navResult == 3:
            rospy.loginfo("Got End time")
            self.timeEnd = rospy.get_rostime()
            self.totalTime = self.timeEnd - self.timeStart
            # Write to file
            self.csvWriter.writerow((self.subjName,
                                     "%i.%i " % (self.totalTime.secs, self.totalTime.nsecs),
                                     ("%f " % self.pathLength)))
            time.sleep(1)
            # Write to terminal
            rospy.loginfo("Total Time: %i.%i", self.totalTime.secs, self.totalTime.nsecs)
            rospy.loginfo("Distance traveled: %f", self.pathLength)
            self.csvFile.close()
            # Kill all threads, sys.exit() not enough
            os._exit(0)

    # Handler for ctrl-c, does same as reaching goal
    def sigintHandler(self, sig, frame):
        rospy.loginfo("Got End time")
        self.timeEnd = rospy.get_rostime()
        self.totalTime = self.timeEnd - self.timeStart
        rospy.loginfo("Total Time: %i.%i", self.totalTime.secs, self.totalTime.nsecs)
        rospy.loginfo("Distance traveled: %f", self.pathLength)
        # Write to file
        self.csvWriter.writerow((self.subjName,
                                 "%i.%i " % (self.totalTime.secs, self.totalTime.nsecs),
                                 ("%f " % self.pathLength)))
        self.csvFile.close()
        os._exit(0)

    # Gets navigation result to determine if goal reached
    def resultCallback(self, msg):
        self.navResult = msg.status.status

# ----------------------------Main Function------------------------------------
if __name__ == "__main__":
    ExpVariables()
    '''
    # Run Drifting node
    if linearDrift or whirlDrift:
        # call(['terminator', '-e', 'rosrun bsc_turtle driftBSC.py'])
        # Popen('rosrun bsc_turtle driftBSC.py', shell=True, stdout=PIPE, stderr=STDOUT)
        runDrift = Process(target=os.system, args=["rosrun bsc_turtle driftBSC.py"])
        runDrift.start()
        time.sleep(1)

    # Keep default configs?
    acceptDefault = str2bool(raw_input("Accept Default configs? (y/n)"))

    # Mess around with configs
    if not acceptDefault:
        # User Input time delay
        userDelay = str2bool(raw_input("User Delay? (y/n)"))
        if userDelay:
            alphaDelay = raw_input("Alpha Delay (default 1): ")

        # Drift velocity commands
        # Check if linear drift wanted
        linearDrift = str2bool(raw_input("Linear Drift? (y/n)"))
        if linearDrift:
            # Set drifting strength
            alphaDriftLin = raw_input("Alpha Drift (default 1): ")
        # Or whirlpools
        elif not linearDrift:
            whirlDrift = str2bool(raw_input("Whirlpool Drift? (y/n)"))
            if whirlDrift:
                # Set drifting strength
                alphaDriftAng = raw_input("Alpha Drift (default 1): ")
    '''

