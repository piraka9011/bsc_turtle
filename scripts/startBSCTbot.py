#!/usr/bin/env python

# ROS Lib
import rospy

# PyLib
from multiprocessing import Process
import os
import time

if __name__ == "__main__":

    # Check if ROScore open
    if not rospy.core.is_initialized():
        print("Starting ROScore")
        roscore = Process(target=os.system, args=["roscore"])
        roscore.start()
        time.sleep(3)

    # Bringup Turtlebot
    print("Turtlebot Bringup")
    tbotBringup = Process(target=os.system, args=["roslaunch turtlebot_bringup minimal.launch"])
    tbotBringup.start()
    time.sleep(2)

    # Start AMCL
    print("Opening AMCL")
    amclLaunch = Process(target=os.system, args=["roslaunch turtlebot_navigation amcl_demo.launch"])
    amclLaunch.start()
    time.sleep(5)


'''
    # ------------------------------- Start BSC -------------------------------
    # Send start location
    print("Sending start location")
    sendStartLaunch = Process(target=os.system, args=["rosrun bsc_turtle sendStart.py"])
    sendStartLaunch.start()
    time.sleep(1)


    For Simulation Only
    # Start Gazebo
    print("Starting Gazebo")
    gazeboLaunch = Process(target=os.system, args=["roslaunch turtlebot_gazebo turtlebot_world.launch"])
    gazeboLaunch.start()
    time.sleep(2)

    # Start AMCL node
    print("Starting AMCL Simulator")
    amclLaunch = Process(target=os.system, args=["roslaunch turtlebot_gazebo amcl_demo.launch"])
    amclLaunch.start()
    time.sleep(2)

    # Open RVIZ
    print("Opening RVIZ")
    rvizLaunch = Process(target=os.system, args=["roslaunch turtlebot_rviz_launchers view_navigation.launch"])
    rvizLaunch.start()
    time.sleep(3)

'''