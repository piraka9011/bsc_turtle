#!/usr/bin/env python

# ROS Lib
import actionlib
from move_base_msgs.msg import *
import rospy

def sendStart():
    # Create Action Client to send goal to "move_base"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # Create goal
    goal = MoveBaseGoal()
    '''
    # RVIZ: X: 0.5, Y: -0.06 W: 1
    # Office: X: -0.3 Y: 0.15 W:1/Z:0
    goal.target_pose.pose.position.x = 0
    goal.target_pose.pose.position.y = 0
    goal.target_pose.pose.orientation.z = 0.7
    goal.target_pose.pose.orientation.w = 0.7
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = 'map'

    # Wait for server to appear
    client.wait_for_server(rospy.Duration(5))

    # Send the goal
    client.send_goal(goal)
    rospy.loginfo("Published starting position")

    # Wait for the result

    rospy.loginfo(client.get_result())
    # Wait till goal reached/BSC open
    raw_input("Press enter once the BSC module is running and the turtlebot is in position...")
    '''
    # Goal position
    # RVIZ: X: 0.6 Y: -4.45 W: 1
    # Office: X: 1.5 Y: -3 W:1/Z:0
    goal.target_pose.pose.position.x = 3.45628553533
    goal.target_pose.pose.position.y = -1.45433644569
    goal.target_pose.pose.orientation.z = -0.674237942631
    goal.target_pose.pose.orientation.w = 0.738514181798
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = 'map'

    # Send goal position
    client.wait_for_server()
    client.send_goal(goal)
    client.wait_for_result()

# ----------------------------Main Function------------------------------------

if __name__ == '__main__':

    # Init Node
    rospy.init_node('bscStart')

    try:
        # Main call
        sendStart()
    except rospy.ROSInterruptException():
        pass
