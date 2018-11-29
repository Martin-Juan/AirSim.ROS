#!/usr/bin/env python


import setup_path
import airsim
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped

def callback(msg):
    rospy.loginfo("Received at goal message!")


    position = msg.pose.position
    rospy.loginfo("Point Position: [ %f, %f, %f ]"%(position.x, position.y, position.z))
    client = airsim.MultirotorClient()
    state = client.getMultirotorState()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    client.moveToPositionAsync(position.x, position.y, position.z, 10).join()
    client.hoverAsync().join()

def moveGoal():
    rospy.init_node('goal_listener', anonymous=True)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback)
    rospy.spin()
if __name__ == '__main__':
    try:
        moveGoal()
    except rospy.ROSInterruptException:
        pass
