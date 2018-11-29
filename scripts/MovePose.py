#!/usr/bin/env python


import setup_path
import airsim
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64


def callback(msg):
    x=msg.pose.position.x
    y=msg.pose.position.y
    z=msg.pose.position.z
    rospy.loginfo(x,y,z)
def GotoGoal():
    client = airsim.MultirotorClient()
    state = client.getMultirotorState()
    pub = rospy.Subscriber("/move_base_simple/goal", Float64, callback)
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    client.moveToPositionAsync(x, y, z, 10).join()
    client.hoverAsync().join()
    rospy.spin()
if __name__ == '__main__':
    try:
        GotoGoal()
    except rospy.ROSInterruptException:
        pass
