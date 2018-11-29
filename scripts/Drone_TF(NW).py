#!/usr/bin/env python

import setup_path
import airsim

import rospy
import tf_conversions
import tf2_ros
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg
import time


def airtf():
    pub = rospy.Publisher("aisrimTF",PoseStamped, queue_size=1 )
    rospy.init_node('airtf', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    while not rospy.is_shutdown():
        multirotor_state = client.getMultirotorState()
        pos = multirotor_state.kinematics_estimated.position
        orientation = multirotor_state.kinematics_estimated.orientation
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = client
        t.transform.translation.x = pos.x_val
        t.transform.translation.y = pos.y_val
        t.transform.translation.z = pos.z_val
        br.sendTransform(t)
if __name__ == '__main__':
    try:
        airtf()
    except rospy.ROSInterruptException:
        pass
