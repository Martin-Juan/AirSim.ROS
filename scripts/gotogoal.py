#!/usr/bin/env python

import setup_path
import airsim
import rospy
import geometry_msgs.msg
import numpy as np
    def move2goal(self):
        """Moves the turtle to the goal."""
        rospy.wait_for_message('move_base_simple/goal', PoseStamped);
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.update_goal)
          # Get the input from the user.
        goal_pose.x = input("Set your x goal: ")
        goal_pose.y = input("Set your y goal: ")
        goal_pose.z = input("Set your z goal: ")
        pub = rospy.Publisher("aisrimgoto",PoseStamped, queue_size=1 )
        rospy.init_node('airsimgo', anonymous=True)
        rate = rospy.Rate(10) # 10hz

        # connect to the AirSim simulator
        client = airsim.MultirotorClient()
        state = client.getMultirotorState()
        client.confirmConnection()
        client.enableApiControl(True)
        client.armDisarm(True)
        client.moveToPositionAsync(goal_pose.x, goal_pose.y, goal_pose.z, 10).join()
        client.hoverAsync().join()
if __name__ == '__main__':
    try:
        x=dronemovement()
        x.move2goal
    except rospy.ROSInterruptException:
        pass
