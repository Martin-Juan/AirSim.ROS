#!/usr/bin/env python
import setup_path
import airsim

import roslib
roslib.load_manifest('learning_tf')
import rospy

import tf

   def handle_turtle_pose(msg):
      br = tf.TransformBroadcaster()
     br.sendTransform((msg.x, msg.y, msg.z),
                        tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                        rospy.Time.now(),
                        multirotor,
                        "world")
 if __name__ == '__main__':
      rospy.init_node('turtle_tf_broadcaster')

      rospy.Subscriber('/airsimPose' % turtlename,
                        airsim.msg.Pose,
                        multirotor,
                        turtlename)
       rospy.spin()
