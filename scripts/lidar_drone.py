#!/usr/bin/env python

# AirSim Python API
import setup_path 
import airsim
import numpy
import rospy

# ROS Image message
from sensor_msgs.msg import Lidar

def lidar():
    pub = rospy.Publisher("airsim/lidar", Image, queue_size=1)
    rospy.init_node('lidar', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # connect to the AirSim simulator 
    client = airsim.MultirotorClient()
    client.confirmConnection()

    while not rospy.is_shutdown():
         # get camera images from the car
        responses = client.getLidarData()

        for response in responses:
            img_rgba_string = response.image_data_uint8

        # Populate image message
        msg=Image() 
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "frameId"
        msg.encoding = "rgba8"
        msg.height = 360  # resolution should match values in settings.json 
        msg.width = 640
        msg.data = img_rgba_string
        msg.is_bigendian = 0
        msg.step = msg.width * 4

        # log time and size of published image
        rospy.loginfo(len(response.image_data_uint8))
        # publish image message
        pub.publish(msg)
        # sleep until next cycle
        rate.sleep()


if __name__ == '__main__':
    try:
        airpub()
    except rospy.ROSInterruptException:
        pass
