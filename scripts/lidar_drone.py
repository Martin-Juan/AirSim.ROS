#!/usr/bin/env python

# AirSim Python API
import setup_path
import airsim
import numpy
import rospy

# ROS Image message
from sensor_msgs.msg import LaserScan

def lidar():
    pub = rospy.Publisher("airsim/lidar", LaserScan, queue_size=1)
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
        msg=lidar()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "laser_frame"
    	msg.angle_min = -1
    	msg.angle_max = 1
    	msg.scan_increment = 3.14 /num_readings
    	msg.range_min = 0.0
    	msg.range_max = 100.0
    	msg.set_ranges_size(num_readings_)
        msg.set_intensities_size(num_readings)
    	for(unsigned int i = 0; i < num_readings; ++i){
    		msg.ranges[i] = ranges[i]
    		msg.intensities[i] = intensities[i]     }

        # publish image message
        pub.publish(msg)
        # sleep until next cycle
        rate.sleep()


if __name__ == '__main__':
    try:
        lidar()
    except rospy.ROSInterruptException:
        pass
