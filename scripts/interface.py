#! /usr/bin/env python

import airsim
import time
import rospy
import tf2_ros
import sys
import cv2
from numpy.lib.stride_tricks import as_strided
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseStamped, TransformStamped
import math
import numpy as np


# Source: https://github.com/eric-wieser/ros_numpy/blob/master/src/ros_numpy/image.py
name_to_dtypes = {
	"rgb8":    (np.uint8,  3),
	"rgba8":   (np.uint8,  4),
	"rgb16":   (np.uint16, 3),
	"rgba16":  (np.uint16, 4),
	"bgr8":    (np.uint8,  3),
	"bgra8":   (np.uint8,  4),
	"bgr16":   (np.uint16, 3),
	"bgra16":  (np.uint16, 4),
	"mono8":   (np.uint8,  1),
	"mono16":  (np.uint16, 1),

    # for bayer image (based on cv_bridge.cpp)
	"bayer_rggb8":	(np.uint8,  1),
	"bayer_bggr8":	(np.uint8,  1),
	"bayer_gbrg8":	(np.uint8,  1),
	"bayer_grbg8":	(np.uint8,  1),
	"bayer_rggb16":	(np.uint16, 1),
	"bayer_bggr16":	(np.uint16, 1),
	"bayer_gbrg16":	(np.uint16, 1),
	"bayer_grbg16":	(np.uint16, 1),

    # OpenCV CvMat types
	"8UC1":    (np.uint8,   1),
	"8UC2":    (np.uint8,   2),
	"8UC3":    (np.uint8,   3),
	"8UC4":    (np.uint8,   4),
	"8SC1":    (np.int8,    1),
	"8SC2":    (np.int8,    2),
	"8SC3":    (np.int8,    3),
	"8SC4":    (np.int8,    4),
	"16UC1":   (np.int16,   1),
	"16UC2":   (np.int16,   2),
	"16UC3":   (np.int16,   3),
	"16UC4":   (np.int16,   4),
	"16SC1":   (np.uint16,  1),
	"16SC2":   (np.uint16,  2),
	"16SC3":   (np.uint16,  3),
	"16SC4":   (np.uint16,  4),
	"32SC1":   (np.int32,   1),
	"32SC2":   (np.int32,   2),
	"32SC3":   (np.int32,   3),
	"32SC4":   (np.int32,   4),
	"32FC1":   (np.float32, 1),
	"32FC2":   (np.float32, 2),
	"32FC3":   (np.float32, 3),
	"32FC4":   (np.float32, 4),
	"64FC1":   (np.float64, 1),
	"64FC2":   (np.float64, 2),
	"64FC3":   (np.float64, 3),
	"64FC4":   (np.float64, 4)
}


def numpy_to_image(arr, encoding):
	if not encoding in name_to_dtypes:
		raise TypeError('Unrecognized encoding {}'.format(encoding))

	im = Image(encoding=encoding)

	# extract width, height, and channels
	dtype_class, exp_channels = name_to_dtypes[encoding]
	dtype = np.dtype(dtype_class)
	if len(arr.shape) == 2:
		im.height, im.width, channels = arr.shape + (1,)
	elif len(arr.shape) == 3:
		im.height, im.width, channels = arr.shape
	else:
		raise TypeError("Array must be two or three dimensional")

	# check type and channels
	if exp_channels != channels:
		raise TypeError("Array has {} channels, {} requires {}".format(
			channels, encoding, exp_channels
		))
	if dtype_class != arr.dtype.type:
		raise TypeError("Array is {}, {} requires {}".format(
			arr.dtype.type, encoding, dtype_class
		))

	# make the array contiguous in memory, as mostly required by the format
	contig = np.ascontiguousarray(arr)
	im.data = contig.tostring()
	im.step = contig.strides[0]
	im.is_bigendian = (
		arr.dtype.byteorder == '>' or
		arr.dtype.byteorder == '=' and sys.byteorder == 'big'
	)

	return im

def publish_tf_msg(sim_pose_msg):
    br = tf2_ros.TransformBroadcaster()

    t = TransformStamped()
    # populate tf ros message
    t.header.stamp = sim_pose_msg.header.stamp
    t.header.frame_id = "world"
    t.child_frame_id = "base_link"
    t.transform.translation.x = sim_pose_msg.pose.position.x
    t.transform.translation.y = sim_pose_msg.pose.position.y
    t.transform.translation.z = sim_pose_msg.pose.position.z
    t.transform.rotation.x = sim_pose_msg.pose.orientation.x
    t.transform.rotation.y = sim_pose_msg.pose.orientation.y
    t.transform.rotation.z = sim_pose_msg.pose.orientation.z
    t.transform.rotation.w = sim_pose_msg.pose.orientation.w

    br.sendTransform(t)

def get_camera_params():
    # read parameters
    width = rospy.get_param('~width', 640)
    height = rospy.get_param('~height', 480)
    Fx = rospy.get_param('~Fx', 360)
    Fy = rospy.get_param('~Fy', 360)
    Cx = rospy.get_param('~Cx', 360)
    Cy = rospy.get_param('~Cy', 240)

    # create sensor message
    camera_info_msg = CameraInfo()
    camera_info_msg.distortion_model = 'plumb_bob'
    camera_info_msg.width = width
    camera_info_msg.height = height
    camera_info_msg.roi.width = width
    camera_info_msg.roi.height = height
    camera_info_msg.K = [Fx, 0, Cx,
                         0, Fy, Cy,
                         0, 0, 1]
    camera_info_msg.D = [0, 0, 0, 0]

    camera_info_msg.P = [Fx, 0, Cx, 0,
                         0, Fy, Cy, 0,
                         0, 0, 1, 0]
    camera_info_msg.header.frame_id = "front_center_optical"
    return camera_info_msg

def convert_ned_to_enu(pos_ned, orientation_ned):
	pos_enu = airsim.Vector3r(pos_ned.x_val,
							- pos_ned.y_val,
							- pos_ned.z_val)
	orientation_enu = airsim.Quaternionr(orientation_ned.w_val,
									   - orientation_ned.z_val,
										 orientation_ned.x_val,
										 orientation_ned.y_val)
	return pos_enu, orientation_ned

def get_sim_pose(car_state):

    pos_ned = car_state.kinematics_estimated.position
    orientation_ned = car_state.kinematics_estimated.orientation

    pos, orientation = convert_ned_to_enu(pos_ned, orientation_ned)

    # populate PoseStamped ros message
    sim_pose_msg = PoseStamped()
    sim_pose_msg.pose.position.x = pos.x_val
    sim_pose_msg.pose.position.y = pos.y_val
    sim_pose_msg.pose.position.z = pos.z_val
    sim_pose_msg.pose.orientation.w = orientation.w_val
    sim_pose_msg.pose.orientation.x = orientation.x_val
    sim_pose_msg.pose.orientation.y = orientation.y_val
    sim_pose_msg.pose.orientation.z = orientation.z_val
    sim_pose_msg.header.seq = 1
    sim_pose_msg.header.frame_id = "world"

    return sim_pose_msg

def convert_posestamped_to_odom_msg(sim_pose_msg):
    # populate odom ros message
    odom_msg = Odometry()
    odom_msg.pose.pose = sim_pose_msg.pose
    odom_msg.header.frame_id = "world"
    odom_msg.child_frame_id = "base_link"

    return odom_msg

def get_image_messages(client):
    # get camera images from the multirotor
    responses = client.simGetImages([
       airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
       airsim.ImageRequest("0", airsim.ImageType.DepthPlanner, True)
    ])

    # convert scene uint8 array to NumPy 2D array using
    scene_img1d = np.fromstring(responses[0].image_data_uint8, dtype=np.uint8)         # get numpy array
    scene_img_rgba = scene_img1d.reshape(responses[0].height, responses[0].width, 4)   # reshape array to image array H X W
    


    # convert depth float array to NumPy 2D array using
    depth_img = airsim.list_to_2d_float_array(responses[1].image_data_float, responses[1].width, responses[1].height)

    # Populate image message
    rgb_msg = numpy_to_image(scene_img_rgba, "rgba8")
    depth_msg = numpy_to_image(depth_img, "32FC1")

    return rgb_msg, depth_msg

class Interface(object):
    def __init__(self):
        rospy.init_node('airsim_iface')


        
        # connect to the AirSim simulator 
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()
        
        #configure car controls for automatic transmission
        self.car_controls.is_manual_gear = False
        
        self.car_controls.throttle = 0.0
        self.car_controls.steering = 0.0
        self.car_controls.brake = 1.0
        
        #set up class variables to store data from subscribers
        self.current_velocity = 0.0
        self.current_angular_velocity = 0.0 
        self.velocity_cmd = 0.0
        self.angular_velocity_cmd = 0.0

        #set up timestamp for measuring actual cycle time
        self.time = rospy.get_time()
        
        
        
        #PID gain for throttle control
        self.kp_throttle = 1.0
        self.kd_throttle = 0.1
        self.ki_throttle = 0.01
        
        #PID gain for steering control
        self.kp_steering = 1.0
        self.kd_steering = 0.1
        self.ki_steering = 0.01
        
        #PID controller objects for throttle and steering
        self.pid_throttle = PID(self.kp_throttle, self.ki_throttle, self.kd_throttle, -1.0, 1.0)
        
        self.pid_steering = PID(self.kp_steering, self.ki_steering, self.kd_steering, -1.0, 1.0)
 
        
        
        #subscribe to velocity commands from teleop or path planner node
        self.twist_cmd_sub = rospy.Subscriber("/cmd_vel_mux/input/teleop", Twist, self.twist_cmd_callback)
             
        
        ## Publishers --------------------------------------------------------------
        # image publishers
        self.rgb_pub = rospy.Publisher("front_center/image_raw", Image, queue_size=1)
        self.depth_pub = rospy.Publisher("front_center/depth", Image, queue_size=1)

        # camera paramters publisher
        self.rgb_cam_pub = rospy.Publisher("front_center/camera_info", CameraInfo, queue_size=1)
        self.depth_cam_pub = rospy.Publisher("front_center/depth/camera_info", CameraInfo, queue_size=1)
        # odometry publisher
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
        #pose publisher
        self.pose_pub = rospy.Publisher("airsim/pose", PoseStamped, queue_size=1)
        
        
        self.loop()
        
    def twist_cmd_callback(self, data):
        self.velocity_cmd = data.linear.x
        self.angular_velocity_cmd = data.angular.z   
        

        


        
        
    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            #measure tume passed since last iteration
            new_time = rospy.get_time()
            dt = new_time - self.time
            self.time = new_time
            
            
            #read current state of vehicle
            car_state = self.client.getCarState()
            
            self.current_velocity = car_state.speed
            
            angular_velocity = car_state.kinematics_estimated.angular_velocity
            self.current_angular_velocity = angular_velocity.z_val
            
            #if car is commanded to stop or reverse direction, apply brake
            if ((self.current_velocity > 0 and self.velocity_cmd < 0) or (self.current_velocity < 0 and self.velocity_cmd > 0)) or abs(self.velocity_cmd) < 0.005:
                self.car_controls.throttle = 0.0
                self.car_controls.brake = -1.0
                self.pid_throttle.reset()
            else:
                self.car_controls.brake = 0.0
                if self.velocity_cmd < -0.005: #if car is commanded to drive in reverse, shift into reverse
                    self.car_controls.is_manual_gear = True
                    self.car_controls.manual_gear = -1
                else:
                    if self.car_controls.manual_gear < 1:
                        self.car_controls.manual_gear = 1
                        self.client.setCarControls(self.car_controls)
                    self.car_controls.is_manual_gear = False

                #set throttle based on x velocity command                 
                throttle_error = abs(self.velocity_cmd) - abs(self.current_velocity)
                throttle = self.pid_throttle.step(throttle_error, dt)
                
                #apply throttle if current speed less than target, apply brake if current speed greater than target
                if throttle > 0:
                    self.car_controls.throttle = throttle
                    self.car_controls.brake = 0.0
                else:
                    self.car_controls.throttle = 0.0
                    self.car_controls.brake = throttle
                    
                #set steering based on angular velocity command
                #positive values for steering command correspond to right turn (negative angular velocity)
                
                if abs(self.current_velocity) >= 0.005:
                    steering_error = self.angular_velocity_cmd - self.current_angular_velocity
                    self.car_controls.steering = -1.0 * self.pid_steering.step(steering_error, dt)
                else:
                    self.car_controls.steering = 0.0
                    self.pid_steering.reset()
                
            #send commands to simulator
            self.client.setCarControls(self.car_controls)
            
            #get camera data
            camera_info_msg = get_camera_params()
            sim_pose_msg = get_sim_pose(car_state)
            odom_msg = convert_posestamped_to_odom_msg(sim_pose_msg)
            rgb_msg, depth_msg = get_image_messages(self.client)
            
    
            # header message
            sim_pose_msg.header.stamp = rospy.Time.now()
            odom_msg.header.stamp = sim_pose_msg.header.stamp
            camera_info_msg.header.stamp = sim_pose_msg.header.stamp
            rgb_msg.header = camera_info_msg.header
            depth_msg.header = camera_info_msg.header
            

    
            # publish messages
            self.pose_pub.publish(sim_pose_msg)
            publish_tf_msg(sim_pose_msg)
            self.odom_pub.publish(odom_msg)
            self.rgb_cam_pub.publish(camera_info_msg)
            self.depth_cam_pub.publish(camera_info_msg)
            self.rgb_pub.publish(rgb_msg)
            self.depth_pub.publish(depth_msg)

            


            rate.sleep()
            
                
                
if __name__ == '__main__':
    Interface()
