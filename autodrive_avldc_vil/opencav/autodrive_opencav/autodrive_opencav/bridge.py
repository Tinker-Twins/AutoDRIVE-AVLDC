#!/usr/bin/env python

################################################################################

# Copyright (c) 2023, Tinker Twins
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

################################################################################

# Import libraries
import rospy
import tf
from std_msgs.msg import Int32, Float32, Header, String
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import JointState, Imu, PointCloud2, PointField, Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import autodrive_opencav.config as config

msg_types = {
    'int': Int32,
    'float': Float32,
    'string': String,
    'joint_state': JointState,
    'point': Point,
    'pose': Pose,
    'imu': Imu,
    'pointcloud': PointCloud2,
    'image':Image
}

class Bridge(object):
    def __init__(self, pub_sub_dict):
        rospy.init_node('autodrive_ros_bridge') # Initialize node
        self.cv_bridge = CvBridge()
        self.callbacks = {
            # Co-simulation subscriber callbacks
            '/autodrive/opencav_1/cosim_mode': self.callback_cosim_mode,
            '/autodrive/opencav_1/pose_command': self.callback_pose_command,
            # Vehicle data subscriber callbacks
            '/autodrive/opencav_1/throttle_command': self.callback_throttle_command,
            '/autodrive/opencav_1/steering_command': self.callback_steering_command,
            '/autodrive/opencav_1/brake_command': self.callback_brake_command,
            '/autodrive/opencav_1/handbrake_command': self.callback_handbrake_command,
            '/autodrive/opencav_1/headlights_command': self.callback_headlights_command,
            '/autodrive/opencav_1/indicators_command': self.callback_indicators_command,
            # Environment data subscriber callbacks
            '/autodrive/environment/auto_time': self.callback_auto_time,
            '/autodrive/environment/time_scale': self.callback_time_scale,
            '/autodrive/environment/time_of_day': self.callback_time_of_day,
            '/autodrive/environment/weather_id': self.callback_weather_id,
            '/autodrive/environment/cloud_intensity': self.callback_cloud_intensity,
            '/autodrive/environment/fog_intensity': self.callback_fog_intensity,
            '/autodrive/environment/rain_intensity': self.callback_rain_intensity,
            '/autodrive/environment/snow_intensity': self.callback_snow_intensity,
        }
        # Subscribers
        self.subscribers = [rospy.Subscriber(e.topic, msg_types[e.type], self.callbacks[e.topic])
                            for e in pub_sub_dict.subscribers]
        # Publishers
        self.publishers = {e.name: rospy.Publisher(e.topic, msg_types[e.type], queue_size=1)
                           for e in pub_sub_dict.publishers}

    #########################################################
    # ROS MESSAGE GENERATING FUNCTIONS
    #########################################################

    def create_int_msg(self, val):
        i = Int32()
        i.data = val
        return i

    def create_float_msg(self, val):
        f = Float32()
        f.data = val
        return f

    def create_joint_state_msg(self, joint_angle, joint_name, frame_id):
        js = JointState()
        js.header = Header()
        js.header.stamp = rospy.Time.now()
        js.header.frame_id = frame_id
        js.name = [joint_name]
        js.position = [joint_angle]
        js.velocity = []
        js.effort = []
        return js

    def create_point_msg(self, position):
        p = Point()
        p.x = position[0]
        p.y = position[1]
        p.z = position[2]
        return p

    def create_imu_msg(self, orientation_quaternion, angular_velocity, linear_acceleration):
        imu = Imu()
        imu.header = Header()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = 'imu'
        imu.orientation.x = orientation_quaternion[0]
        imu.orientation.y = orientation_quaternion[1]
        imu.orientation.z = orientation_quaternion[2]
        imu.orientation.w = orientation_quaternion[3]
        imu.orientation_covariance = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
        imu.angular_velocity.x = angular_velocity[0]
        imu.angular_velocity.y = angular_velocity[1]
        imu.angular_velocity.z = angular_velocity[2]
        imu.angular_velocity_covariance = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
        imu.linear_acceleration.x = linear_acceleration[0]
        imu.linear_acceleration.y = linear_acceleration[1]
        imu.linear_acceleration.z = linear_acceleration[2]
        imu.linear_acceleration_covariance = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
        return imu

    def create_pointcloud_msg(self, lidar_pointcloud):
        pc = PointCloud2()
        pc.header = Header()
        pc.header.stamp = rospy.Time.now()
        pc.header.frame_id = 'lidar'
        pc.height = 1 # [uint32] 2D structure of the point cloud (if the cloud is unordered height is 1)
        pc.point_step = 24 # [uint32] Length of a point in bytes
        pc.width = int(len(lidar_pointcloud)/pc.point_step) # [uint32] Length of the point cloud data
        pc.row_step = pc.width*pc.point_step # [uint32] Length of a row in bytes
        pc.data = lidar_pointcloud # [uint8[]] Actual pointcloud data with size (row_step*height)
        pc.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                     PointField('y', 4, PointField.FLOAT32, 1),
                     PointField('z', 8, PointField.FLOAT32, 1),
                     PointField('intensity', 16, PointField.FLOAT32, 1),
                     PointField('ring', 20, PointField.UINT16, 1)
                     ] # [PointField[]] Describes the channels and their layout in the binary data blob
        pc.is_bigendian = False # [bool] True if the data is big-endian (BE)
        pc.is_dense = False # [bool] True if there are no invalid points
        return pc

    def create_image_msg(self, image_array, frame_id):
        img = self.cv_bridge.cv2_to_imgmsg(image_array, encoding="rgb8")
        img.header = Header()
        img.header.stamp = rospy.Time.now()
        img.header.frame_id = frame_id
        return img

    def broadcast_transform(self, child_frame_id, parent_frame_id, position_tf, orientation_tf):
        tb = tf.TransformBroadcaster()
        tb.sendTransform(position_tf, orientation_tf, rospy.Time.now(), child_frame_id, parent_frame_id)

    #########################################################
    # ROS PUBLISHER FUNCTIONS
    #########################################################

    # VEHICLE DATA PUBLISHER FUNCTIONS

    def publish_actuator_feedbacks(self, throttle, steering):
        self.publishers['pub_throttle'].publish(self.create_float_msg(throttle))
        self.publishers['pub_steering'].publish(self.create_float_msg(steering))

    def publish_encoder_data(self, encoder_angles):
        self.publishers['pub_left_encoder'].publish(self.create_joint_state_msg(encoder_angles[0], "left_encoder", "left_encoder"))
        self.publishers['pub_right_encoder'].publish(self.create_joint_state_msg(encoder_angles[1], "right_encoder", "right_encoder"))

    def publish_gnss_data(self, position):
        self.publishers['pub_gnss'].publish(self.create_point_msg(position))

    def publish_imu_data(self, orientation_quaternion, angular_velocity, linear_acceleration):
        self.publishers['pub_imu'].publish(self.create_imu_msg(orientation_quaternion, angular_velocity, linear_acceleration))

    def publish_lidar_pointcloud(self, lidar_pointcloud):
        self.publishers['pub_lidar'].publish(self.create_pointcloud_msg(lidar_pointcloud))

    def publish_camera_images(self, left_camera_image, right_camera_image):
        self.publishers['pub_left_camera'].publish(self.create_image_msg(left_camera_image, "left_camera"))
        self.publishers['pub_right_camera'].publish(self.create_image_msg(right_camera_image, "right_camera"))

    #########################################################
    # ROS SUBSCRIBER CALLBACKS
    #########################################################
        
    # CO-SIMULATION SUBSCRIBER CALLBACKS
    
    def callback_cosim_mode(self, cosim_mode):
        config.cosim_mode = cosim_mode.data

    def callback_pose_command(self, pose_command):
        config.posX_command = pose_command.position.x
        config.posY_command = pose_command.position.y
        config.posZ_command = pose_command.position.z
        config.rotX_command = pose_command.orientation.x
        config.rotY_command = pose_command.orientation.y
        config.rotZ_command = pose_command.orientation.z
        config.rotW_command = pose_command.orientation.w
    
    # VEHICLE DATA SUBSCRIBER CALLBACKS

    def callback_throttle_command(self, throttle_command):
        config.throttle_command = np.round(throttle_command.data, 3)

    def callback_steering_command(self, steering_command):
        config.steering_command = np.round(steering_command.data, 3)
    
    def callback_brake_command(self, brake_command):
        config.brake_command = brake_command.data

    def callback_handbrake_command(self, handbrake_command):
        config.handbrake_command = handbrake_command.data
    
    def callback_headlights_command(self, headlights_command):
        config.headlights_command = headlights_command.data

    def callback_indicators_command(self, indicators_command):
        config.indicators_command = indicators_command.data

    # ENVIRONMRNT DATA SUBSCRIBER CALLBACKS

    def callback_auto_time(self, auto_time):
        config.auto_time = auto_time
    
    def callback_time_scale(self, time_scale):
        config.time_scale = time_scale

    def callback_time_of_day(self, time_of_day):
        config.time_of_day = time_of_day.data

    def callback_weather_id(self, weather_id):
        config.weather_id = weather_id.data

    def callback_cloud_intensity(self, cloud_intensity):
        config.cloud_intensity = cloud_intensity.data
    
    def callback_fog_intensity(self, fog_intensity):
        config.fog_intensity = fog_intensity.data

    def callback_rain_intensity(self, rain_intensity):
        config.rain_intensity = rain_intensity.data

    def callback_snow_intensity(self, snow_intensity):
        config.snow_intensity = snow_intensity.data