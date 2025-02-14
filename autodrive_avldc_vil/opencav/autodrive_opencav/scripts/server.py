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
from gevent import pywsgi
from geventwebsocket.handler import WebSocketHandler
import socketio
import numpy as np
import base64
from io import BytesIO
from PIL import Image
from tf.transformations import quaternion_from_euler
import autodrive_opencav.config as config
from autodrive_opencav.bridge import Bridge

# AutoDRIVE-ROS Bridge instance
autodrive_ros_bridge = Bridge(config.pub_sub_dict)

# Initialize the server
sio = socketio.Server(async_mode='gevent')

# Registering "connect" event handler for the server
@sio.on('connect')
def connect(sid, environ):
    print("Connected!")

# Registering "Bridge" event handler for the server
@sio.on('Bridge')
def bridge(sid, data):
    if data:
        ########################################################################
        # VEHICLE DATA
        ########################################################################
        # Actuator feedbacks
        throttle = float(data["V1 Throttle"])
        steering = float(data["V1 Steering"])
        autodrive_ros_bridge.publish_actuator_feedbacks(throttle, steering)
        # Wheel encoders
        encoder_angles = np.fromstring(data["V1 Encoder Angles"], dtype=float, sep=' ')
        autodrive_ros_bridge.publish_encoder_data(encoder_angles)
        # GNSS
        position = np.fromstring(data["V1 Position"], dtype=float, sep=' ')
        autodrive_ros_bridge.publish_gnss_data(position)
        # IMU
        orientation_quaternion = np.fromstring(data["V1 Orientation Quaternion"], dtype=float, sep=' ')
        angular_velocity = np.fromstring(data["V1 Angular Velocity"], dtype=float, sep=' ')
        linear_acceleration = np.fromstring(data["V1 Linear Acceleration"], dtype=float, sep=' ')
        autodrive_ros_bridge.publish_imu_data(orientation_quaternion, angular_velocity, linear_acceleration)
        # Cooordinate transforms
        autodrive_ros_bridge.broadcast_transform("opencav_1", "map", position, orientation_quaternion) # Vehicle frame defined at center of rear axle
        autodrive_ros_bridge.broadcast_transform("left_encoder", "opencav_1", np.asarray([0.0, 0.867, 0.0]), quaternion_from_euler(0.0, 120*encoder_angles[0]%6.283, 0.0))
        autodrive_ros_bridge.broadcast_transform("right_encoder", "opencav_1", np.asarray([0.0, -0.867, 0.0]), quaternion_from_euler(0.0, 120*encoder_angles[1]%6.283, 0.0))
        autodrive_ros_bridge.broadcast_transform("gnss", "opencav_1", np.asarray([0.0, 0.0, 0.0]), np.asarray([0.0, 0.0, 0.0, 1.0]))
        autodrive_ros_bridge.broadcast_transform("imu", "opencav_1", np.asarray([0.0, 0.0, 0.0]), np.asarray([0.0, 0.0, 0.0, 1.0]))
        autodrive_ros_bridge.broadcast_transform("lidar", "opencav_1", np.asarray([0.725, -0.015, 1.84]), np.asarray([0.0, 0.0, 0.0, 1.0]))
        autodrive_ros_bridge.broadcast_transform("left_camera", "opencav_1", np.asarray([2.085, 0.255, 1.285]), np.asarray([0.0, 0.0, 0.0, 1.0]))
        autodrive_ros_bridge.broadcast_transform("right_camera", "opencav_1", np.asarray([2.085, -0.285, 1.285]), np.asarray([0.0, 0.0, 0.0, 1.0]))
        autodrive_ros_bridge.broadcast_transform("front_left_wheel", "opencav_1", np.asarray([3.09, 0.867, 0.0]), quaternion_from_euler(0.0, 0.0, np.arctan((2*0.141537*np.tan(steering))/(2*0.141537-2*0.0765*np.tan(steering)))))
        autodrive_ros_bridge.broadcast_transform("front_right_wheel", "opencav_1", np.asarray([3.09, -0.867, 0.0]), quaternion_from_euler(0.0, 0.0, np.arctan((2*0.141537*np.tan(steering))/(2*0.141537+2*0.0765*np.tan(steering)))))
        autodrive_ros_bridge.broadcast_transform("rear_left_wheel", "opencav_1", np.asarray([0.0, 0.867, 0.0]), quaternion_from_euler(0.0, encoder_angles[0]%6.283, 0.0))
        autodrive_ros_bridge.broadcast_transform("rear_right_wheel", "opencav_1", np.asarray([0.0, -0.867, 0.0]), quaternion_from_euler(0.0, encoder_angles[1]%6.283, 0.0))
        # LIDAR
        # lidar_pointcloud = np.frombuffer(base64.b64decode(data["V1 LIDAR Pointcloud"]), dtype=np.uint8).tolist()
        # autodrive_ros_bridge.publish_lidar_pointcloud(lidar_pointcloud)
        # Cameras
        left_camera_image = np.asarray(Image.open(BytesIO(base64.b64decode(data["V1 Left Camera Image"]))))
        right_camera_image = np.asarray(Image.open(BytesIO(base64.b64decode(data["V1 Right Camera Image"]))))
        autodrive_ros_bridge.publish_camera_images(left_camera_image, right_camera_image)

        ########################################################################
        # CONTROL COMMANDS
        ########################################################################
        # Co-simulation, vehicle and environment control commands
        sio.emit('Bridge', data={
            'V1 CoSim': str(config.cosim_mode),
            'V1 PosX': str(config.posX_command), 'V1 PosY': str(config.posY_command), 'V1 PosZ': str(config.posZ_command),
            'V1 RotX': str(config.rotX_command), 'V1 RotY': str(config.rotY_command), 'V1 RotZ': str(config.rotZ_command), 'V1 RotW': str(config.rotW_command),
            'V1 Throttle': str(config.throttle_command), 'V1 Steering': str(config.steering_command),
            'V1 Brake': str(config.brake_command), 'V1 Handbrake': str(config.handbrake_command),
            'V1 Headlights': str(config.headlights_command), 'V1 Indicators': str(config.indicators_command),
            'Auto Time': str(config.auto_time), 'Time Scale': str(config.time_scale), 'Time': str(config.time_of_day), 'Weather': str(config.weather_id),
            'Clouds': str(config.cloud_intensity), 'Fog': str(config.fog_intensity), 'Rain': str(config.rain_intensity), 'Snow': str(config.snow_intensity)
            })

if __name__ == '__main__':
    app = socketio.WSGIApp(sio) # Create socketio WSGI application
    pywsgi.WSGIServer(('', 4567), app, handler_class=WebSocketHandler).serve_forever() # Deploy as an gevent WSGI server