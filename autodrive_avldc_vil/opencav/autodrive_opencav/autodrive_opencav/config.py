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
from attrdict import AttrDict

# ROS publishers and subscribers
pub_sub_dict = AttrDict({
    'subscribers': [
        # Co-simulation subscribers
        {'topic':'/autodrive/opencav_1/cosim_mode', 'type': 'int', 'name': 'sub_cosim_mode'},
        {'topic':'/autodrive/opencav_1/pose_command', 'type': 'pose', 'name': 'sub_pose_command'},
        # Vehicle data subscribers
        {'topic':'/autodrive/opencav_1/throttle_command', 'type': 'float', 'name': 'sub_throttle_command'},
        {'topic':'/autodrive/opencav_1/steering_command', 'type': 'float', 'name': 'sub_steering_command'},
        {'topic':'/autodrive/opencav_1/brake_command', 'type': 'float', 'name': 'sub_brake_command'},
        {'topic':'/autodrive/opencav_1/handbrake_command', 'type': 'float', 'name': 'sub_handbrake_command'},
        {'topic':'/autodrive/opencav_1/headlights_command', 'type': 'float', 'name': 'sub_headlights_command'},
        {'topic':'/autodrive/opencav_1/indicators_command', 'type': 'float', 'name': 'sub_indicators_command'},
        # Environment data subscribers
        {'topic':'/autodrive/environment/auto_time', 'type': 'string', 'name': 'sub_auto_time'},
        {'topic':'/autodrive/environment/time_scale', 'type': 'int', 'name': 'sub_time_scale'},
        {'topic':'/autodrive/environment/time_of_day', 'type': 'int', 'name': 'sub_time_of_day'},
        {'topic':'/autodrive/environment/weather_id', 'type': 'int', 'name': 'sub_weather_id'},
        {'topic':'/autodrive/environment/cloud_intensity', 'type': 'float', 'name': 'sub_cloud_intensity'},
        {'topic':'/autodrive/environment/fog_intensity', 'type': 'float', 'name': 'sub_fog_intensity'},
        {'topic':'/autodrive/environment/rain_intensity', 'type': 'float', 'name': 'sub_rain_intensity'},
        {'topic':'/autodrive/environment/snow_intensity', 'type': 'float', 'name': 'sub_snow_intensity'}
    ],
    'publishers': [
        # Vehicle data publishers
        {'topic': '/autodrive/opencav_1/throttle', 'type': 'float', 'name': 'pub_throttle'},
        {'topic': '/autodrive/opencav_1/steering', 'type': 'float', 'name': 'pub_steering'},
        {'topic': '/autodrive/opencav_1/left_encoder', 'type': 'joint_state', 'name': 'pub_left_encoder'},
        {'topic': '/autodrive/opencav_1/right_encoder', 'type': 'joint_state', 'name': 'pub_right_encoder'},
        {'topic': '/autodrive/opencav_1/gnss', 'type': 'point', 'name': 'pub_gnss'},
        {'topic': '/autodrive/opencav_1/imu', 'type': 'imu', 'name': 'pub_imu'},
        {'topic': '/autodrive/opencav_1/lidar', 'type': 'pointcloud', 'name': 'pub_lidar'},
        {'topic': '/autodrive/opencav_1/left_camera', 'type': 'image', 'name': 'pub_left_camera'},
        {'topic': '/autodrive/opencav_1/right_camera', 'type': 'image', 'name': 'pub_right_camera'}
    ]
})

# Co-simulation commands
cosim_mode = 0 # [0 = false, 1 = true]
posX_command = 0.0 # [-∞, ∞]
posY_command = 0.0 # [-∞, ∞]
posZ_command = 0.0 # [-∞, ∞]
rotX_command = 0.0 # [-1, 1]
rotY_command = 0.0 # [-1, 1]
rotZ_command = 0.0 # [-1, 1]
rotW_command = 1.0 # [-1, 1]

# Vehicle control commands
throttle_command = 0 # [-1, 1]
steering_command = 0 # [-1, 1]
brake_command = 0 # [0, 1]
handbrake_command = 0 # [0, 1]
headlights_command = 0 # Vehicle headlights command [0 = Disabled, 1 = Low Beam, 2 = High Beam, 3 = Parking Lights, 4 = Fog Lights, 5 = 1+3, 6 = 1+4, 7 = 2+3, 8 = 2+4, 9 = 3+4, 10 = 1+3+4, 11 = 2+3+4]
indicators_command = 0 # Vehicle indicators command [0 = Disabled, 1 = Left Turn Indicator, 2 = Right Turn Indicator, 3 = Hazard Indicators]

# Environment conditions
auto_time = "False" # ["False", "True"]
time_scale = 60 # [0, inf) (only used if auto_time==True)
time_of_day = 560 # [minutes in 24 hour format] (only used if auto_time==False)
weather_id = 3 # [0=Custom, 1=Sunny, 2=Cloudy, 3=LightFog, 4=HeavyFog, 5=LightRain, 6=HeavyRain, 7=LightSnow, 8=HeavySnow]
cloud_intensity = 0.0 # [0, 1] (only used if weather_id==0)
fog_intensity = 0.0 # [0, 1] (only used if weather_id==0)
rain_intensity = 0.0 # [0, 1] (only used if weather_id==0)
snow_intensity = 0.0 # [0, 1] (only used if weather_id==0)