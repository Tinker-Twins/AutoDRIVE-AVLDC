#!/usr/bin/env python

################################################################################

# Copyright (c) 2024, Tinker Twins
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

#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import numpy as np

################################################################################

class AEB_Emulation:
    def __init__(self):
        self.ego_pos = None
        self.obs_pos = None
        self.dtc     = None
        self.pub_steering_command = None
        self.pub_throttle_command = None
        self.pub_brake_command = None
        self.pub_handbrake_command = None

    def aeb_emulation_callback(self, msg):
        self.ego_pos = np.array([msg.x, msg.y, msg.z])
        self.obs_pos = np.array([-242.16, -119.00, 341.91])
        self.dtc = np.linalg.norm(self.ego_pos - self.obs_pos)
        print("DTC: ", self.dtc)

    def aeb_emulation(self):
        if self.dtc < 20:
            throttle_command = 0 # [-1, 1]
            steering_command = 0 # [-1, 1]
            brake_command = 1 # [0, 1]
            handbrake_command = 0 # [0, 1]
        else:
            throttle_command = 0.20 # [-1, 1]
            steering_command = 0 # [-1, 1]
            brake_command = 0 # [0, 1]
            handbrake_command = 0 # [0, 1]

        self.pub_throttle_command.publish(throttle_command)
        self.pub_steering_command.publish(steering_command)
        self.pub_brake_command.publish(brake_command)
        self.pub_handbrake_command.publish(handbrake_command)

################################################################################

if __name__ == '__main__':
    rospy.init_node('aeb_emulation', anonymous=True)

    aeb_emulation_node = AEB_Emulation()

    rospy.Subscriber("/autodrive/opencav_1/gnss", Point, aeb_emulation_node.aeb_emulation_callback)

    aeb_emulation_node.pub_steering_command = rospy.Publisher('/autodrive/opencav_1/steering_command', Float32, queue_size=1)
    aeb_emulation_node.pub_throttle_command = rospy.Publisher('/autodrive/opencav_1/throttle_command', Float32, queue_size=1)
    aeb_emulation_node.pub_brake_command = rospy.Publisher('/autodrive/opencav_1/brake_command', Float32, queue_size=1)
    aeb_emulation_node.pub_handbrake_command = rospy.Publisher('/autodrive/opencav_1/handbrake_command', Float32, queue_size=1)

    rospy.wait_for_message("/autodrive/opencav_1/gnss", Point)

    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            aeb_emulation_node.aeb_emulation()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass