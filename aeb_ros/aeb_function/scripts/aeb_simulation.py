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
from darknet_ros_msgs.msg import BoundingBoxes

################################################################################

class AEB_Simulation:
    def __init__(self):
        self.detection     = None
        self.pub_steering_command = None
        self.pub_throttle_command = None
        self.pub_brake_command = None
        self.pub_handbrake_command = None

    def aeb_simulation_callback(self, msg):
        self.detection = msg.bounding_boxes[0] # Capture most recent YOLO detection

    def aeb_simulation(self):
        AREA_THRESH = 3000 # Threshold bounding box area (px squared)
        PROB_THRESH = 0.7 # Threshold classification probability [0, 1]

        detection = self.detection.Class # Class (label) of YOLO object detection
        probability = self.detection.probability # Probability (confidence) of classification
        box_length = self.detection.xmax - self.detection.xmin # Length of bounding box
        box_height = self.detection.ymax - self.detection.ymin # Height of bounding box
        box_area = box_length*box_height # Area of bounding box

        if detection=='car' and probability>PROB_THRESH and box_area>AREA_THRESH:
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
    rospy.init_node('aeb_simulation', anonymous=True)

    aeb_simulation_node = AEB_Simulation()

    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, aeb_simulation_node.aeb_simulation_callback)

    aeb_simulation_node.pub_steering_command = rospy.Publisher('/autodrive/opencav_1/steering_command', Float32, queue_size=1)
    aeb_simulation_node.pub_throttle_command = rospy.Publisher('/autodrive/opencav_1/throttle_command', Float32, queue_size=1)
    aeb_simulation_node.pub_brake_command = rospy.Publisher('/autodrive/opencav_1/brake_command', Float32, queue_size=1)
    aeb_simulation_node.pub_handbrake_command = rospy.Publisher('/autodrive/opencav_1/handbrake_command', Float32, queue_size=1)

    rospy.wait_for_message("/darknet_ros/bounding_boxes", BoundingBoxes)

    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            aeb_simulation_node.aeb_simulation()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass