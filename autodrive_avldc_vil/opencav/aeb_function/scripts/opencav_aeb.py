#!/usr/bin/env python

################################################################################

# Copyright (c) 2025, Tinker Twins
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

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import rospkg

################################################################################

class OpenCAV_AEB:
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.detection = None
        self.pub_steering_command = None
        self.pub_throttle_command = None
        self.pub_brake_command = None
        self.pub_handbrake_command = None

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('aeb_function')

        # Load YOLO Model
        self.net = cv2.dnn.readNet(pkg_path+"/models/yolov3-tiny.weights", pkg_path+"/config/yolov3-tiny.cfg")

        # Load Classes
        with open(pkg_path+"/config/coco.names", 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]

        # Configuration
        self.layer_name = self.net.getLayerNames()
        self.output_layer = [self.layer_name[i - 1] for i in self.net.getUnconnectedOutLayers()]
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))

    def opencav_aeb(self, msg):
        
        ########################################################################
        # PERCEPTION
        ########################################################################
        
        # Load image
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as error:
            print(error)
        
        # Resize image
        img = cv2.resize(cv_image, (640, 360))
        height, width, channel = img.shape

        # Detect Objects
        blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layer)

        # Display object detection information
        label = None
        confidence = None
        size = None
        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w/2)
                    y = int(center_y - h/2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        font = cv2.FONT_HERSHEY_PLAIN
        for i in range(len(boxes)):
            if i in indices:
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                confidence = np.round(confidences[i]*100, 2)
                size = w*h
                # print('Class: {} \t Confidence: {} % \t Size: {} px²'.format(label, confidence, size))
                color = self.colors[i]
                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                cv2.putText(img, label, (x, y + 30), font, 3, color, 3)
        cv2.imshow("Object Detection", img)
        cv2.waitKey(1)

        ########################################################################
        # PLANNING
        ########################################################################

        # Compute AEB trigger
        AEB = 1 if (label=="car" and confidence>=50 and size>=1000) else 0

        # Verbose
        print("AEB: {}".format(AEB))

        ########################################################################
        # CONTROL
        ########################################################################

        # Set commands
        throttle_command = 0.0 if (AEB==1) else 0.5
        steering_command = 0.0
        brake_command = 1.0 if (AEB==1) else 0.0
        handbrake_command = 0.0

        # Publish commands
        self.pub_throttle_command.publish(throttle_command)
        self.pub_steering_command.publish(steering_command)
        self.pub_brake_command.publish(brake_command)
        self.pub_handbrake_command.publish(handbrake_command)

################################################################################

if __name__ == '__main__':
    rospy.init_node('opencav_aeb', anonymous=True)

    opencav_aeb_node = OpenCAV_AEB()

    rospy.Subscriber("/autodrive/opencav_1/right_camera", Image, opencav_aeb_node.opencav_aeb)
    opencav_aeb_node.pub_steering_command = rospy.Publisher('/autodrive/opencav_1/steering_command', Float32, queue_size=1)
    opencav_aeb_node.pub_throttle_command = rospy.Publisher('/autodrive/opencav_1/throttle_command', Float32, queue_size=1)
    opencav_aeb_node.pub_brake_command = rospy.Publisher('/autodrive/opencav_1/brake_command', Float32, queue_size=1)
    opencav_aeb_node.pub_handbrake_command = rospy.Publisher('/autodrive/opencav_1/handbrake_command', Float32, queue_size=1)
    rospy.wait_for_message("/autodrive/opencav_1/right_camera", Image)

    try:
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass