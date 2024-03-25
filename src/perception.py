#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
import tf
import cv2 as cv

from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import pyrealsense2

class DepthListener:
    def __init__(self, topic='/realsense/aligned_depth_to_color/image_raw'):
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.image = 0.0

        rospy.Subscriber(topic, Image, callback=self.callback)

    def callback(self, data):
        if self.image is not None: return

        try:
            self.image = self.bridge.imgmsg_to_cv2(data, 'passthrough')
        except:
            pass

    def get(self):
        self.image = None
        while self.image is None:
            self.rate.sleep()
        return self.image

class RGBListener:
    def __init__(self, topic='/realsense/rgb/image_raw'):
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.image = 0.0

        rospy.Subscriber(topic, Image, callback=self.image_callback)

    def image_callback(self, data):
        if self.image is not None: return

        try:
            self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except:
            pass

    def get(self):
        self.image = None
        while self.image is None:
            self.rate.sleep()
        return self.image
