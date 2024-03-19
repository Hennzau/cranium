#!/usr/bin/env python3
import os
import sys
import copy
import re
import importlib
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rclpy.clock import ROSClock
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor
import sensor_msgs.msg
from synapse_msgs.msg import Status
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile
import cv2

if cv2.__version__ < "4.0.0":
    raise ImportError("Requires opencv >= 4.0, "
                      "but found {:s}".format(cv2.__version__))


class NXPTrackVision(Node):

    def __init__(self):
        super().__init__("nxp_track_vision")

        # setup CvBridge
        self.bridge = CvBridge()

        # Rectangualr area to remove from image calculation to
        # eliminate the vehicle. Used as ratio of overall image width and height
        # "width ratio,height ratio"
        self.maskRectRatioWidthHeight = np.array([0.0, 0.0])

        # Bool for generating and publishing the debug image evaluation
        self.debug = True
        self.debugLineMethodUsed = False

        self.timeStamp = self.get_clock().now().nanoseconds
        # Camera image size parameters
        self.imageHeight = 240
        self.imageWidth = 320

        self.pts1 = np.float32([[0, self.imageHeight/3*2], [self.imageWidth/3, self.imageHeight/2], [self.imageWidth, self.imageHeight/3*2], [self.imageWidth*2/3, self.imageHeight/2]])
        self.pts2 = np.float32([[0, self.imageHeight], [self.imageWidth/3, 0], [self.imageWidth, self.imageHeight], [self.imageWidth*2/3, 0]])
        self.matrix = cv2.getPerspectiveTransform(self.pts1, self.pts2)

        # Subscribers
        self.imageSub = self.create_subscription(sensor_msgs.msg.CompressedImage,
                                                 'camera/image_raw/compressed',
                                                 self.pixyImageCallback,
                                                 qos_profile_sensor_data)

        # Publishers
        self.debugDetectionImagePub = self.create_publisher(sensor_msgs.msg.CompressedImage,
                                                            "/debugImage", 0)

    def pixyImageCallback(self, data):
        # Scene from subscription callback
        scene = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')
        scene = cv2.warpPerspective(scene, self.matrix, (self.imageWidth, self.imageHeight), cv2.BORDER_CONSTANT, 0)

        self.debugDetectionImagePub.publish(self.bridge.cv2_to_compressed_imgmsg(scene))


def main(args=None):
    rclpy.init(args=args)
    node = NXPTrackVision()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
