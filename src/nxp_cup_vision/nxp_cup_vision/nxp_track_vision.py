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

        # Camera image size parameters
        self.imageHeight = 240
        self.imageWidth = 320

        # Subscribers
        self.imageSub = self.create_subscription(sensor_msgs.msg.CompressedImage,
                                                 'camera/image_raw/compressed',
                                                 self.camera_image_callback,
                                                 qos_profile_sensor_data)

        # Publishers
        self.debugDetectionImagePub = self.create_publisher(sensor_msgs.msg.CompressedImage,
                                                            "/debugImage", 0)

    def camera_image_callback(self, data):
        scene = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')

        binary_image = cv2.inRange(scene, (100, 100, 100), (255, 255, 255))

        height, width = binary_image.shape

        # y_debut = height * 2 // 5
        y_start = 0
        y_end = height

        binary_image = binary_image[y_start:y_end, :]

        for y in range(height):
            count = 0

            for x in range(width):
                if binary_image[y, x] == 0:
                    count += 1
                else:
                    if count > 25:
                        for i in range(count + 1):
                            binary_image[y, x - i] = 255
                    count = 0

        # Scene from subscription callback
        msg = self.bridge.cv2_to_compressed_imgmsg(binary_image)
        self.debugDetectionImagePub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NXPTrackVision()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
