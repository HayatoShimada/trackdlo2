#!/usr/bin/env python3
"""Converts Gazebo float32 depth (meters) to uint16 depth (millimeters)
for TrackDLO perception compatibility."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class DepthFormatConverter(Node):
    def __init__(self):
        super().__init__('depth_format_converter')
        self.bridge = CvBridge()

        self.declare_parameter('input_topic', '/gz/camera/depth_raw')
        self.declare_parameter('output_topic',
                               '/camera/aligned_depth_to_color/image_raw')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.sub = self.create_subscription(
            Image, input_topic, self.callback, 10)
        self.pub = self.create_publisher(Image, output_topic, 10)

        self.get_logger().info(
            f'Depth converter: {input_topic} (32FC1) -> {output_topic} (16UC1)')

    def callback(self, msg):
        depth_float = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        depth_mm = np.copy(depth_float)
        depth_mm[np.isnan(depth_mm) | np.isinf(depth_mm)] = 0.0
        depth_uint16 = (depth_mm * 1000.0).astype(np.uint16)

        out_msg = self.bridge.cv2_to_imgmsg(depth_uint16, encoding='16UC1')
        out_msg.header = msg.header
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthFormatConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
