#!/usr/bin/env python3
"""Converts Gazebo float32 depth (meters) to uint16 depth (millimeters)
for TrackDLO perception compatibility.
Also corrects and republishes CameraInfo with proper intrinsics
(workaround for Gazebo Fortress ros_gz_bridge camera_info bug)."""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np


class DepthFormatConverter(Node):
    def __init__(self):
        super().__init__('depth_format_converter')
        self.bridge = CvBridge()

        self.declare_parameter('input_topic', '/gz/camera/depth_raw')
        self.declare_parameter('output_topic',
                               '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_in',
                               '/camera/color/camera_info')
        self.declare_parameter('camera_info_out',
                               '/camera/aligned_depth_to_color/camera_info')
        self.declare_parameter('hfov', 1.2112)  # D435 color HFOV in radians

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        camera_info_in = self.get_parameter('camera_info_in').value
        camera_info_out = self.get_parameter('camera_info_out').value
        self.hfov = self.get_parameter('hfov').value

        # Depth converter
        self.depth_sub = self.create_subscription(
            Image, input_topic, self.depth_callback, 10)
        self.depth_pub = self.create_publisher(Image, output_topic, 10)

        # CameraInfo corrector
        self.info_sub = self.create_subscription(
            CameraInfo, camera_info_in, self.info_callback, 10)
        self.info_pub_color = self.create_publisher(
            CameraInfo, '/camera/color/camera_info_corrected', 10)
        self.info_pub_depth = self.create_publisher(
            CameraInfo, camera_info_out, 10)

        self.info_corrected = False

        self.get_logger().info(
            f'Depth converter: {input_topic} (32FC1) -> {output_topic} (16UC1)')
        self.get_logger().info(
            f'CameraInfo corrector: {camera_info_in} -> corrected')

    def depth_callback(self, msg):
        depth_float = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        depth_mm = np.copy(depth_float)
        depth_mm[np.isnan(depth_mm) | np.isinf(depth_mm)] = 0.0
        depth_uint16 = (depth_mm * 1000.0).astype(np.uint16)

        out_msg = self.bridge.cv2_to_imgmsg(depth_uint16, encoding='16UC1')
        out_msg.header = msg.header
        self.depth_pub.publish(out_msg)

    def info_callback(self, msg):
        # Compute correct intrinsics from HFOV and image dimensions
        w = msg.width
        h = msg.height
        fx = w / (2.0 * math.tan(self.hfov / 2.0))
        fy = fx  # square pixels
        cx = w / 2.0
        cy = h / 2.0

        # Check if correction is needed
        if abs(msg.k[0] - fx) < 1.0:
            # Already correct, just republish
            self.info_pub_depth.publish(msg)
            return

        if not self.info_corrected:
            self.get_logger().info(
                f'Correcting CameraInfo: fx {msg.k[0]:.1f}->{fx:.1f}, '
                f'cx {msg.k[2]:.1f}->{cx:.1f}, cy {msg.k[5]:.1f}->{cy:.1f}')
            self.info_corrected = True

        # Correct K matrix (3x3 intrinsic)
        corrected = CameraInfo()
        corrected.header = msg.header
        corrected.height = h
        corrected.width = w
        corrected.distortion_model = msg.distortion_model
        corrected.d = msg.d
        corrected.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ]
        corrected.r = list(msg.r)
        corrected.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        corrected.binning_x = msg.binning_x
        corrected.binning_y = msg.binning_y
        corrected.roi = msg.roi

        self.info_pub_depth.publish(corrected)
        # Also publish corrected color camera_info
        corrected_color = CameraInfo()
        corrected_color.header = corrected.header
        corrected_color.height = corrected.height
        corrected_color.width = corrected.width
        corrected_color.distortion_model = corrected.distortion_model
        corrected_color.d = corrected.d
        corrected_color.k = list(corrected.k)
        corrected_color.r = list(corrected.r)
        corrected_color.p = list(corrected.p)
        corrected_color.binning_x = corrected.binning_x
        corrected_color.binning_y = corrected.binning_y
        corrected_color.roi = corrected.roi
        self.info_pub_color.publish(corrected_color)


def main(args=None):
    rclpy.init(args=args)
    node = DepthFormatConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
