#!/usr/bin/env python3
"""HSV Tuner GUI Node: Interactive HSV threshold tuning with live camera feed.

Subscribes to RGB camera topic, displays OpenCV window with sliders,
publishes binary segmentation mask to /trackdlo/segmentation_mask.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class HSVTunerNode(Node):
    def __init__(self):
        super().__init__('hsv_tuner')
        self.bridge = CvBridge()

        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('hsv_threshold_upper_limit', '135 255 255')
        self.declare_parameter('hsv_threshold_lower_limit', '85 50 20')

        rgb_topic = self.get_parameter('rgb_topic').value
        upper_str = self.get_parameter('hsv_threshold_upper_limit').value
        lower_str = self.get_parameter('hsv_threshold_lower_limit').value
        upper_vals = [int(x) for x in upper_str.split()]
        lower_vals = [int(x) for x in lower_str.split()]

        self.h_min, self.s_min, self.v_min = lower_vals
        self.h_max, self.s_max, self.v_max = upper_vals
        self.latest_image = None

        self.rgb_sub = self.create_subscription(
            Image, rgb_topic, self.image_callback, 10)
        self.mask_pub = self.create_publisher(
            Image, '/trackdlo/segmentation_mask', 10)

        cv2.namedWindow('HSV Tuner', cv2.WINDOW_AUTOSIZE)
        cv2.createTrackbar('H Min', 'HSV Tuner', self.h_min, 179, lambda x: None)
        cv2.createTrackbar('S Min', 'HSV Tuner', self.s_min, 255, lambda x: None)
        cv2.createTrackbar('V Min', 'HSV Tuner', self.v_min, 255, lambda x: None)
        cv2.createTrackbar('H Max', 'HSV Tuner', self.h_max, 179, lambda x: None)
        cv2.createTrackbar('S Max', 'HSV Tuner', self.s_max, 255, lambda x: None)
        cv2.createTrackbar('V Max', 'HSV Tuner', self.v_max, 255, lambda x: None)

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info('HSV Tuner started. Adjust sliders, press Q to save & quit.')

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def timer_callback(self):
        if self.latest_image is None:
            cv2.waitKey(1)
            return

        h_min = cv2.getTrackbarPos('H Min', 'HSV Tuner')
        s_min = cv2.getTrackbarPos('S Min', 'HSV Tuner')
        v_min = cv2.getTrackbarPos('V Min', 'HSV Tuner')
        h_max = cv2.getTrackbarPos('H Max', 'HSV Tuner')
        s_max = cv2.getTrackbarPos('S Max', 'HSV Tuner')
        v_max = cv2.getTrackbarPos('V Max', 'HSV Tuner')

        hsv = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2HSV)
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)

        masked = cv2.bitwise_and(self.latest_image, self.latest_image, mask=mask)

        # Show original and masked side by side
        display = np.hstack([
            cv2.resize(self.latest_image, (640, 480)),
            cv2.resize(masked, (640, 480)),
        ])
        cv2.imshow('HSV Tuner', display)

        if (h_min != self.h_min or s_min != self.s_min or v_min != self.v_min or
                h_max != self.h_max or s_max != self.s_max or v_max != self.v_max):
            self.get_logger().info(
                f'HSV: lower="{h_min} {s_min} {v_min}" upper="{h_max} {s_max} {v_max}"')
            self.h_min, self.s_min, self.v_min = h_min, s_min, v_min
            self.h_max, self.s_max, self.v_max = h_max, s_max, v_max

        mask_msg = self.bridge.cv2_to_imgmsg(mask, 'mono8')
        mask_msg.header.stamp = self.get_clock().now().to_msg()
        self.mask_pub.publish(mask_msg)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info(
                f'Final HSV values for realsense_params.yaml:')
            self.get_logger().info(
                f'  hsv_threshold_lower_limit: "{h_min} {s_min} {v_min}"')
            self.get_logger().info(
                f'  hsv_threshold_upper_limit: "{h_max} {s_max} {v_max}"')
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HSVTunerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
