#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters


class MaskNode(Node):
    def __init__(self):
        super().__init__('mask_node')
        self.bridge = CvBridge()

        self.mask_img_pub = self.create_publisher(Image, '/trackdlo/results_img', 10)

        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.pc_sub = message_filters.Subscriber(self, PointCloud2, '/camera/depth/color/points')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.pc_sub], 10, 0.1)
        self.ts.registerCallback(self.callback)

    def callback(self, rgb, pc):
        cur_image = self.bridge.imgmsg_to_cv2(rgb, 'rgb8')
        hsv_image = cv2.cvtColor(cur_image.copy(), cv2.COLOR_RGB2HSV)

        # latex blue
        lower = (100, 230, 60)
        upper = (130, 255, 255)
        mask = cv2.inRange(hsv_image, lower, upper)

        mask = cv2.cvtColor(mask.copy(), cv2.COLOR_GRAY2BGR)

        mask_img_msg = self.bridge.cv2_to_imgmsg(mask, 'rgb8')
        self.mask_img_pub.publish(mask_img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MaskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
