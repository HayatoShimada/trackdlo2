#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import numpy as np


class SimulateOcclusionEval(Node):
    def __init__(self):
        super().__init__('simulate_occlusion_eval')
        self.bridge = CvBridge()

        self.arr_sub = self.create_subscription(
            Int32MultiArray, '/corners', self.callback, 10)
        self.occlusion_mask_img_pub = self.create_publisher(
            Image, '/mask_with_occlusion', 10)

    def callback(self, arr_msg):
        arr = arr_msg.data

        mouse_mask = np.ones((720, 1280, 3))
        mouse_mask[arr[1]:arr[3], arr[0]:arr[2], :] = 0

        occlusion_mask = (mouse_mask * 255).astype('uint8')

        occlusion_mask_img_msg = self.bridge.cv2_to_imgmsg(occlusion_mask, 'rgb8')
        self.occlusion_mask_img_pub.publish(occlusion_mask_img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulateOcclusionEval()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
