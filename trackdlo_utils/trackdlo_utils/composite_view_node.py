#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


class CompositeViewNode(Node):
    def __init__(self):
        super().__init__('composite_view')
        self.bridge = CvBridge()

        self.panels = {
            'Camera': None,
            'Segmentation Mask': None,
            'Segmentation Overlay': None,
            'TrackDLO Results': None,
        }

        self.create_subscription(
            Image, '/camera/color/image_raw', self._cb_camera, 10)
        self.create_subscription(
            Image, '/trackdlo/segmentation_mask_img', self._cb_mask, 10)
        self.create_subscription(
            Image, '/trackdlo/segmentation_overlay', self._cb_overlay, 10)
        self.create_subscription(
            Image, '/trackdlo/results_img', self._cb_results, 10)

        self.timer = self.create_timer(1.0 / 30.0, self._timer_cb)
        self.get_logger().info('Composite view node started')

    def _cb_camera(self, msg):
        self.panels['Camera'] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _cb_mask(self, msg):
        mono = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        self.panels['Segmentation Mask'] = cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

    def _cb_overlay(self, msg):
        self.panels['Segmentation Overlay'] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _cb_results(self, msg):
        self.panels['TrackDLO Results'] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _timer_cb(self):
        # Determine panel size from first available image
        ref = next((img for img in self.panels.values() if img is not None), None)
        if ref is None:
            return

        h, w = ref.shape[:2]
        ph, pw = h // 2, w // 2

        grid = []
        for label, img in self.panels.items():
            if img is not None:
                panel = cv2.resize(img, (pw, ph))
            else:
                panel = np.zeros((ph, pw, 3), dtype=np.uint8)
            cv2.putText(panel, label, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            grid.append(panel)

        top = np.hstack((grid[0], grid[1]))
        bottom = np.hstack((grid[2], grid[3]))
        composite = np.vstack((top, bottom))

        cv2.imshow('TrackDLO Composite View', composite)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CompositeViewNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
