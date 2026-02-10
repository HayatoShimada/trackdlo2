#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class OcclusionSimulation(Node):
    def __init__(self):
        super().__init__('simulated_occlusion')
        self.bridge = CvBridge()

        self.rect = [0, 0, 0, 0]
        self.startPoint = False
        self.endPoint = False
        self.start_moving = False
        self.rect_center = None
        self.offsets = None
        self.resting = False

        self.mouse_mask = None

        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.callback, 10)
        self.occlusion_mask_img_pub = self.create_publisher(
            Image, '/mask_with_occlusion', 100)

    def callback(self, rgb):
        cur_image = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')

        height, width, layers = cur_image.shape
        new_h = int(height / 1.5)
        new_w = int(width / 1.5)
        frame = cv2.resize(cur_image, (new_w, new_h))

        if self.mouse_mask is None:
            self.mouse_mask = np.ones(frame.shape)

        frame = (frame * np.clip(self.mouse_mask, 0.5, 1)).astype('uint8')

        cv2.namedWindow('frame')
        cv2.setMouseCallback('frame', self.on_mouse)

        key = cv2.waitKey(10)

        if key == 114:  # r
            frame = cv2.resize(cur_image, (new_w, new_h))
            self.startPoint = False
            self.endPoint = False
            self.start_moving = False
            self.mouse_mask = np.ones(frame.shape)
            cv2.imshow('frame', frame)
        elif self.start_moving and not self.resting:
            self.mouse_mask = np.ones(frame.shape)
            self.mouse_mask[self.rect[1]:self.rect[3], self.rect[0]:self.rect[2], :] = 0
            cv2.imshow('frame', frame)
        else:
            if self.startPoint and not self.endPoint:
                cv2.rectangle(frame, (self.rect[0], self.rect[1]),
                              (self.rect[2], self.rect[3]), (0, 0, 255), 2)
                if self.rect[0] < self.rect[2] and self.rect[1] < self.rect[3]:
                    frame = cv2.putText(frame, 'occlusion', (self.rect[0], self.rect[1]-10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 240), 2)
                elif self.rect[0] < self.rect[2] and self.rect[1] > self.rect[3]:
                    frame = cv2.putText(frame, 'occlusion', (self.rect[0], self.rect[3]-10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 240), 2)
                elif self.rect[0] > self.rect[2] and self.rect[1] < self.rect[3]:
                    frame = cv2.putText(frame, 'occlusion', (self.rect[2], self.rect[1]-10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 240), 2)
                else:
                    frame = cv2.putText(frame, 'occlusion', (self.rect[2], self.rect[3]-10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 240), 2)

            if self.startPoint and self.endPoint:
                if self.rect[1] > self.rect[3]:
                    self.rect[1], self.rect[3] = self.rect[3], self.rect[1]
                if self.rect[0] > self.rect[2]:
                    self.rect[0], self.rect[2] = self.rect[2], self.rect[0]

                self.mouse_mask[self.rect[1]:self.rect[3], self.rect[0]:self.rect[2], :] = 0

                if not self.mouse_mask.all() == 1:
                    frame = cv2.putText(frame, 'occlusion', (self.rect[0], self.rect[1]-10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 240), 2)

            cv2.imshow('frame', frame)

        # publish mask
        occlusion_mask = (self.mouse_mask * 255).astype('uint8')
        occlusion_mask = cv2.resize(occlusion_mask, (width, height))

        occlusion_mask_img_msg = self.bridge.cv2_to_imgmsg(occlusion_mask, 'rgb8')
        self.occlusion_mask_img_pub.publish(occlusion_mask_img_msg)

    def on_mouse(self, event, x, y, flags, params):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.startPoint and self.endPoint:
                self.startPoint = False
                self.endPoint = False
                self.rect = [0, 0, 0, 0]

            if not self.startPoint:
                self.rect = [x, y, x, y]
                self.startPoint = True
            elif not self.endPoint:
                self.rect = [self.rect[0], self.rect[1], x, y]
                self.endPoint = True

        elif event == cv2.EVENT_MOUSEMOVE and self.startPoint and not self.endPoint:
            self.rect = [self.rect[0], self.rect[1], x, y]

        elif (event == cv2.EVENT_MBUTTONDOWN and not self.start_moving
              and np.sum(self.mouse_mask[y, x]) == 0):
            self.start_moving = True
            self.rect_center = (x, y)
            self.offsets = (self.rect[0]-self.rect_center[0], self.rect[1]-self.rect_center[1],
                            self.rect[2]-self.rect_center[0], self.rect[3]-self.rect_center[1])

        elif event == cv2.EVENT_MOUSEMOVE and self.start_moving:
            self.rect = [x+self.offsets[0], y+self.offsets[1],
                         x+self.offsets[2], y+self.offsets[3]]
            self.resting = False

        elif event == cv2.EVENT_MBUTTONDOWN and self.start_moving:
            self.start_moving = False

        elif event != cv2.EVENT_MOUSEMOVE and self.start_moving:
            self.resting = True


def main(args=None):
    rclpy.init(args=args)
    node = OcclusionSimulation()
    try:
        rclpy.spin(node)
    except Exception:
        print("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
