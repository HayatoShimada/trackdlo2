#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2

import cv2
import numpy as np
from scipy import ndimage


class TrackingResultImg(Node):
    def __init__(self):
        super().__init__('tracking_result_img')
        self.bridge = CvBridge()

        self.cur_image = []
        self.cur_image_arr = []
        self.bmask = []
        self.mask = []

        self.proj_matrix = np.array([
            [918.359130859375, 0.0, 645.8908081054688, 0.0],
            [0.0, 916.265869140625, 354.02392578125, 0.0],
            [0.0, 0.0, 1.0, 0.0]])

        self.create_subscription(Image, '/mask', self.update_mask, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.update_rgb, 10)
        self.create_subscription(PointCloud2, '/cdcpd2_no_gripper_results_pc', self.callback, 10)
        self.tracking_img_pub = self.create_publisher(Image, '/tracking_img', 10)

    def update_rgb(self, data):
        temp = self.bridge.imgmsg_to_cv2(data, 'rgb8')
        if len(self.cur_image_arr) <= 3:
            self.cur_image_arr.append(temp)
            self.cur_image = self.cur_image_arr[0]
        else:
            self.cur_image_arr.append(temp)
            self.cur_image = self.cur_image_arr[0]
            self.cur_image_arr.pop(0)

    def update_mask(self, data):
        self.mask = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        self.bmask = cv2.cvtColor(self.mask, cv2.COLOR_BGR2GRAY)

    def callback(self, pc_msg):
        if len(self.cur_image) == 0 or len(self.bmask) == 0:
            return

        # process point cloud
        points = list(pc2.read_points(pc_msg, field_names=('x', 'y', 'z'), skip_nans=True))
        nodes = np.array(points)

        if len(nodes) == 0:
            return

        # projection
        mask_dis_threshold = 10
        nodes_h = np.hstack((nodes, np.ones((len(nodes), 1))))
        image_coords = np.matmul(self.proj_matrix, nodes_h.T).T
        us = (image_coords[:, 0] / image_coords[:, 2]).astype(int)
        vs = (image_coords[:, 1] / image_coords[:, 2]).astype(int)

        us = np.where(us >= 1280, 1279, us)
        vs = np.where(vs >= 720, 719, vs)

        uvs = np.vstack((vs, us)).T
        uvs_t = tuple(map(tuple, uvs.T))

        bmask_transformed = ndimage.distance_transform_edt(255 - self.bmask)
        vis = bmask_transformed[uvs_t]

        tracking_img = self.cur_image.copy()
        for i in range(len(image_coords)):
            uv = (us[i], vs[i])
            if vis[i] < mask_dis_threshold:
                cv2.circle(tracking_img, uv, 5, (255, 150, 0), -1)
            else:
                cv2.circle(tracking_img, uv, 5, (255, 0, 0), -1)

            if i != len(image_coords)-1:
                if vis[i] < mask_dis_threshold:
                    cv2.line(tracking_img, uv, (us[i+1], vs[i+1]), (0, 255, 0), 2)
                else:
                    cv2.line(tracking_img, uv, (us[i+1], vs[i+1]), (255, 0, 0), 2)

        tracking_img_msg = self.bridge.cv2_to_imgmsg(tracking_img, 'rgb8')
        self.tracking_img_pub.publish(tracking_img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingResultImg()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
