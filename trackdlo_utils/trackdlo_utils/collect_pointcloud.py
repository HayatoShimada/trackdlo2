#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2

import cv2
import numpy as np
import pickle as pkl
import os
from os.path import dirname, abspath, join


class CollectPointCloud(Node):
    def __init__(self):
        super().__init__('record_data')
        self.bridge = CvBridge()

        self.cur_pc = []
        self.cur_image_arr = []
        self.cur_result = []
        self.cur_tracking_image_arr = []

        self.create_subscription(
            Image, '/camera/color/image_raw', self.update_img, 10)
        self.create_subscription(
            Image, '/tracking_img', self.update_tracking_img, 10)
        self.create_subscription(
            PointCloud2, '/trackdlo_results_pc', self.update_cur_result, 10)
        self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.update_cur_pc, 10)

    def update_cur_pc(self, data):
        points = list(pc2.read_points(data, field_names=('x', 'y', 'z'), skip_nans=True))
        self.cur_pc = np.array(points)

    def update_cur_result(self, data):
        points = list(pc2.read_points(data, field_names=('x', 'y', 'z'), skip_nans=True))
        self.cur_result = np.array(points)

    def update_img(self, data):
        cur_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        self.cur_image_arr = cur_image

    def update_tracking_img(self, data):
        self.cur_tracking_image_arr = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def record(self, main_dir, start=0, save_image=False, save_results=False):
        i = start

        while rclpy.ok():
            sample_id = ''
            if len(str(i)) == 1:
                sample_id = '00' + str(i)
            else:
                sample_id = '0' + str(i)

            print("======================================================================")
            print("Press enter to collect and save point cloud and camera pose data")
            print("Press q + enter to exit the program")
            key_pressed = input("sample_id = " + sample_id + "\n")

            if key_pressed == 'q':
                print("Shutting down... \n")
                rclpy.shutdown()
                break
            else:
                if save_image:
                    if len(self.cur_image_arr) == 0:
                        print(" ")
                        print("Could not capture image, please try again! \n")
                        continue
                    cv2.imwrite(main_dir + sample_id + "_rgb.png", self.cur_image_arr)

                if save_results:
                    if len(self.cur_result) == 0:
                        print(" ")
                        print("Could not capture results, please try again! \n")
                        continue

                    f = open(main_dir + sample_id + "_results.json", "wb")
                    pkl.dump(self.cur_result, f)
                    f.close()

                    if len(self.cur_tracking_image_arr) == 0:
                        print(" ")
                        print("Could not capture tracking image, please try again! \n")
                        continue
                    cv2.imwrite(main_dir + sample_id + "_result.png", self.cur_tracking_image_arr)

                if len(self.cur_pc) == 0:
                    print(" ")
                    print("Could not capture point cloud, please try again! \n")
                    continue

                f = open(main_dir + sample_id + "_pc.json", "wb")
                pkl.dump(self.cur_pc, f)
                f.close()

                print("Data saved successfully! \n")
                i += 1


def main(args=None):
    rclpy.init(args=args)
    node = CollectPointCloud()

    main_dir = join(dirname(dirname(abspath(__file__))), "data/")
    os.listdir(main_dir)
    print("######################################################################")
    print("Collected data will be saved at the following directory:")
    print(main_dir)
    print("###################################################################### \n")

    node.record(main_dir, start=0, save_image=True, save_results=True)
    node.destroy_node()


if __name__ == '__main__':
    main()
