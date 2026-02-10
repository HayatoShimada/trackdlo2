#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
import sensor_msgs_py.point_cloud2 as pcl2
import std_msgs.msg
from cv_bridge import CvBridge
import message_filters

import struct
import time
import cv2
import numpy as np

from visualization_msgs.msg import MarkerArray
from scipy import interpolate

from trackdlo_perception.utils import extract_connected_skeleton, ndarray2MarkerArray


class InitTrackerNode(Node):
    def __init__(self):
        super().__init__('init_tracker')
        self.bridge = CvBridge()
        self.proj_matrix = None

        # Declare parameters
        self.declare_parameter('num_of_nodes', 30)
        self.declare_parameter('multi_color_dlo', False)
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('result_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('visualize_initialization_process', False)
        self.declare_parameter('hsv_threshold_upper_limit', '130 255 255')
        self.declare_parameter('hsv_threshold_lower_limit', '90 90 90')

        self.num_of_nodes = self.get_parameter('num_of_nodes').value
        self.multi_color_dlo = self.get_parameter('multi_color_dlo').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        self.result_frame_id = self.get_parameter('result_frame_id').value
        self.visualize_initialization_process = self.get_parameter('visualize_initialization_process').value

        hsv_threshold_upper_limit = self.get_parameter('hsv_threshold_upper_limit').value
        hsv_threshold_lower_limit = self.get_parameter('hsv_threshold_lower_limit').value

        upper_array = hsv_threshold_upper_limit.split(' ')
        lower_array = hsv_threshold_lower_limit.split(' ')
        self.upper = (int(upper_array[0]), int(upper_array[1]), int(upper_array[2]))
        self.lower = (int(lower_array[0]), int(lower_array[1]), int(lower_array[2]))

        # Camera info subscriber (will be destroyed after first message)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        # Header and fields for point cloud publishing
        self.header = std_msgs.msg.Header()
        self.header.stamp = self.get_clock().now().to_msg()
        self.header.frame_id = self.result_frame_id
        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),
        ]

        # Publishers
        self.pc_pub = self.create_publisher(PointCloud2, '/trackdlo/init_nodes', 10)
        self.results_pub = self.create_publisher(MarkerArray, '/trackdlo/init_nodes_markers', 10)

        # Message filter subscribers for synchronized rgb + depth
        self.rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.callback)

    def camera_info_callback(self, info):
        self.proj_matrix = np.array(list(info.p)).reshape(3, 4)
        self.get_logger().info('Received camera projection matrix:')
        self.get_logger().info(str(self.proj_matrix))
        # Unsubscribe after first message
        self.destroy_subscription(self.camera_info_sub)

    def color_thresholding(self, hsv_image, cur_depth):
        mask_dlo = cv2.inRange(hsv_image.copy(), self.lower, self.upper).astype('uint8')

        # tape green
        lower_green = (58, 130, 50)
        upper_green = (90, 255, 89)
        mask_green = cv2.inRange(hsv_image.copy(), lower_green, upper_green).astype('uint8')

        # combine masks
        mask = cv2.bitwise_or(mask_green.copy(), mask_dlo.copy())

        # filter mask based on depth values
        mask[cur_depth < 0.57*1000] = 0

        return mask, mask_green

    def remove_duplicate_rows(self, array):
        _, idx = np.unique(array, axis=0, return_index=True)
        data = array[np.sort(idx)]
        self.set_parameters([Parameter('num_of_nodes', Parameter.Type.INTEGER, len(data))])
        return data

    def callback(self, rgb, depth):
        if self.proj_matrix is None:
            return

        self.get_logger().info("Initializing...")

        # Process rgb image
        cur_image = self.bridge.imgmsg_to_cv2(rgb, 'rgb8')
        hsv_image = cv2.cvtColor(cur_image.copy(), cv2.COLOR_RGB2HSV)

        # Process depth image
        cur_depth = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')

        if not self.multi_color_dlo:
            mask = cv2.inRange(hsv_image, self.lower, self.upper)
        else:
            mask, mask_tip = self.color_thresholding(hsv_image, cur_depth)

        try:
            start_time = time.time()
            mask = cv2.cvtColor(mask.copy(), cv2.COLOR_GRAY2BGR)

            img_scale = 1
            extracted_chains = extract_connected_skeleton(
                self.visualize_initialization_process, mask,
                img_scale=img_scale, seg_length=8, max_curvature=25)

            all_pixel_coords = []
            for chain in extracted_chains:
                all_pixel_coords += chain
            self.get_logger().info(f'Finished extracting chains. Time taken: {time.time()-start_time}')

            all_pixel_coords = np.array(all_pixel_coords) * img_scale
            all_pixel_coords = np.flip(all_pixel_coords, 1)

            pc_z = cur_depth[tuple(map(tuple, all_pixel_coords.T))] / 1000.0
            fx = self.proj_matrix[0, 0]
            fy = self.proj_matrix[1, 1]
            cx = self.proj_matrix[0, 2]
            cy = self.proj_matrix[1, 2]
            pixel_x = all_pixel_coords[:, 1]
            pixel_y = all_pixel_coords[:, 0]

            if self.multi_color_dlo:
                pixel_value1 = mask_tip[pixel_y[-1], pixel_x[-1]]
                if pixel_value1 == 255:
                    pixel_x, pixel_y = pixel_x[::-1], pixel_y[::-1]

            pc_x = (pixel_x - cx) * pc_z / fx
            pc_y = (pixel_y - cy) * pc_z / fy
            extracted_chains_3d = np.vstack((pc_x, pc_y))
            extracted_chains_3d = np.vstack((extracted_chains_3d, pc_z))
            extracted_chains_3d = extracted_chains_3d.T

            # do not include those without depth values
            extracted_chains_3d = extracted_chains_3d[
                ((extracted_chains_3d[:, 0] != 0) |
                 (extracted_chains_3d[:, 1] != 0) |
                 (extracted_chains_3d[:, 2] != 0))]

            if self.multi_color_dlo:
                depth_threshold = 0.57
                extracted_chains_3d = extracted_chains_3d[extracted_chains_3d[:, 2] > depth_threshold]

            tck, u = interpolate.splprep(extracted_chains_3d.T, s=0.0005)
            # 1st fit, less points
            u_fine = np.linspace(0, 1, 300)
            x_fine, y_fine, z_fine = interpolate.splev(u_fine, tck)
            spline_pts = np.vstack((x_fine, y_fine, z_fine)).T

            # 2nd fit, higher accuracy
            num_true_pts = int(np.sum(np.sqrt(np.sum(np.square(np.diff(spline_pts, axis=0)), axis=1))) * 1000)
            u_fine = np.linspace(0, 1, num_true_pts)
            x_fine, y_fine, z_fine = interpolate.splev(u_fine, tck)
            spline_pts = np.vstack((x_fine, y_fine, z_fine)).T

            nodes = spline_pts[np.linspace(0, num_true_pts-1, self.num_of_nodes).astype(int)]

            init_nodes = self.remove_duplicate_rows(nodes)
            results = ndarray2MarkerArray(
                init_nodes, self.result_frame_id,
                [0, 149/255, 203/255, 0.75], [0, 149/255, 203/255, 0.75])
            self.results_pub.publish(results)

            # add color
            pc_rgba = struct.unpack('I', struct.pack('BBBB', 255, 40, 40, 255))[0]
            pc_rgba_arr = np.full((len(init_nodes), 1), pc_rgba)
            pc_colored = np.hstack((init_nodes, pc_rgba_arr)).astype(object)
            pc_colored[:, 3] = pc_colored[:, 3].astype(int)

            self.header.stamp = self.get_clock().now().to_msg()
            converted_points = pcl2.create_cloud(self.header, self.fields, pc_colored)
            self.pc_pub.publish(converted_points)
        except Exception:
            self.get_logger().error("Failed to extract splines.")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = InitTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
