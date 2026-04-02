#ifndef POINTCLOUD_CUDA_CUH
#define POINTCLOUD_CUDA_CUH

#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

namespace trackdlo_perception {
namespace cuda {

pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_pointcloud(
    const cv::Mat& mask,
    const cv::Mat& depth_image,
    const cv::Mat& color_image,
    const Eigen::MatrixXd& proj_matrix);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_downsampled_pointcloud(
    const cv::Mat& mask,
    const cv::Mat& depth_image,
    const cv::Mat& color_image,
    const Eigen::MatrixXd& proj_matrix,
    float leaf_size);

} // namespace cuda
} // namespace trackdlo_perception

#endif // POINTCLOUD_CUDA_CUH
