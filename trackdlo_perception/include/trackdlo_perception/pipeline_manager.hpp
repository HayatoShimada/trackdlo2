#ifndef TRACKDLO_PIPELINE_MANAGER_HPP
#define TRACKDLO_PIPELINE_MANAGER_HPP

#include "trackdlo_perception/trackdlo.hpp"
#include "trackdlo_perception/image_preprocessor.hpp"
#include "trackdlo_perception/visibility_checker.hpp"
#include "trackdlo_perception/visualizer.hpp"

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <memory>

namespace trackdlo_perception {

struct PipelineResult {
    bool success;
    bool request_reinit; // True if tracking failed for too many frames and needs re-initialization
    cv::Mat tracking_img;
    cv::Mat mask;
    cv::Mat cur_image;
    Eigen::MatrixXd Y;
    Eigen::MatrixXd guide_nodes;
    std::vector<Eigen::MatrixXd> priors;
    pcl::PointCloud<pcl::PointXYZRGB> cur_pc_downsampled;
    pcl::PointCloud<pcl::PointXYZ> trackdlo_pc;
    pcl::PointCloud<pcl::PointXYZ> self_occluded_pc;
    std::vector<int> not_self_occluded_nodes;
};

class PipelineManager {
public:
    PipelineManager(bool use_external_mask, bool multi_color_dlo, const std::vector<int>& lower, const std::vector<int>& upper);
    ~PipelineManager() = default;

    void set_parameters(
        double visibility_threshold,
        int dlo_pixel_width,
        double downsample_leaf_size,
        double d_vis,
        int max_zero_visible_frames
    );

    void set_tracker_parameters(
        double beta, double beta_pre_proc, double lambda, double lambda_pre_proc,
        double alpha, double lle_weight, double mu, int max_iter, double tol, double k_vis
    );

    void initialize_tracker(const Eigen::MatrixXd& init_nodes, const std::vector<double>& converted_node_coord);

    PipelineResult process(const cv::Mat& cur_image_orig, const cv::Mat& depth_image, const Eigen::MatrixXd& proj_matrix);

    void set_external_mask(const cv::Mat& mask);
    void set_occlusion_mask(const cv::Mat& mask);

    bool is_initialized() const { return initialized_; }

private:
    std::unique_ptr<trackdlo_perception::ImagePreprocessor> preprocessor_;
    std::unique_ptr<trackdlo_perception::VisibilityChecker> visibility_checker_;
    std::unique_ptr<trackdlo_perception::Visualizer> visualizer_;
    trackdlo tracker_;

    bool initialized_ = false;
    Eigen::MatrixXd Y_;
    std::vector<double> converted_node_coord_;

    // Parameters
    double visibility_threshold_ = 0.02;
    int dlo_pixel_width_ = 20;
    double downsample_leaf_size_ = 0.02;
    double d_vis_ = 0.05;
    int max_zero_visible_frames_ = 30;
    int zero_visible_count_ = 0;
};

} // namespace trackdlo_perception

#endif // TRACKDLO_PIPELINE_MANAGER_HPP
