#include "trackdlo_perception/pipeline_manager.hpp"
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include "trackdlo_perception/pointcloud_cuda.cuh"

namespace trackdlo_perception {

PipelineManager::PipelineManager(bool use_external_mask, bool multi_color_dlo, const std::vector<int>& lower, const std::vector<int>& upper) {
    preprocessor_ = std::make_unique<trackdlo_perception::ImagePreprocessor>(use_external_mask, multi_color_dlo, lower, upper);
    visibility_checker_ = std::make_unique<trackdlo_perception::VisibilityChecker>();
    visualizer_ = std::make_unique<trackdlo_perception::Visualizer>();
}

void PipelineManager::set_parameters(
    double visibility_threshold,
    int dlo_pixel_width,
    double downsample_leaf_size,
    double d_vis,
    int max_zero_visible_frames)
{
    visibility_threshold_ = visibility_threshold;
    dlo_pixel_width_ = dlo_pixel_width;
    downsample_leaf_size_ = downsample_leaf_size;
    d_vis_ = d_vis;
    max_zero_visible_frames_ = max_zero_visible_frames;
}

void PipelineManager::set_tracker_parameters(
    double beta, double beta_pre_proc, double lambda, double lambda_pre_proc,
    double alpha, double lle_weight, double mu, int max_iter, double tol, double k_vis)
{
    tracker_.set_beta(beta);
    tracker_.set_beta_pre_proc(beta_pre_proc);
    tracker_.set_lambda(lambda);
    tracker_.set_lambda_pre_proc(lambda_pre_proc);
    tracker_.set_alpha(alpha);
    tracker_.set_lle_weight(lle_weight);
    tracker_.set_mu(mu);
    tracker_.set_max_iter(max_iter);
    tracker_.set_tol(tol);
    tracker_.set_k_vis(k_vis);
}

void PipelineManager::initialize_tracker(const Eigen::MatrixXd& init_nodes, const std::vector<double>& converted_node_coord)
{
    Y_ = init_nodes;
    converted_node_coord_ = converted_node_coord;
    tracker_.initialize_nodes(Y_);
    initialized_ = true;
    zero_visible_count_ = 0;
}

void PipelineManager::set_external_mask(const cv::Mat& mask) {
    preprocessor_->set_external_mask(mask);
}

void PipelineManager::set_occlusion_mask(const cv::Mat& mask) {
    preprocessor_->set_occlusion_mask(mask);
}

PipelineResult PipelineManager::process(const cv::Mat& cur_image_orig, const cv::Mat& depth_image, const Eigen::MatrixXd& proj_matrix)
{
    PipelineResult result;
    result.success = false;
    result.request_reinit = false;

    if (!initialized_) {
        return result;
    }

    cv::Mat mask, cur_image;
    if (!preprocessor_->process(cur_image_orig, mask, cur_image)) {
        return result;
    }

    result.mask = mask;
    result.cur_image = cur_image;

    // Point cloud generation and downsampling on GPU
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_ptr = trackdlo_perception::cuda::generate_downsampled_pointcloud(
        mask, depth_image, cur_image_orig, proj_matrix, downsample_leaf_size_);
    
    if (downsampled_ptr) {
        result.cur_pc_downsampled = *downsampled_ptr;
    }

    Eigen::MatrixXd X = result.cur_pc_downsampled.getMatrixXfMap().topRows(3).transpose().cast<double>();

    // Visibility mapping
    trackdlo_perception::VisibilityResult vis_res = visibility_checker_->check_visibility(Y_, X, proj_matrix, mask, visibility_threshold_, dlo_pixel_width_);
    std::vector<int> visible_nodes = vis_res.visible_nodes;
    result.not_self_occluded_nodes = vis_res.not_self_occluded_nodes;

    if (visible_nodes.size() < 3) {
        if (visible_nodes.empty()) {
            zero_visible_count_++;
            if (zero_visible_count_ >= max_zero_visible_frames_) {
                result.request_reinit = true;
            }
        }
        result.tracking_img = visualizer_->draw_tracking_image(cur_image_orig, cur_image, Y_, proj_matrix, result.not_self_occluded_nodes);
        return result;
    }
    zero_visible_count_ = 0;

    if (X.rows() < 3) {
        result.tracking_img = visualizer_->draw_tracking_image(cur_image_orig, cur_image, Y_, proj_matrix, result.not_self_occluded_nodes);
        return result;
    }

    // extended visible nodes array
    std::vector<int> visible_nodes_extended = {};
    for (size_t i = 0; i + 1 < visible_nodes.size(); i++) {
        visible_nodes_extended.push_back(visible_nodes[i]);
        if (fabs(converted_node_coord_[visible_nodes[i + 1]] - converted_node_coord_[visible_nodes[i]]) <= d_vis_) {
            for (int j = 1; j < visible_nodes[i + 1] - visible_nodes[i]; j++) {
                visible_nodes_extended.push_back(visible_nodes[i] + j);
            }
        }
    }
    visible_nodes_extended.push_back(visible_nodes.back());

    // run tracking
    tracker_.tracking_step(X, visible_nodes, visible_nodes_extended, proj_matrix, mask.rows, mask.cols);
    Y_ = tracker_.get_tracking_result();
    result.Y = Y_;
    result.guide_nodes = tracker_.get_guide_nodes();
    result.priors = tracker_.get_correspondence_pairs();

    // visualization and PC generation
    result.tracking_img = visualizer_->draw_tracking_image(cur_image_orig, cur_image, Y_, proj_matrix, result.not_self_occluded_nodes);

    for (int i = 0; i < Y_.rows(); i++) {
        pcl::PointXYZ temp;
        temp.x = Y_(i, 0);
        temp.y = Y_(i, 1);
        temp.z = Y_(i, 2);
        result.trackdlo_pc.points.push_back(temp);
    }

    std::vector<int> self_occluded_nodes;
    for (int i = 0; i < Y_.rows(); ++i) {
        if (std::find(result.not_self_occluded_nodes.begin(), result.not_self_occluded_nodes.end(), i) == result.not_self_occluded_nodes.end()) {
            self_occluded_nodes.push_back(i);
        }
    }
    for (auto i : self_occluded_nodes) {
        pcl::PointXYZ temp;
        temp.x = Y_(i, 0);
        temp.y = Y_(i, 1);
        temp.z = Y_(i, 2);
        result.self_occluded_pc.points.push_back(temp);
    }

    result.success = true;
    return result;
}

} // namespace trackdlo_perception
