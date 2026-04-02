#include "trackdlo_perception/trackdlo.hpp"
#include "trackdlo_perception/utils.hpp"
#include "trackdlo_perception/pipeline_manager.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/header.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

using cv::Mat;
using Eigen::MatrixXd;
using Eigen::RowVectorXd;

class TrackDLONode : public rclcpp::Node
{
public:
    TrackDLONode()
    : Node("tracker_node"),
      received_init_nodes_(false),
      received_proj_matrix_(false),
      proj_matrix_(3, 4),
      algo_total_(0.0),
      pub_data_total_(0.0),
      frames_(0)
    {
        // Declare and get parameters

        // --- 追跡アルゴリズム (CPD-LLE) の主要パラメータ ---
        this->declare_parameter<double>("beta", 0.35);
        this->declare_parameter<double>("lambda", 50000.0);
        this->declare_parameter<double>("alpha", 3.0);
        this->declare_parameter<double>("mu", 0.1);
        this->declare_parameter<int>("max_iter", 50);
        this->declare_parameter<double>("tol", 0.0002);

        // --- 可視性・オクルージョン判定パラメータ ---
        this->declare_parameter<double>("k_vis", 50.0);
        this->declare_parameter<double>("d_vis", 0.06);
        this->declare_parameter<double>("visibility_threshold", 0.008);

        // --- 画像描画・事前処理パラメータ ---
        this->declare_parameter<int>("dlo_pixel_width", 40);
        this->declare_parameter<double>("beta_pre_proc", 3.0);
        this->declare_parameter<double>("lambda_pre_proc", 1.0);
        this->declare_parameter<double>("lle_weight", 10.0);

        // --- その他の動作設定 ---
        this->declare_parameter<bool>("multi_color_dlo", false);
        this->declare_parameter<double>("downsample_leaf_size", 0.008);

        this->declare_parameter<std::string>("camera_info_topic", "/camera/aligned_depth_to_color/camera_info");
        this->declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
        this->declare_parameter<std::string>("depth_topic", "/camera/aligned_depth_to_color/image_raw");
        this->declare_parameter<std::string>("result_frame_id", "camera_color_optical_frame");
        this->declare_parameter<std::string>("hsv_threshold_upper_limit", "130 255 255");
        this->declare_parameter<std::string>("hsv_threshold_lower_limit", "90 90 30");
        this->declare_parameter<bool>("use_external_mask", false);

        beta_ = this->get_parameter("beta").as_double();
        lambda_ = this->get_parameter("lambda").as_double();
        alpha_ = this->get_parameter("alpha").as_double();
        mu_ = this->get_parameter("mu").as_double();
        max_iter_ = this->get_parameter("max_iter").as_int();
        tol_ = this->get_parameter("tol").as_double();
        k_vis_ = this->get_parameter("k_vis").as_double();
        d_vis_ = this->get_parameter("d_vis").as_double();
        visibility_threshold_ = this->get_parameter("visibility_threshold").as_double();
        dlo_pixel_width_ = this->get_parameter("dlo_pixel_width").as_int();
        beta_pre_proc_ = this->get_parameter("beta_pre_proc").as_double();
        lambda_pre_proc_ = this->get_parameter("lambda_pre_proc").as_double();
        lle_weight_ = this->get_parameter("lle_weight").as_double();
        multi_color_dlo_ = this->get_parameter("multi_color_dlo").as_bool();
        downsample_leaf_size_ = this->get_parameter("downsample_leaf_size").as_double();

        camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
        rgb_topic_ = this->get_parameter("rgb_topic").as_string();
        depth_topic_ = this->get_parameter("depth_topic").as_string();
        result_frame_id_ = this->get_parameter("result_frame_id").as_string();
        std::string hsv_threshold_upper_limit = this->get_parameter("hsv_threshold_upper_limit").as_string();
        std::string hsv_threshold_lower_limit = this->get_parameter("hsv_threshold_lower_limit").as_string();
        use_external_mask_ = this->get_parameter("use_external_mask").as_bool();

        std::vector<int> upper;
        std::string rgb_val = "";
        for (size_t i = 0; i < hsv_threshold_upper_limit.length(); i++) {
            if (hsv_threshold_upper_limit.substr(i, 1) != " ") {
                rgb_val += hsv_threshold_upper_limit.substr(i, 1);
            }
            else {
                upper.push_back(std::stoi(rgb_val));
                rgb_val = "";
            }
            if (i == hsv_threshold_upper_limit.length() - 1) {
                upper.push_back(std::stoi(rgb_val));
            }
        }

        std::vector<int> lower;
        rgb_val = "";
        for (size_t i = 0; i < hsv_threshold_lower_limit.length(); i++) {
            if (hsv_threshold_lower_limit.substr(i, 1) != " ") {
                rgb_val += hsv_threshold_lower_limit.substr(i, 1);
            }
            else {
                lower.push_back(std::stoi(rgb_val));
                rgb_val = "";
            }
            if (i == hsv_threshold_lower_limit.length() - 1) {
                lower.push_back(std::stoi(rgb_val));
            }
        }

        pipeline_manager_ = std::make_unique<trackdlo_perception::PipelineManager>(use_external_mask_, multi_color_dlo_, lower, upper);
        update_pipeline_parameters();

        proj_matrix_.setZero();

        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(0),
            [this]() {
                this->init();
                this->init_timer_->cancel();
            }
        );

        param_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & params) {
                for (const auto & p : params) {
                    const auto & name = p.get_name();
                    if (name == "beta") {
                        beta_ = p.as_double();
                    } else if (name == "lambda") {
                        lambda_ = p.as_double();
                    } else if (name == "alpha") {
                        alpha_ = p.as_double();
                    } else if (name == "mu") {
                        mu_ = p.as_double();
                    } else if (name == "max_iter") {
                        max_iter_ = p.as_int();
                    } else if (name == "tol") {
                        tol_ = p.as_double();
                    } else if (name == "k_vis") {
                        k_vis_ = p.as_double();
                    } else if (name == "d_vis") {
                        d_vis_ = p.as_double();
                    } else if (name == "visibility_threshold") {
                        visibility_threshold_ = p.as_double();
                    } else if (name == "dlo_pixel_width") {
                        dlo_pixel_width_ = p.as_int();
                    } else if (name == "downsample_leaf_size") {
                        downsample_leaf_size_ = p.as_double();
                    } else if (name == "beta_pre_proc") {
                        beta_pre_proc_ = p.as_double();
                    } else if (name == "lambda_pre_proc") {
                        lambda_pre_proc_ = p.as_double();
                    } else if (name == "lle_weight") {
                        lle_weight_ = p.as_double();
                    } else {
                        continue;
                    }
                }
                update_pipeline_parameters();
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                return result;
            });
    }

    void init()
    {
        int pub_queue_size = 30;

        image_transport::ImageTransport it(shared_from_this());

        opencv_mask_sub_ = it.subscribe(
            "/mask_with_occlusion", 10,
            std::bind(&TrackDLONode::update_opencv_mask, this, std::placeholders::_1));

        if (use_external_mask_) {
            external_mask_sub_ = it.subscribe(
                "/trackdlo/segmentation_mask", 10,
                std::bind(&TrackDLONode::update_external_mask, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "External mask mode enabled. Subscribing to /trackdlo/segmentation_mask");
        }

        init_nodes_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/trackdlo/init_nodes", 1,
            std::bind(&TrackDLONode::update_init_nodes, this, std::placeholders::_1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_, 1,
            std::bind(&TrackDLONode::update_camera_info, this, std::placeholders::_1));

        tracking_img_pub_ = it.advertise("/trackdlo/results_img", pub_queue_size);
        seg_mask_pub_ = it.advertise("/trackdlo/segmentation_mask_img", pub_queue_size);
        seg_overlay_pub_ = it.advertise("/trackdlo/segmentation_overlay", pub_queue_size);

        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/trackdlo/filtered_pointcloud", pub_queue_size);
        results_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/trackdlo/results_marker", pub_queue_size);
        guide_nodes_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/trackdlo/guide_nodes", pub_queue_size);
        corr_priors_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/trackdlo/corr_priors", pub_queue_size);
        result_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/trackdlo/results_pc", pub_queue_size);
        self_occluded_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/trackdlo/self_occluded_pc", pub_queue_size);

        image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            shared_from_this(), rgb_topic_, rmw_qos_profile_default);
        depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            shared_from_this(), depth_topic_, rmw_qos_profile_default);

        sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
            ApproxSyncPolicy(10), *image_sub_, *depth_sub_);

        sync_->registerCallback(
            std::bind(&TrackDLONode::Callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO_STREAM(this->get_logger(), "TrackDLO node initialized.");
    }

private:
    using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    // ---------- Publishers ----------
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr results_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr guide_nodes_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corr_priors_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr self_occluded_pc_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr result_pc_pub_;

    image_transport::Publisher tracking_img_pub_;
    image_transport::Publisher seg_mask_pub_;
    image_transport::Publisher seg_overlay_pub_;

    // ---------- Subscribers ----------
    image_transport::Subscriber opencv_mask_sub_;
    image_transport::Subscriber external_mask_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr init_nodes_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // ---------- Synchronized subscribers ----------
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;

    // ---------- Timer for deferred init ----------
    rclcpp::TimerBase::SharedPtr init_timer_;

    // ---------- Parameter callback ----------
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // ---------- Pipeline Manager ----------
    std::unique_ptr<trackdlo_perception::PipelineManager> pipeline_manager_;

    // ---------- Tracker state ----------
    bool received_init_nodes_;
    bool received_proj_matrix_;
    bool reinit_requested_{false};
    MatrixXd init_nodes_;
    MatrixXd proj_matrix_;

    // ---------- Parameters ----------
    bool multi_color_dlo_;
    double visibility_threshold_;
    int dlo_pixel_width_;
    double beta_;
    double beta_pre_proc_;
    double lambda_;
    double lambda_pre_proc_;
    double alpha_;
    double lle_weight_;
    double mu_;
    int max_iter_;
    double tol_;
    double k_vis_;
    double d_vis_;
    double downsample_leaf_size_;

    std::string camera_info_topic_;
    std::string rgb_topic_;
    std::string depth_topic_;
    std::string result_frame_id_;
    bool use_external_mask_;

    // ---------- Timing ----------
    double algo_total_;
    double pub_data_total_;
    int frames_;

    void update_pipeline_parameters() {
        if (pipeline_manager_) {
            pipeline_manager_->set_parameters(visibility_threshold_, dlo_pixel_width_, downsample_leaf_size_, d_vis_, 30);
            pipeline_manager_->set_tracker_parameters(beta_, beta_pre_proc_, lambda_, lambda_pre_proc_, alpha_, lle_weight_, mu_, max_iter_, tol_, k_vis_);
        }
    }

    void update_opencv_mask(const sensor_msgs::msg::Image::ConstSharedPtr& opencv_mask_msg)
    {
        pipeline_manager_->set_occlusion_mask(cv_bridge::toCvShare(opencv_mask_msg, "bgr8")->image);
    }

    void update_external_mask(const sensor_msgs::msg::Image::ConstSharedPtr& mask_msg)
    {
        pipeline_manager_->set_external_mask(cv_bridge::toCvShare(mask_msg, "mono8")->image);
    }

    void update_init_nodes(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_msg)
    {
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        pcl_conversions::toPCL(*pc_msg, *cloud);
        pcl::PointCloud<pcl::PointXYZRGB> cloud_xyz;
        pcl::fromPCLPointCloud2(*cloud, cloud_xyz);
        delete cloud;

        init_nodes_ = cloud_xyz.getMatrixXfMap().topRows(3).transpose().cast<double>();

        if (!received_init_nodes_) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Received " << init_nodes_.rows() << " init nodes");
        }
        received_init_nodes_ = true;

        if (reinit_requested_) {
            RCLCPP_WARN(this->get_logger(),
                "Re-initializing tracker with %d init nodes",
                static_cast<int>(init_nodes_.rows()));

            std::vector<double> converted_node_coord;
            double cur_sum = 0;
            converted_node_coord.push_back(0.0);
            for (int i = 0; i < init_nodes_.rows() - 1; i++) {
                cur_sum += (init_nodes_.row(i + 1) - init_nodes_.row(i)).norm();
                converted_node_coord.push_back(cur_sum);
            }
            pipeline_manager_->initialize_tracker(init_nodes_, converted_node_coord);
            reinit_requested_ = false;
        }
    }

    void update_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_msg)
    {
        auto P = cam_msg->p;
        for (size_t i = 0; i < P.size(); i++) {
            proj_matrix_(i / 4, i % 4) = P[i];
        }
        received_proj_matrix_ = true;
        camera_info_sub_.reset();
    }

    void Callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
    {
        Mat cur_image_orig = cv_bridge::toCvShare(image_msg, "bgr8")->image;
        Mat cur_depth = cv_bridge::toCvShare(depth_msg, depth_msg->encoding)->image;

        sensor_msgs::msg::Image::SharedPtr tracking_img_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cur_image_orig).toImageMsg();

        if (!pipeline_manager_->is_initialized()) {
            if (received_init_nodes_ && received_proj_matrix_) {
                std::vector<double> converted_node_coord;
                double cur_sum = 0;
                converted_node_coord.push_back(0.0);
                for (int i = 0; i < init_nodes_.rows() - 1; i++) {
                    cur_sum += (init_nodes_.row(i + 1) - init_nodes_.row(i)).norm();
                    converted_node_coord.push_back(cur_sum);
                }
                pipeline_manager_->initialize_tracker(init_nodes_, converted_node_coord);
            }
        }
        else {
            std::chrono::high_resolution_clock::time_point cur_time_cb = std::chrono::high_resolution_clock::now();

            trackdlo_perception::PipelineResult result = pipeline_manager_->process(cur_image_orig, cur_depth, proj_matrix_);

            if (!result.success) {
                if (result.request_reinit && !reinit_requested_) {
                    RCLCPP_WARN(this->get_logger(), "Requesting re-initialization.");
                    reinit_requested_ = true;
                }
                if (!result.tracking_img.empty()) {
                    tracking_img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result.tracking_img).toImageMsg();
                }
                tracking_img_pub_.publish(tracking_img_msg);
                return;
            }

            // Publish masks and overlays
            if (!result.mask.empty() && !result.cur_image.empty()) {
                seg_mask_pub_.publish(
                    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", result.mask).toImageMsg());

                Mat seg_overlay;
                cur_image_orig.copyTo(seg_overlay);
                Mat color_layer(seg_overlay.size(), CV_8UC3, cv::Scalar(0, 255, 0));
                color_layer.copyTo(seg_overlay, result.mask);
                cv::addWeighted(cur_image_orig, 0.6, seg_overlay, 0.4, 0, seg_overlay);
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(result.mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                cv::drawContours(seg_overlay, contours, -1, cv::Scalar(0, 255, 0), 2);
                seg_overlay_pub_.publish(
                    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", seg_overlay).toImageMsg());
            }

            double time_diff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - cur_time_cb).count() / 1000.0;
            RCLCPP_INFO_STREAM(this->get_logger(), "Pipeline processing: " + std::to_string(time_diff) + " ms");
            algo_total_ += time_diff;

            std::chrono::high_resolution_clock::time_point cur_time_pub = std::chrono::high_resolution_clock::now();

            tracking_img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result.tracking_img).toImageMsg();

            visualization_msgs::msg::MarkerArray results_msg = MatrixXd2MarkerArray(
                result.Y, result_frame_id_, "node_results", 
                {1.0, 150.0/255.0, 0.0, 1.0}, {0.0, 1.0, 0.0, 1.0}, 0.01, 0.005, 
                result.not_self_occluded_nodes, {1.0, 0.0, 0.0, 1.0}, {1.0, 0.0, 0.0, 1.0});
            
            visualization_msgs::msg::MarkerArray guide_nodes_results = MatrixXd2MarkerArray(
                result.guide_nodes, result_frame_id_, "guide_node_results", 
                {0.0, 0.0, 0.0, 0.5}, {0.0, 0.0, 1.0, 0.5});
            
            visualization_msgs::msg::MarkerArray corr_priors_results = MatrixXd2MarkerArray(
                result.priors, result_frame_id_, "corr_prior_results", 
                {0.0, 0.0, 0.0, 0.5}, {1.0, 0.0, 0.0, 0.5});

            pcl::PCLPointCloud2 cur_pc_pointcloud2, result_pc_poincloud2, self_occluded_pc_poincloud2;
            pcl::toPCLPointCloud2(result.cur_pc_downsampled, cur_pc_pointcloud2);
            pcl::toPCLPointCloud2(result.trackdlo_pc, result_pc_poincloud2);
            pcl::toPCLPointCloud2(result.self_occluded_pc, self_occluded_pc_poincloud2);

            sensor_msgs::msg::PointCloud2 cur_pc_msg, result_pc_msg, self_occluded_pc_msg;
            pcl_conversions::moveFromPCL(cur_pc_pointcloud2, cur_pc_msg);
            pcl_conversions::moveFromPCL(result_pc_poincloud2, result_pc_msg);
            pcl_conversions::moveFromPCL(self_occluded_pc_poincloud2, self_occluded_pc_msg);

            cur_pc_msg.header.frame_id = result_frame_id_;
            result_pc_msg.header.frame_id = result_frame_id_;
            result_pc_msg.header.stamp = image_msg->header.stamp;
            self_occluded_pc_msg.header.frame_id = result_frame_id_;
            self_occluded_pc_msg.header.stamp = image_msg->header.stamp;

            results_pub_->publish(results_msg);
            guide_nodes_pub_->publish(guide_nodes_results);
            corr_priors_pub_->publish(corr_priors_results);
            pc_pub_->publish(cur_pc_msg);
            result_pc_pub_->publish(result_pc_msg);
            self_occluded_pc_pub_->publish(self_occluded_pc_msg);

            for (size_t i = 0; i < guide_nodes_results.markers.size(); i++) {
                guide_nodes_results.markers[i].action = visualization_msgs::msg::Marker::DELETEALL;
            }
            for (size_t i = 0; i < corr_priors_results.markers.size(); i++) {
                corr_priors_results.markers[i].action = visualization_msgs::msg::Marker::DELETEALL;
            }

            time_diff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - cur_time_pub).count() / 1000.0;
            RCLCPP_INFO_STREAM(this->get_logger(), "Pub data: " + std::to_string(time_diff) + " ms");
            pub_data_total_ += time_diff;

            frames_ += 1;
            RCLCPP_INFO_STREAM(this->get_logger(), "Avg tracking step: " + std::to_string(algo_total_ / frames_) + " ms");
            RCLCPP_INFO_STREAM(this->get_logger(), "Avg total: " + std::to_string((algo_total_ + pub_data_total_) / frames_) + " ms");
        }

        tracking_img_pub_.publish(tracking_img_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrackDLONode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
