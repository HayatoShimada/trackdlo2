#include "trackdlo_perception/trackdlo.hpp"
#include "trackdlo_perception/utils.hpp"

#include <rclcpp/rclcpp.hpp>
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
      initialized_(false),
      received_init_nodes_(false),
      received_proj_matrix_(false),
      updated_opencv_mask_(false),
      proj_matrix_(3, 4),
      converted_node_coord_({0.0}),
      sigma2_(0.0),
      pre_proc_total_(0.0),
      algo_total_(0.0),
      pub_data_total_(0.0),
      frames_(0)
    {
        // Declare and get parameters
        this->declare_parameter<double>("beta", 0.35);
        this->declare_parameter<double>("lambda", 50000.0);
        this->declare_parameter<double>("alpha", 3.0);
        this->declare_parameter<double>("mu", 0.1);
        this->declare_parameter<int>("max_iter", 50);
        this->declare_parameter<double>("tol", 0.0002);
        this->declare_parameter<double>("k_vis", 50.0);
        this->declare_parameter<double>("d_vis", 0.06);
        this->declare_parameter<double>("visibility_threshold", 0.008);
        this->declare_parameter<int>("dlo_pixel_width", 40);
        this->declare_parameter<double>("beta_pre_proc", 3.0);
        this->declare_parameter<double>("lambda_pre_proc", 1.0);
        this->declare_parameter<double>("lle_weight", 10.0);
        this->declare_parameter<bool>("multi_color_dlo", false);
        this->declare_parameter<double>("downsample_leaf_size", 0.008);

        this->declare_parameter<std::string>("camera_info_topic", "/camera/aligned_depth_to_color/camera_info");
        this->declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
        this->declare_parameter<std::string>("depth_topic", "/camera/aligned_depth_to_color/image_raw");
        this->declare_parameter<std::string>("result_frame_id", "camera_color_optical_frame");
        this->declare_parameter<std::string>("hsv_threshold_upper_limit", "130 255 255");
        this->declare_parameter<std::string>("hsv_threshold_lower_limit", "90 90 30");

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

        // Parse HSV upper threshold
        std::string rgb_val = "";
        for (size_t i = 0; i < hsv_threshold_upper_limit.length(); i++) {
            if (hsv_threshold_upper_limit.substr(i, 1) != " ") {
                rgb_val += hsv_threshold_upper_limit.substr(i, 1);
            }
            else {
                upper_.push_back(std::stoi(rgb_val));
                rgb_val = "";
            }

            if (i == hsv_threshold_upper_limit.length() - 1) {
                upper_.push_back(std::stoi(rgb_val));
            }
        }

        // Parse HSV lower threshold
        rgb_val = "";
        for (size_t i = 0; i < hsv_threshold_lower_limit.length(); i++) {
            if (hsv_threshold_lower_limit.substr(i, 1) != " ") {
                rgb_val += hsv_threshold_lower_limit.substr(i, 1);
            }
            else {
                lower_.push_back(std::stoi(rgb_val));
                rgb_val = "";
            }

            if (i == hsv_threshold_lower_limit.length() - 1) {
                lower_.push_back(std::stoi(rgb_val));
            }
        }

        proj_matrix_.setZero();

        // Use a one-shot timer to call init() after construction, so that
        // shared_from_this() is available for image_transport.
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(0),
            [this]() {
                this->init();
                this->init_timer_->cancel();
            }
        );
    }

    void init()
    {
        int pub_queue_size = 30;

        // image_transport requires shared_from_this(), so it must be called
        // after the constructor returns and the shared_ptr is fully formed.
        image_transport::ImageTransport it(shared_from_this());

        opencv_mask_sub_ = it.subscribe(
            "/mask_with_occlusion", 10,
            std::bind(&TrackDLONode::update_opencv_mask, this, std::placeholders::_1));

        init_nodes_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/trackdlo/init_nodes", 1,
            std::bind(&TrackDLONode::update_init_nodes, this, std::placeholders::_1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_, 1,
            std::bind(&TrackDLONode::update_camera_info, this, std::placeholders::_1));

        tracking_img_pub_ = it.advertise("/trackdlo/results_img", pub_queue_size);

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

        // Set up synchronized image + depth subscribers
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
    // Typedef for the approximate time sync policy
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

    // ---------- Subscribers ----------
    image_transport::Subscriber opencv_mask_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr init_nodes_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // ---------- Synchronized subscribers ----------
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;

    // ---------- Timer for deferred init ----------
    rclcpp::TimerBase::SharedPtr init_timer_;

    // ---------- Tracker state ----------
    MatrixXd Y_;
    double sigma2_;
    bool initialized_;
    bool received_init_nodes_;
    bool received_proj_matrix_;
    MatrixXd init_nodes_;
    std::vector<double> converted_node_coord_;
    Mat occlusion_mask_;
    bool updated_opencv_mask_;
    MatrixXd proj_matrix_;

    trackdlo tracker_;

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
    std::vector<int> upper_;
    std::vector<int> lower_;

    // ---------- Timing ----------
    double pre_proc_total_;
    double algo_total_;
    double pub_data_total_;
    int frames_;

    // ---------- Callback: opencv mask ----------
    void update_opencv_mask(const sensor_msgs::msg::Image::ConstSharedPtr& opencv_mask_msg)
    {
        occlusion_mask_ = cv_bridge::toCvShare(opencv_mask_msg, "bgr8")->image;
        if (!occlusion_mask_.empty()) {
            updated_opencv_mask_ = true;
        }
    }

    // ---------- Callback: init nodes ----------
    void update_init_nodes(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_msg)
    {
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        pcl_conversions::toPCL(*pc_msg, *cloud);
        pcl::PointCloud<pcl::PointXYZRGB> cloud_xyz;
        pcl::fromPCLPointCloud2(*cloud, cloud_xyz);
        delete cloud;

        init_nodes_ = cloud_xyz.getMatrixXfMap().topRows(3).transpose().cast<double>();
        RCLCPP_INFO_STREAM(this->get_logger(), "Received " << init_nodes_.rows() << " init nodes");
        received_init_nodes_ = true;
        // Shut down the subscription by resetting the shared_ptr
        init_nodes_sub_.reset();
    }

    // ---------- Callback: camera info ----------
    void update_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_msg)
    {
        auto P = cam_msg->p;
        for (size_t i = 0; i < P.size(); i++) {
            proj_matrix_(i / 4, i % 4) = P[i];
        }
        received_proj_matrix_ = true;
        // Shut down the subscription by resetting the shared_ptr
        camera_info_sub_.reset();
    }

    // ---------- Color thresholding for multi-color DLO ----------
    Mat color_thresholding(Mat cur_image_hsv)
    {
        std::vector<int> lower_blue = {90, 90, 60};
        std::vector<int> upper_blue = {130, 255, 255};

        std::vector<int> lower_red_1 = {130, 60, 50};
        std::vector<int> upper_red_1 = {255, 255, 255};

        std::vector<int> lower_red_2 = {0, 60, 50};
        std::vector<int> upper_red_2 = {10, 255, 255};

        std::vector<int> lower_yellow = {15, 100, 80};
        std::vector<int> upper_yellow = {40, 255, 255};

        Mat mask_blue, mask_red_1, mask_red_2, mask_red, mask_yellow, mask;
        // filter blue
        cv::inRange(cur_image_hsv, cv::Scalar(lower_blue[0], lower_blue[1], lower_blue[2]), cv::Scalar(upper_blue[0], upper_blue[1], upper_blue[2]), mask_blue);

        // filter red
        cv::inRange(cur_image_hsv, cv::Scalar(lower_red_1[0], lower_red_1[1], lower_red_1[2]), cv::Scalar(upper_red_1[0], upper_red_1[1], upper_red_1[2]), mask_red_1);
        cv::inRange(cur_image_hsv, cv::Scalar(lower_red_2[0], lower_red_2[1], lower_red_2[2]), cv::Scalar(upper_red_2[0], upper_red_2[1], upper_red_2[2]), mask_red_2);

        // filter yellow
        cv::inRange(cur_image_hsv, cv::Scalar(lower_yellow[0], lower_yellow[1], lower_yellow[2]), cv::Scalar(upper_yellow[0], upper_yellow[1], upper_yellow[2]), mask_yellow);

        // combine red mask
        cv::bitwise_or(mask_red_1, mask_red_2, mask_red);
        // combine overall mask
        cv::bitwise_or(mask_red, mask_blue, mask);
        cv::bitwise_or(mask_yellow, mask, mask);

        return mask;
    }

    // ---------- Main synchronized callback ----------
    void Callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
    {
        Mat cur_image_orig = cv_bridge::toCvShare(image_msg, "bgr8")->image;
        Mat cur_depth = cv_bridge::toCvShare(depth_msg, depth_msg->encoding)->image;

        // will get overwritten later if initialized
        sensor_msgs::msg::Image::SharedPtr tracking_img_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cur_image_orig).toImageMsg();

        if (!initialized_) {
            if (received_init_nodes_ && received_proj_matrix_) {
                tracker_ = trackdlo(init_nodes_.rows(), visibility_threshold_, beta_, lambda_, alpha_, k_vis_, mu_, max_iter_, tol_, beta_pre_proc_, lambda_pre_proc_, lle_weight_);

                sigma2_ = 0.001;

                // record geodesic coord
                double cur_sum = 0;
                for (int i = 0; i < init_nodes_.rows() - 1; i++) {
                    cur_sum += (init_nodes_.row(i + 1) - init_nodes_.row(i)).norm();
                    converted_node_coord_.push_back(cur_sum);
                }

                tracker_.initialize_nodes(init_nodes_);
                tracker_.initialize_geodesic_coord(converted_node_coord_);
                Y_ = init_nodes_.replicate(1, 1);

                initialized_ = true;
            }
        }
        else {
            // log time
            std::chrono::high_resolution_clock::time_point cur_time_cb = std::chrono::high_resolution_clock::now();
            double time_diff;
            std::chrono::high_resolution_clock::time_point cur_time;

            Mat mask, mask_rgb, mask_without_occlusion_block;
            Mat cur_image_hsv;

            // convert color
            cv::cvtColor(cur_image_orig, cur_image_hsv, cv::COLOR_BGR2HSV);

            if (!multi_color_dlo_) {
                // color_thresholding
                cv::inRange(cur_image_hsv, cv::Scalar(lower_[0], lower_[1], lower_[2]), cv::Scalar(upper_[0], upper_[1], upper_[2]), mask_without_occlusion_block);
            }
            else {
                mask_without_occlusion_block = color_thresholding(cur_image_hsv);
            }

            // update cur image for visualization
            Mat cur_image;
            Mat occlusion_mask_gray;
            if (updated_opencv_mask_) {
                cv::cvtColor(occlusion_mask_, occlusion_mask_gray, cv::COLOR_BGR2GRAY);
                cv::bitwise_and(mask_without_occlusion_block, occlusion_mask_gray, mask);
                cv::bitwise_and(cur_image_orig, occlusion_mask_, cur_image);
            }
            else {
                mask_without_occlusion_block.copyTo(mask);
                cur_image_orig.copyTo(cur_image);
            }

            cv::cvtColor(mask, mask_rgb, cv::COLOR_GRAY2BGR);

            bool simulated_occlusion = false;
            int occlusion_corner_i = -1;
            int occlusion_corner_j = -1;
            int occlusion_corner_i_2 = -1;
            int occlusion_corner_j_2 = -1;

            // filter point cloud
            pcl::PointCloud<pcl::PointXYZRGB> cur_pc;
            pcl::PointCloud<pcl::PointXYZRGB> cur_pc_downsampled;

            // filter point cloud from mask
            for (int i = 0; i < mask.rows; i++) {
                for (int j = 0; j < mask.cols; j++) {
                    // for text label (visualization)
                    if (updated_opencv_mask_ && !simulated_occlusion && occlusion_mask_gray.at<uchar>(i, j) == 0) {
                        occlusion_corner_i = i;
                        occlusion_corner_j = j;
                        simulated_occlusion = true;
                    }

                    // update the other corner of occlusion mask (visualization)
                    if (updated_opencv_mask_ && occlusion_mask_gray.at<uchar>(i, j) == 0) {
                        occlusion_corner_i_2 = i;
                        occlusion_corner_j_2 = j;
                    }

                    if (mask.at<uchar>(i, j) != 0) {
                        // point cloud from image pixel coordinates and depth value
                        pcl::PointXYZRGB point;
                        double pixel_x = static_cast<double>(j);
                        double pixel_y = static_cast<double>(i);
                        double cx = proj_matrix_(0, 2);
                        double cy = proj_matrix_(1, 2);
                        double fx = proj_matrix_(0, 0);
                        double fy = proj_matrix_(1, 1);
                        double pc_z = cur_depth.at<uint16_t>(i, j) / 1000.0;

                        point.x = (pixel_x - cx) * pc_z / fx;
                        point.y = (pixel_y - cy) * pc_z / fy;
                        point.z = pc_z;

                        // currently something so color doesn't show up in rviz
                        point.r = cur_image_orig.at<cv::Vec3b>(i, j)[0];
                        point.g = cur_image_orig.at<cv::Vec3b>(i, j)[1];
                        point.b = cur_image_orig.at<cv::Vec3b>(i, j)[2];

                        cur_pc.push_back(point);
                    }
                }
            }

            // Perform downsampling
            pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudPtr(cur_pc.makeShared());
            pcl::VoxelGrid<pcl::PointXYZRGB> sor;
            sor.setInputCloud(cloudPtr);
            sor.setLeafSize(downsample_leaf_size_, downsample_leaf_size_, downsample_leaf_size_);
            sor.filter(cur_pc_downsampled);

            MatrixXd X = cur_pc_downsampled.getMatrixXfMap().topRows(3).transpose().cast<double>();
            RCLCPP_INFO_STREAM(this->get_logger(), "Number of points in downsampled point cloud: " + std::to_string(X.rows()));

            MatrixXd guide_nodes;
            std::vector<MatrixXd> priors;

            // log time
            time_diff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - cur_time_cb).count() / 1000.0;
            RCLCPP_INFO_STREAM(this->get_logger(), "Before tracking step: " + std::to_string(time_diff) + " ms");
            pre_proc_total_ += time_diff;
            cur_time = std::chrono::high_resolution_clock::now();

            // calculate node visibility
            // for each node in Y, determine its shortest distance to X
            // for each point in X, determine its shortest distance to Y
            std::map<int, double> shortest_node_pt_dists;
            std::vector<double> shortest_pt_node_dists(X.rows(), 100000.0);
            for (int m = 0; m < Y_.rows(); m++) {
                int closest_pt_idx = 0;
                double shortest_dist = 100000;
                // loop through all points in X
                for (int n = 0; n < X.rows(); n++) {
                    double dist = (Y_.row(m) - X.row(n)).norm();
                    // update shortest dist for Y
                    if (dist < shortest_dist) {
                        closest_pt_idx = n;
                        shortest_dist = dist;
                    }

                    // update shortest dist for X
                    if (dist < shortest_pt_node_dists[n]) {
                        shortest_pt_node_dists[n] = dist;
                    }
                }
                shortest_node_pt_dists.insert(std::pair<int, double>(m, shortest_dist));
            }

            // suppress unused variable warning
            (void)occlusion_corner_i_2;
            (void)occlusion_corner_j_2;

            // for current nodes and edges in Y, sort them based on how far away they are from the camera
            std::vector<double> averaged_node_camera_dists = {};
            std::vector<int> indices_vec = {};
            for (int i = 0; i < Y_.rows() - 1; i++) {
                averaged_node_camera_dists.push_back(((Y_.row(i) + Y_.row(i + 1)) / 2).norm());
                indices_vec.push_back(i);
            }
            // sort
            std::sort(indices_vec.begin(), indices_vec.end(),
                [&](const int& a, const int& b) {
                    return (averaged_node_camera_dists[a] < averaged_node_camera_dists[b]);
                }
            );
            Mat projected_edges = Mat::zeros(mask.rows, mask.cols, CV_8U);

            // project Y^{t-1} onto projected_edges
            MatrixXd Y_h = Y_.replicate(1, 1);
            Y_h.conservativeResize(Y_h.rows(), Y_h.cols() + 1);
            Y_h.col(Y_h.cols() - 1) = MatrixXd::Ones(Y_h.rows(), 1);
            MatrixXd image_coords_mask = (proj_matrix_ * Y_h.transpose()).transpose();

            std::vector<int> visible_nodes = {};
            std::vector<int> self_occluded_nodes = {};
            std::vector<int> not_self_occluded_nodes = {};
            std::vector<int> self_occluding_nodes = {};

            // draw edges closest to the camera first
            int img_rows = projected_edges.rows;
            int img_cols = projected_edges.cols;

            for (int idx : indices_vec) {
                // skip nodes behind the camera (z <= 0)
                if (image_coords_mask(idx, 2) <= 0 || image_coords_mask(idx + 1, 2) <= 0) {
                    continue;
                }

                int col_1 = static_cast<int>(image_coords_mask(idx, 0) / image_coords_mask(idx, 2));
                int row_1 = static_cast<int>(image_coords_mask(idx, 1) / image_coords_mask(idx, 2));

                int col_2 = static_cast<int>(image_coords_mask(idx + 1, 0) / image_coords_mask(idx + 1, 2));
                int row_2 = static_cast<int>(image_coords_mask(idx + 1, 1) / image_coords_mask(idx + 1, 2));

                bool pt1_in_bounds = (row_1 >= 0 && row_1 < img_rows && col_1 >= 0 && col_1 < img_cols);
                bool pt2_in_bounds = (row_2 >= 0 && row_2 < img_rows && col_2 >= 0 && col_2 < img_cols);

                // only add to visible nodes if did not overlap with existing edges
                if (pt1_in_bounds && projected_edges.at<uchar>(row_1, col_1) == 0) {
                    if (shortest_node_pt_dists[idx] <= visibility_threshold_) {
                        if (std::find(visible_nodes.begin(), visible_nodes.end(), idx) == visible_nodes.end()) {
                            visible_nodes.push_back(idx);
                        }
                    }
                    if (std::find(not_self_occluded_nodes.begin(), not_self_occluded_nodes.end(), idx) == not_self_occluded_nodes.end()) {
                        not_self_occluded_nodes.push_back(idx);
                    }
                }

                // do not consider adjacent nodes directly on top of each other
                if (pt2_in_bounds && projected_edges.at<uchar>(row_2, col_2) == 0) {
                    if (shortest_node_pt_dists[idx + 1] <= visibility_threshold_) {
                        if (std::find(visible_nodes.begin(), visible_nodes.end(), idx + 1) == visible_nodes.end()) {
                            visible_nodes.push_back(idx + 1);
                        }
                    }
                    if (std::find(not_self_occluded_nodes.begin(), not_self_occluded_nodes.end(), idx + 1) == not_self_occluded_nodes.end()) {
                        not_self_occluded_nodes.push_back(idx + 1);
                    }
                }

                // add edges for checking overlap with upcoming nodes
                // cv::line handles out-of-bounds clipping internally
                double x1 = col_1;
                double y1 = row_1;
                double x2 = col_2;
                double y2 = row_2;
                cv::line(projected_edges, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 255, 255), dlo_pixel_width_);
            }

            // sort visible nodes to preserve the original connectivity
            std::sort(visible_nodes.begin(), visible_nodes.end());

            RCLCPP_INFO_STREAM(this->get_logger(), "Visible nodes: " + std::to_string(visible_nodes.size()) + " / " + std::to_string(Y_.rows()));

            if (visible_nodes.empty()) {
                RCLCPP_WARN(this->get_logger(), "No visible nodes — skipping tracking step");
                tracking_img_pub_.publish(tracking_img_msg);
                return;
            }

            // minor mid-section occlusion is usually fine
            // extend visible nodes so that gaps as small as 2 to 3 nodes are filled
            std::vector<int> visible_nodes_extended = {};
            for (size_t i = 0; i + 1 < visible_nodes.size(); i++) {
                visible_nodes_extended.push_back(visible_nodes[i]);
                // extend visible nodes
                if (fabs(converted_node_coord_[visible_nodes[i + 1]] - converted_node_coord_[visible_nodes[i]]) <= d_vis_) {
                    for (int j = 1; j < visible_nodes[i + 1] - visible_nodes[i]; j++) {
                        visible_nodes_extended.push_back(visible_nodes[i] + j);
                    }
                }
            }
            visible_nodes_extended.push_back(visible_nodes.back());

            // store Y_0 for post processing
            MatrixXd Y_0 = Y_.replicate(1, 1);

            // step tracker
            tracker_.tracking_step(X, visible_nodes, visible_nodes_extended, proj_matrix_, mask.rows, mask.cols);
            Y_ = tracker_.get_tracking_result();
            guide_nodes = tracker_.get_guide_nodes();
            priors = tracker_.get_correspondence_pairs();

            // log time
            time_diff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - cur_time).count() / 1000.0;
            RCLCPP_INFO_STREAM(this->get_logger(), "Tracking step: " + std::to_string(time_diff) + " ms");
            algo_total_ += time_diff;
            cur_time = std::chrono::high_resolution_clock::now();

            // projection and pub image
            averaged_node_camera_dists = {};
            indices_vec = {};
            for (int i = 0; i < Y_.rows() - 1; i++) {
                averaged_node_camera_dists.push_back(((Y_.row(i) + Y_.row(i + 1)) / 2).norm());
                indices_vec.push_back(i);
            }
            // sort
            std::sort(indices_vec.begin(), indices_vec.end(),
                [&](const int& a, const int& b) {
                    return (averaged_node_camera_dists[a] < averaged_node_camera_dists[b]);
                }
            );
            std::reverse(indices_vec.begin(), indices_vec.end());

            MatrixXd nodes_h = Y_.replicate(1, 1);
            nodes_h.conservativeResize(nodes_h.rows(), nodes_h.cols() + 1);
            nodes_h.col(nodes_h.cols() - 1) = MatrixXd::Ones(nodes_h.rows(), 1);
            MatrixXd image_coords = (proj_matrix_ * nodes_h.transpose()).transpose();

            Mat tracking_img;
            tracking_img = 0.5 * cur_image_orig + 0.5 * cur_image;

            // std::vector<int> vis = visible_nodes;
            std::vector<int> vis = not_self_occluded_nodes;

            // draw points
            for (int idx : indices_vec) {

                int x = static_cast<int>(image_coords(idx, 0) / image_coords(idx, 2));
                int y = static_cast<int>(image_coords(idx, 1) / image_coords(idx, 2));

                cv::Scalar point_color;
                cv::Scalar line_color;

                if (std::find(vis.begin(), vis.end(), idx) != vis.end()) {
                    point_color = cv::Scalar(0, 150, 255);
                    line_color = cv::Scalar(0, 255, 0);
                }
                else {
                    point_color = cv::Scalar(0, 0, 255);

                    // line is colored red only when both bounding nodes are not visible
                    if (std::find(vis.begin(), vis.end(), idx + 1) == vis.end()) {
                        line_color = cv::Scalar(0, 0, 255);
                    }
                    else {
                        line_color = cv::Scalar(0, 255, 0);
                    }
                }

                cv::line(tracking_img, cv::Point(x, y),
                                       cv::Point(static_cast<int>(image_coords(idx + 1, 0) / image_coords(idx + 1, 2)),
                                                 static_cast<int>(image_coords(idx + 1, 1) / image_coords(idx + 1, 2))),
                                       line_color, 5);

                cv::circle(tracking_img, cv::Point(x, y), 7, point_color, -1);

                if (std::find(vis.begin(), vis.end(), idx + 1) != vis.end()) {
                    point_color = cv::Scalar(0, 150, 255);
                }
                else {
                    point_color = cv::Scalar(0, 0, 255);
                }
                cv::circle(tracking_img, cv::Point(static_cast<int>(image_coords(idx + 1, 0) / image_coords(idx + 1, 2)),
                                                    static_cast<int>(image_coords(idx + 1, 1) / image_coords(idx + 1, 2))),
                                                    7, point_color, -1);
            }

            // add text
            if (updated_opencv_mask_ && simulated_occlusion) {
                cv::putText(tracking_img, "occlusion", cv::Point(occlusion_corner_j, occlusion_corner_i - 10), cv::FONT_HERSHEY_DUPLEX, 1.2, cv::Scalar(0, 0, 240), 2);
            }

            // publish image
            tracking_img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", tracking_img).toImageMsg();

            // publish the results as a marker array
            visualization_msgs::msg::MarkerArray results = MatrixXd2MarkerArray(Y_, result_frame_id_, "node_results", {1.0, 150.0/255.0, 0.0, 1.0}, {0.0, 1.0, 0.0, 1.0}, 0.01, 0.005, vis, {1.0, 0.0, 0.0, 1.0}, {1.0, 0.0, 0.0, 1.0});
            // visualization_msgs::msg::MarkerArray results = MatrixXd2MarkerArray(Y_, result_frame_id_, "node_results", {1.0, 150.0/255.0, 0.0, 1.0}, {0.0, 1.0, 0.0, 1.0}, 0.01, 0.005);
            visualization_msgs::msg::MarkerArray guide_nodes_results = MatrixXd2MarkerArray(guide_nodes, result_frame_id_, "guide_node_results", {0.0, 0.0, 0.0, 0.5}, {0.0, 0.0, 1.0, 0.5});
            visualization_msgs::msg::MarkerArray corr_priors_results = MatrixXd2MarkerArray(priors, result_frame_id_, "corr_prior_results", {0.0, 0.0, 0.0, 0.5}, {1.0, 0.0, 0.0, 0.5});

            // convert to pointcloud2 for eval
            pcl::PointCloud<pcl::PointXYZ> trackdlo_pc;
            for (int i = 0; i < Y_.rows(); i++) {
                pcl::PointXYZ temp;
                temp.x = Y_(i, 0);
                temp.y = Y_(i, 1);
                temp.z = Y_(i, 2);
                trackdlo_pc.points.push_back(temp);
            }

            // get self-occluded nodes
            pcl::PointCloud<pcl::PointXYZ> self_occluded_pc;
            for (auto i : self_occluded_nodes) {
                pcl::PointXYZ temp;
                temp.x = Y_(i, 0);
                temp.y = Y_(i, 1);
                temp.z = Y_(i, 2);
                self_occluded_pc.points.push_back(temp);
            }

            // publish filtered point cloud
            pcl::PCLPointCloud2 cur_pc_pointcloud2;
            pcl::PCLPointCloud2 result_pc_poincloud2;
            pcl::PCLPointCloud2 self_occluded_pc_poincloud2;
            pcl::toPCLPointCloud2(cur_pc_downsampled, cur_pc_pointcloud2);
            pcl::toPCLPointCloud2(trackdlo_pc, result_pc_poincloud2);
            pcl::toPCLPointCloud2(self_occluded_pc, self_occluded_pc_poincloud2);

            // Convert to ROS data type
            sensor_msgs::msg::PointCloud2 cur_pc_msg;
            sensor_msgs::msg::PointCloud2 result_pc_msg;
            sensor_msgs::msg::PointCloud2 self_occluded_pc_msg;
            pcl_conversions::moveFromPCL(cur_pc_pointcloud2, cur_pc_msg);
            pcl_conversions::moveFromPCL(result_pc_poincloud2, result_pc_msg);
            pcl_conversions::moveFromPCL(self_occluded_pc_poincloud2, self_occluded_pc_msg);

            // for evaluation sync
            cur_pc_msg.header.frame_id = result_frame_id_;
            result_pc_msg.header.frame_id = result_frame_id_;
            result_pc_msg.header.stamp = image_msg->header.stamp;
            self_occluded_pc_msg.header.frame_id = result_frame_id_;
            self_occluded_pc_msg.header.stamp = image_msg->header.stamp;

            results_pub_->publish(results);
            guide_nodes_pub_->publish(guide_nodes_results);
            corr_priors_pub_->publish(corr_priors_results);
            pc_pub_->publish(cur_pc_msg);
            result_pc_pub_->publish(result_pc_msg);
            self_occluded_pc_pub_->publish(self_occluded_pc_msg);

            // reset all guide nodes
            for (size_t i = 0; i < guide_nodes_results.markers.size(); i++) {
                guide_nodes_results.markers[i].action = visualization_msgs::msg::Marker::DELETEALL;
            }
            for (size_t i = 0; i < corr_priors_results.markers.size(); i++) {
                corr_priors_results.markers[i].action = visualization_msgs::msg::Marker::DELETEALL;
            }

            // log time
            time_diff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - cur_time).count() / 1000.0;
            RCLCPP_INFO_STREAM(this->get_logger(), "Pub data: " + std::to_string(time_diff) + " ms");
            pub_data_total_ += time_diff;

            frames_ += 1;

            RCLCPP_INFO_STREAM(this->get_logger(), "Avg before tracking step: " + std::to_string(pre_proc_total_ / frames_) + " ms");
            RCLCPP_INFO_STREAM(this->get_logger(), "Avg tracking step: " + std::to_string(algo_total_ / frames_) + " ms");
            RCLCPP_INFO_STREAM(this->get_logger(), "Avg pub data: " + std::to_string(pub_data_total_ / frames_) + " ms");
            RCLCPP_INFO_STREAM(this->get_logger(), "Avg total: " + std::to_string((pre_proc_total_ + algo_total_ + pub_data_total_) / frames_) + " ms");
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
