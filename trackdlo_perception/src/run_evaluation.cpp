#include "trackdlo_perception/utils.hpp"
#include "trackdlo_perception/trackdlo.hpp"
#include "trackdlo_perception/evaluator.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <fstream>
#include <stdexcept>

using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using cv::Mat;

using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::PointCloud2>;

class EvaluationNode : public rclcpp::Node
{
public:
    EvaluationNode() : Node("evaluation"),
        callback_count_(0),
        head_node_(MatrixXd::Zero(1, 3)),
        proj_matrix_(3, 4),
        initialized_(false)
    {
        // Declare parameters
        this->declare_parameter<int>("bag_file", 0);
        this->declare_parameter<int>("trial", 1);
        this->declare_parameter<std::string>("alg", "trackdlo");
        this->declare_parameter<std::string>("bag_dir", "");
        this->declare_parameter<std::string>("save_location", "");
        this->declare_parameter<int>("pct_occlusion", 0);
        this->declare_parameter<double>("start_record_at", 0.0);
        this->declare_parameter<double>("exit_at", -1.0);
        this->declare_parameter<double>("wait_before_occlusion", 0.0);
        this->declare_parameter<double>("bag_rate", 1.0);
        this->declare_parameter<int>("num_of_nodes", 30);
        this->declare_parameter<bool>("save_images", false);
        this->declare_parameter<bool>("save_errors", true);

        bag_file_ = this->get_parameter("bag_file").as_int();
        trial_ = this->get_parameter("trial").as_int();
        alg_ = this->get_parameter("alg").as_string();
        bag_dir_ = this->get_parameter("bag_dir").as_string();
        save_location_ = this->get_parameter("save_location").as_string();
        pct_occlusion_ = this->get_parameter("pct_occlusion").as_int();
        start_record_at_ = this->get_parameter("start_record_at").as_double();
        exit_at_ = this->get_parameter("exit_at").as_double();
        wait_before_occlusion_ = this->get_parameter("wait_before_occlusion").as_double();
        bag_rate_ = this->get_parameter("bag_rate").as_double();
        num_of_nodes_ = this->get_parameter("num_of_nodes").as_int();
        save_images_ = this->get_parameter("save_images").as_bool();
        save_errors_ = this->get_parameter("save_errors").as_bool();

        // Set projection matrix
        proj_matrix_ << 918.359130859375, 0.0, 645.8908081054688, 0.0,
                        0.0, 916.265869140625, 354.02392578125, 0.0,
                        0.0, 0.0, 1.0, 0.0;

        // Count messages in bag file using rosbag2 metadata
        int rgb_count = 0;
        int depth_count = 0;
        int pc_count = 0;

        rosbag2_cpp::Reader reader;
        reader.open(bag_dir_);

        auto metadata = reader.get_metadata();
        for (const auto & topic_info : metadata.topics_with_message_count) {
            if (topic_info.topic_metadata.name == "/camera/color/image_raw") {
                rgb_count = static_cast<int>(topic_info.message_count);
            }
            else if (topic_info.topic_metadata.name == "/camera/aligned_depth_to_color/image_raw") {
                depth_count = static_cast<int>(topic_info.message_count);
            }
            else if (topic_info.topic_metadata.name == "/camera/depth/color/points") {
                pc_count = static_cast<int>(topic_info.message_count);
            }
        }

        RCLCPP_INFO_STREAM(this->get_logger(), "num of rgb images: " << rgb_count);
        RCLCPP_INFO_STREAM(this->get_logger(), "num of depth images: " << depth_count);
        RCLCPP_INFO_STREAM(this->get_logger(), "num of point cloud messages: " << pc_count);

        // Initialize evaluator
        tracking_evaluator_ = evaluator(rgb_count, trial_, pct_occlusion_, alg_, bag_file_,
                                        save_location_, start_record_at_, exit_at_,
                                        wait_before_occlusion_, bag_rate_, num_of_nodes_);

        // Use one-shot timer for post-constructor initialization (shared_from_this())
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(0),
            std::bind(&EvaluationNode::deferred_init, this));
    }

private:
    void deferred_init()
    {
        init_timer_->cancel();

        // Image transport publisher
        image_transport::ImageTransport it(shared_from_this());
        eval_img_pub_ = it.advertise("/eval_img", 10);

        // Occlusion corners publisher
        corners_arr_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/corners", 10);

        // Message filter subscribers
        image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            shared_from_this(), "/camera/color/image_raw", rmw_qos_profile_default);
        pc_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
            shared_from_this(), "/camera/depth/color/points", rmw_qos_profile_default);
        result_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
            shared_from_this(), "/" + alg_ + "/results_pc", rmw_qos_profile_default);

        sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
            ApproxSyncPolicy(10), *image_sub_, *pc_sub_, *result_sub_);

        sync_->registerCallback(std::bind(&EvaluationNode::callback, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        RCLCPP_INFO(this->get_logger(), "Evaluation node initialized.");
    }

    void callback(
        const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & result_msg)
    {
        if (!initialized_) {
            tracking_evaluator_.set_start_time(std::chrono::steady_clock::now());
            initialized_ = true;
        }

        double time_from_start;
        time_from_start = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - tracking_evaluator_.start_time()).count();
        time_from_start = (time_from_start / 1000.0 + 3.0) * tracking_evaluator_.rate();

        RCLCPP_INFO_STREAM(this->get_logger(), time_from_start << "; " << tracking_evaluator_.exit_time());

        if (tracking_evaluator_.exit_time() == -1) {
            if (callback_count_ >= tracking_evaluator_.length() - 3) {
                RCLCPP_INFO(this->get_logger(), "Shutting down evaluator...");
                rclcpp::shutdown();
                return;
            }
        }
        else {
            if (time_from_start > tracking_evaluator_.exit_time() ||
                callback_count_ >= tracking_evaluator_.length() - 3)
            {
                RCLCPP_INFO(this->get_logger(), "Shutting down evaluator...");
                rclcpp::shutdown();
                return;
            }
        }

        callback_count_ += 1;
        RCLCPP_INFO_STREAM(this->get_logger(), "callback: " << callback_count_);

        Mat cur_image_orig = cv_bridge::toCvShare(image_msg, "bgr8")->image;

        // For visualizing results
        Mat eval_img;
        cur_image_orig.copyTo(eval_img);

        // Process original point cloud
        pcl::PCLPointCloud2 cloud;
        pcl_conversions::toPCL(*pc_msg, cloud);
        pcl::PointCloud<pcl::PointXYZRGB> cloud_xyz;
        pcl::fromPCLPointCloud2(cloud, cloud_xyz);

        // Process result point cloud
        pcl::PCLPointCloud2 result_cloud;
        pcl_conversions::toPCL(*result_msg, result_cloud);
        pcl::PointCloud<pcl::PointXYZ> result_cloud_xyz;
        pcl::fromPCLPointCloud2(result_cloud, result_cloud_xyz);
        MatrixXd Y_track = result_cloud_xyz.getMatrixXfMap().topRows(3).transpose().cast<double>();

        int top_left_x = -1;
        int top_left_y = -1;
        int bottom_right_x = -1;
        int bottom_right_y = -1;

        double cur_error = -1;
        if (time_from_start > tracking_evaluator_.recording_start_time()) {

            if (bag_file_ != 3) {
                MatrixXd gt_nodes = tracking_evaluator_.get_ground_truth_nodes(cur_image_orig, cloud_xyz);
                MatrixXd Y_true = gt_nodes.replicate(1, 1);

                // If head_node not initialized
                if (head_node_(0, 0) == 0.0 && head_node_(0, 1) == 0.0 && head_node_(0, 2) == 0.0) {
                    if (Y_track(0, 0) > Y_track(Y_track.rows()-1, 0)) {
                        head_node_ = Y_track.row(Y_track.rows()-1).replicate(1, 1);
                    }
                    else {
                        head_node_ = Y_track.row(0).replicate(1, 1);
                    }
                }
                Y_true = tracking_evaluator_.sort_pts(gt_nodes, head_node_);

                // Update head node
                head_node_ = Y_true.row(0).replicate(1, 1);
                RCLCPP_INFO_STREAM(this->get_logger(),
                    "Y_true size: " << Y_true.rows() << "; Y_track size: " << Y_track.rows());

                if (time_from_start > tracking_evaluator_.recording_start_time() + tracking_evaluator_.wait_before_occlusion()) {
                    if (bag_file_ == 0) {
                        int num_of_occluded_nodes = static_cast<int>(Y_track.rows() * tracking_evaluator_.pct_occlusion() / 100.0);

                        if (num_of_occluded_nodes != 0) {
                            double min_x = Y_true(0, 0);
                            double min_y = Y_true(0, 1);
                            double min_z = Y_true(0, 2);

                            double max_x = Y_true(0, 0);
                            double max_y = Y_true(0, 1);
                            double max_z = Y_true(0, 2);

                            for (int i = 0; i < num_of_occluded_nodes; i ++) {
                                if (Y_true(i, 0) < min_x) { min_x = Y_true(i, 0); }
                                if (Y_true(i, 1) < min_y) { min_y = Y_true(i, 1); }
                                if (Y_true(i, 2) < min_z) { min_z = Y_true(i, 2); }
                                if (Y_true(i, 0) > max_x) { max_x = Y_true(i, 0); }
                                if (Y_true(i, 1) > max_y) { max_y = Y_true(i, 1); }
                                if (Y_true(i, 2) > max_z) { max_z = Y_true(i, 2); }
                            }

                            MatrixXd min_corner(1, 3);
                            min_corner << min_x, min_y, min_z;
                            MatrixXd max_corner(1, 3);
                            max_corner << max_x, max_y, max_z;

                            MatrixXd corners = MatrixXd::Zero(2, 3);
                            corners.row(0) = min_corner.replicate(1, 1);
                            corners.row(1) = max_corner.replicate(1, 1);

                            // Project to find occlusion block coordinate
                            MatrixXd nodes_h = corners.replicate(1, 1);
                            nodes_h.conservativeResize(nodes_h.rows(), nodes_h.cols()+1);
                            nodes_h.col(nodes_h.cols()-1) = MatrixXd::Ones(nodes_h.rows(), 1);
                            MatrixXd image_coords = (proj_matrix_ * nodes_h.transpose()).transpose();

                            int pix_coord_1_x = static_cast<int>(image_coords(0, 0)/image_coords(0, 2));
                            int pix_coord_1_y = static_cast<int>(image_coords(0, 1)/image_coords(0, 2));
                            int pix_coord_2_x = static_cast<int>(image_coords(1, 0)/image_coords(1, 2));
                            int pix_coord_2_y = static_cast<int>(image_coords(1, 1)/image_coords(1, 2));

                            int extra_border = 30;

                            if (pix_coord_1_x <= pix_coord_2_x && pix_coord_1_y <= pix_coord_2_y) {
                                top_left_x = pix_coord_1_x - extra_border;
                                if (top_left_x < 0) {top_left_x = 0;}
                                top_left_y = pix_coord_1_y - extra_border;
                                if (top_left_y < 0) {top_left_y = 0;}
                                bottom_right_x = pix_coord_2_x + extra_border;
                                if (bottom_right_x >= cur_image_orig.cols) {bottom_right_x = cur_image_orig.cols-1;}
                                bottom_right_y = pix_coord_2_y + extra_border;
                                if (bottom_right_y >= cur_image_orig.rows) {bottom_right_y = cur_image_orig.rows-1;}
                            }
                            else if (pix_coord_2_x <= pix_coord_1_x && pix_coord_2_y <= pix_coord_1_y) {
                                top_left_x = pix_coord_2_x - extra_border;
                                if (top_left_x < 0) {top_left_x = 0;}
                                top_left_y = pix_coord_2_y - extra_border;
                                if (top_left_y < 0) {top_left_y = 0;}
                                bottom_right_x = pix_coord_1_x + extra_border;
                                if (bottom_right_x >= cur_image_orig.cols) {bottom_right_x = cur_image_orig.cols-1;}
                                bottom_right_y = pix_coord_1_y + extra_border;
                                if (bottom_right_y >= cur_image_orig.rows) {bottom_right_y = cur_image_orig.rows-1;}
                            }
                            else if (pix_coord_2_x <= pix_coord_1_x && pix_coord_1_y <= pix_coord_2_y) {
                                top_left_x = pix_coord_2_x - extra_border;
                                if (top_left_x < 0) {top_left_x = 0;}
                                top_left_y = pix_coord_1_y - extra_border;
                                if (top_left_y < 0) {top_left_y = 0;}
                                bottom_right_x = pix_coord_1_x + extra_border;
                                if (bottom_right_x >= cur_image_orig.cols) {bottom_right_x = cur_image_orig.cols-1;}
                                bottom_right_y = pix_coord_2_y + extra_border;
                                if (bottom_right_y >= cur_image_orig.rows) {bottom_right_y = cur_image_orig.rows-1;}
                            }
                            else {
                                top_left_x = pix_coord_1_x - extra_border;
                                if (top_left_x < 0) {top_left_x = 0;}
                                top_left_y = pix_coord_2_y - extra_border;
                                if (top_left_y < 0) {top_left_y = 0;}
                                bottom_right_x = pix_coord_2_x + extra_border;
                                if (bottom_right_x >= cur_image_orig.cols) {bottom_right_x = cur_image_orig.cols-1;}
                                bottom_right_y = pix_coord_1_y + extra_border;
                                if (bottom_right_y >= cur_image_orig.rows) {bottom_right_y = cur_image_orig.rows-1;}
                            }

                            auto corners_arr = std_msgs::msg::Int32MultiArray();
                            corners_arr.data = {top_left_x, top_left_y, bottom_right_x, bottom_right_y};
                            corners_arr_pub_->publish(corners_arr);
                        }
                    }

                    else if (bag_file_ == 1) {
                        top_left_x = 840;
                        top_left_y = 408;
                        bottom_right_x = 1191;
                        bottom_right_y = 678;

                        auto corners_arr = std_msgs::msg::Int32MultiArray();
                        corners_arr.data = {top_left_x, top_left_y, bottom_right_x, bottom_right_y};
                        corners_arr_pub_->publish(corners_arr);
                    }

                    else if (bag_file_ == 2) {
                        top_left_x = 780;
                        top_left_y = 120;
                        bottom_right_x = 1050;
                        bottom_right_y = 290;

                        auto corners_arr = std_msgs::msg::Int32MultiArray();
                        corners_arr.data = {top_left_x, top_left_y, bottom_right_x, bottom_right_y};
                        corners_arr_pub_->publish(corners_arr);
                    }

                    else if (bag_file_ == 4) {
                        top_left_x = 543;
                        top_left_y = 276;
                        bottom_right_x = 738;
                        bottom_right_y = 383;

                        auto corners_arr = std_msgs::msg::Int32MultiArray();
                        corners_arr.data = {top_left_x, top_left_y, bottom_right_x, bottom_right_y};
                        corners_arr_pub_->publish(corners_arr);
                    }

                    else if (bag_file_ == 5) {
                        top_left_x = 300;
                        top_left_y = 317;
                        bottom_right_x = 698;
                        bottom_right_y = 440;

                        auto corners_arr = std_msgs::msg::Int32MultiArray();
                        corners_arr.data = {top_left_x, top_left_y, bottom_right_x, bottom_right_y};
                        corners_arr_pub_->publish(corners_arr);
                    }

                    else {
                        throw std::invalid_argument("Invalid bag file ID!");
                    }
                }

                // Compute error
                if (save_errors_) {
                    cur_error = tracking_evaluator_.compute_and_save_error(Y_track, Y_true);
                }
                else {
                    cur_error = tracking_evaluator_.compute_error(Y_track, Y_true);
                }
                RCLCPP_INFO_STREAM(this->get_logger(), "error = " << cur_error);

                // Optional: draw occlusion block and save result image
                if (time_from_start > tracking_evaluator_.recording_start_time() + tracking_evaluator_.wait_before_occlusion()) {
                    cv::Point p1(top_left_x, top_left_y);
                    cv::Point p2(bottom_right_x, bottom_right_y);
                    cv::rectangle(eval_img, p1, p2, cv::Scalar(0, 0, 0), -1);
                    eval_img = 0.5*eval_img + 0.5*cur_image_orig;

                    if (bag_file_ == 4) {
                        cv::putText(eval_img, "occlusion", cv::Point(top_left_x-190, bottom_right_y-5),
                                    cv::FONT_HERSHEY_DUPLEX, 1.2, cv::Scalar(0, 0, 240), 2);
                    }
                    else {
                        cv::putText(eval_img, "occlusion", cv::Point(top_left_x, top_left_y-10),
                                    cv::FONT_HERSHEY_DUPLEX, 1.2, cv::Scalar(0, 0, 240), 2);
                    }
                }
                else {
                    cur_error = tracking_evaluator_.compute_error(Y_track, Y_true);
                }
            }
        }

        // Project tracking results onto the image
        MatrixXd Y_track_h = Y_track.replicate(1, 1);
        Y_track_h.conservativeResize(Y_track_h.rows(), Y_track_h.cols()+1);
        Y_track_h.col(Y_track_h.cols()-1) = MatrixXd::Ones(Y_track_h.rows(), 1);

        MatrixXd image_coords_Y = (proj_matrix_ * Y_track_h.transpose()).transpose();

        for (int i = 0; i < image_coords_Y.rows(); i ++) {
            int row = static_cast<int>(image_coords_Y(i, 0)/image_coords_Y(i, 2));
            int col = static_cast<int>(image_coords_Y(i, 1)/image_coords_Y(i, 2));

            cv::Scalar point_color;
            cv::Scalar line_color;
            int line_width = 3;
            int point_radius = 7;

            if (row <= bottom_right_x && row >= top_left_x && col <= bottom_right_y && col >= top_left_y) {
                point_color = cv::Scalar(0, 0, 255);
                line_color = cv::Scalar(0, 0, 255);
            }
            else {
                point_color = cv::Scalar(0, 150, 255);
                line_color = cv::Scalar(0, 255, 0);
            }

            if (i != image_coords_Y.rows()-1) {
                cv::line(eval_img, cv::Point(row, col),
                    cv::Point(static_cast<int>(image_coords_Y(i+1, 0)/image_coords_Y(i+1, 2)),
                    static_cast<int>(image_coords_Y(i+1, 1)/image_coords_Y(i+1, 2))),
                    line_color, line_width);
            }

            cv::circle(eval_img, cv::Point(row, col), point_radius, point_color, -1);
        }

        // Save image
        if (save_images_) {
            double diff = time_from_start - tracking_evaluator_.recording_start_time();
            double time_step = 0.5;
            if (bag_file_ == 0) {
                time_step = 1.0;
            }
            if ((int)(diff/time_step) == tracking_evaluator_.image_counter() &&
                fabs(diff-(tracking_evaluator_.image_counter()*time_step)) <= 0.1)
            {
                std::string dir;
                if (bag_file_ == 0) {
                    dir = save_location_ + "images/" + alg_ + "_" + std::to_string(trial_) + "_" + std::to_string(pct_occlusion_) + "_stationary_frame_" + std::to_string(tracking_evaluator_.image_counter()) + ".png";
                }
                else if (bag_file_ == 1) {
                    dir = save_location_ + "images/" + alg_ + "_" + std::to_string(trial_) + "_" + std::to_string(pct_occlusion_) + "_perpendicular_motion_frame_" + std::to_string(tracking_evaluator_.image_counter()) + ".png";
                }
                else if (bag_file_ == 2) {
                    dir = save_location_ + "images/" + alg_ + "_" + std::to_string(trial_) + "_" + std::to_string(pct_occlusion_) + "_parallel_motion_frame_" + std::to_string(tracking_evaluator_.image_counter()) + ".png";
                }
                else if (bag_file_ == 3) {
                    dir = save_location_ + "images/" + alg_ + "_" + std::to_string(trial_) + "_" + std::to_string(pct_occlusion_) + "_self_occlusion_frame_" + std::to_string(tracking_evaluator_.image_counter()) + ".png";
                }
                else if (bag_file_ == 4) {
                    dir = save_location_ + "images/" + alg_ + "_" + std::to_string(trial_) + "_" + std::to_string(pct_occlusion_) + "_short_rope_folding_frame_" + std::to_string(tracking_evaluator_.image_counter()) + ".png";
                }
                else if (bag_file_ == 5) {
                    dir = save_location_ + "images/" + alg_ + "_" + std::to_string(trial_) + "_" + std::to_string(pct_occlusion_) + "_short_rope_stationary_frame_" + std::to_string(tracking_evaluator_.image_counter()) + ".png";
                }
                cv::imwrite(dir, eval_img);
                tracking_evaluator_.increment_image_counter();
            }
        }

        auto eval_img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", eval_img).toImageMsg();
        eval_img_pub_.publish(eval_img_msg);
    }

    // Parameters
    int bag_file_;
    int trial_;
    std::string alg_;
    std::string bag_dir_;
    std::string save_location_;
    int pct_occlusion_;
    double start_record_at_;
    double exit_at_;
    double wait_before_occlusion_;
    double bag_rate_;
    int num_of_nodes_;
    bool save_images_;
    bool save_errors_;

    // State
    int callback_count_;
    evaluator tracking_evaluator_;
    MatrixXd head_node_;
    MatrixXd proj_matrix_;
    bool initialized_;

    // Timer for deferred init
    rclcpp::TimerBase::SharedPtr init_timer_;

    // Publishers
    image_transport::Publisher eval_img_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr corners_arr_pub_;

    // Message filter subscribers and synchronizer
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pc_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> result_sub_;
    std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EvaluationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
