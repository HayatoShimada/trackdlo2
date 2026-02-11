#pragma once

#ifndef TRACKDLO_MOVEIT__DLO_MANIPULATION_NODE_HPP_
#define TRACKDLO_MOVEIT__DLO_MANIPULATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <atomic>
#include <mutex>

class DloManipulationNode : public rclcpp::Node
{
public:
    DloManipulationNode();

private:
    void deferred_init();
    void results_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
    void tracking_timer_callback();
    void add_collision_objects();
    void enable_callback(
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response);

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr results_sub_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr endpoints_pub_;

    // Services
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
    bool enabled_{true};

    // MoveIt interfaces
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>
        planning_scene_interface_;

    // Timer for deferred init
    rclcpp::TimerBase::SharedPtr init_timer_;
    // Timer for periodic tracking
    rclcpp::TimerBase::SharedPtr tracking_timer_;

    // Parameters
    std::string planning_group_;
    std::string results_topic_;
    double approach_distance_;
    double grasp_offset_z_;
    double tracking_rate_;
    double position_tolerance_;

    // Traversal state machine
    enum class TraversalState { GOTO_A, GOTO_B };
    TraversalState traversal_state_{TraversalState::GOTO_A};
    int consecutive_failures_{0};
    int max_consecutive_failures_;

    // State
    std::mutex endpoint_mutex_;
    Eigen::Vector3d endpoint_a_;
    Eigen::Vector3d endpoint_b_;
    std::string latest_frame_id_;
    bool both_endpoints_valid_{false};
    std::atomic<bool> executing_{false};
};

#endif  // TRACKDLO_MOVEIT__DLO_MANIPULATION_NODE_HPP_
