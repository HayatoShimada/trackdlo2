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
#include <moveit_msgs/msg/robot_trajectory.hpp>
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
    void search_for_dlo();
    void track_along_dlo();
    void scale_trajectory_speed(moveit_msgs::msg::RobotTrajectory & trajectory,
                                double scale);
    void move_to_home_position();
    void add_collision_objects();
    void enable_callback(
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response);

    // ノード列からウェイポイント（approach_distance_ 上方、下向き姿勢）を生成
    std::vector<geometry_msgs::msg::Pose> generate_waypoints(
        const std::vector<Eigen::Vector3d> & nodes) const;

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
    double tracking_rate_;
    double position_tolerance_;
    double detection_timeout_;
    double startup_delay_;
    double tracking_velocity_scale_;

    // Table geometry parameters
    double table_height_;
    double table_center_x_;
    double table_center_y_;
    double table_size_x_;
    double table_size_y_;

    // Velocity/pause parameters
    double search_pause_duration_;
    double tracking_pause_duration_;
    double search_velocity_scale_;
    double approach_velocity_scale_;

    // Traversal state machine
    enum class TraversalState { SEARCHING, FORWARD, BACKWARD };
    TraversalState traversal_state_{TraversalState::SEARCHING};
    int consecutive_failures_{0};
    int max_consecutive_failures_;
    int max_search_iterations_;

    // DLO nodes (all points in planning frame)
    std::mutex endpoint_mutex_;
    std::vector<Eigen::Vector3d> dlo_nodes_;
    std::string latest_frame_id_;
    bool dlo_nodes_valid_{false};
    rclcpp::Time last_detection_time_;

    // Search: dynamic waypoints around last known DLO center
    Eigen::Vector3d last_known_dlo_center_{0.4, 0.0, 0.85};
    std::vector<Eigen::Vector3d> search_waypoints_;
    size_t search_index_{0};

    std::atomic<bool> executing_{false};
};

#endif  // TRACKDLO_MOVEIT__DLO_MANIPULATION_NODE_HPP_
