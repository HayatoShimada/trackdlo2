#pragma once

#ifndef TRACKDLO_MOVEIT__DLO_MANIPULATION_NODE_HPP_
#define TRACKDLO_MOVEIT__DLO_MANIPULATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <Eigen/Dense>

class DloManipulationNode : public rclcpp::Node
{
public:
    DloManipulationNode();

private:
    void deferred_init();
    void results_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
    bool plan_and_execute(const geometry_msgs::msg::PoseStamped & target_pose);
    void add_collision_objects();

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr results_sub_;

    // MoveIt interfaces (initialized after construction)
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

    // Timer for deferred init
    rclcpp::TimerBase::SharedPtr init_timer_;

    // Parameters
    std::string planning_group_;
    std::string results_topic_;
    double approach_distance_;
    double grasp_offset_z_;
};

#endif  // TRACKDLO_MOVEIT__DLO_MANIPULATION_NODE_HPP_
