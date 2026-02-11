#include "trackdlo_moveit/dlo_manipulation_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

DloManipulationNode::DloManipulationNode()
    : Node("dlo_manipulation")
{
    this->declare_parameter<std::string>("planning_group", "ur_manipulator");
    this->declare_parameter<std::string>("results_topic", "/trackdlo/results_pc");
    this->declare_parameter<double>("approach_distance", 0.1);
    this->declare_parameter<double>("grasp_offset_z", 0.05);
    this->declare_parameter<double>("tracking_rate", 2.0);
    this->declare_parameter<double>("position_tolerance", 0.02);
    this->declare_parameter<int>("max_consecutive_failures", 3);

    planning_group_ = this->get_parameter("planning_group").as_string();
    results_topic_ = this->get_parameter("results_topic").as_string();
    approach_distance_ = this->get_parameter("approach_distance").as_double();
    grasp_offset_z_ = this->get_parameter("grasp_offset_z").as_double();
    tracking_rate_ = this->get_parameter("tracking_rate").as_double();
    position_tolerance_ = this->get_parameter("position_tolerance").as_double();
    max_consecutive_failures_ = this->get_parameter("max_consecutive_failures").as_int();

    // TF2 setup
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Deferred init for MoveGroupInterface (needs shared_from_this())
    init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(0),
        std::bind(&DloManipulationNode::deferred_init, this));
}

void DloManipulationNode::deferred_init()
{
    init_timer_->cancel();

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_);
    planning_scene_interface_ =
        std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(20);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);
    move_group_->setGoalPositionTolerance(0.02);
    move_group_->setGoalOrientationTolerance(0.15);

    add_collision_objects();

    results_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        results_topic_, 10,
        std::bind(&DloManipulationNode::results_callback, this, std::placeholders::_1));

    endpoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/trackdlo/endpoints", 10);

    enable_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "~/enable",
        std::bind(&DloManipulationNode::enable_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    double period_ms = 1000.0 / tracking_rate_;
    tracking_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(period_ms)),
        std::bind(&DloManipulationNode::tracking_timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
        "DLO following initialized: group=%s, rate=%.1fHz, tolerance=%.3fm",
        planning_group_.c_str(), tracking_rate_, position_tolerance_);
}

void DloManipulationNode::results_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(*msg, pcl_cloud);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(pcl_cloud, cloud);

    if (cloud.size() < 2) return;

    // Get both DLO endpoints
    auto & first_pt = cloud.points[0];
    auto & last_pt = cloud.points[cloud.size() - 1];

    Eigen::Vector3d ep0(first_pt.x, first_pt.y, first_pt.z);
    Eigen::Vector3d ep1(last_pt.x, last_pt.y, last_pt.z);

    // Transform both endpoints to planning frame
    try {
        geometry_msgs::msg::PointStamped pt0_cam, pt1_cam, pt0_world, pt1_world;
        pt0_cam.header = msg->header;
        pt0_cam.point.x = ep0.x(); pt0_cam.point.y = ep0.y(); pt0_cam.point.z = ep0.z();
        pt1_cam.header = msg->header;
        pt1_cam.point.x = ep1.x(); pt1_cam.point.y = ep1.y(); pt1_cam.point.z = ep1.z();

        std::string planning_frame = move_group_->getPlanningFrame();
        pt0_world = tf_buffer_->transform(pt0_cam, planning_frame, tf2::durationFromSec(0.1));
        pt1_world = tf_buffer_->transform(pt1_cam, planning_frame, tf2::durationFromSec(0.1));

        std::lock_guard<std::mutex> lock(endpoint_mutex_);
        endpoint_a_ = Eigen::Vector3d(
            pt0_world.point.x, pt0_world.point.y, pt0_world.point.z);
        endpoint_b_ = Eigen::Vector3d(
            pt1_world.point.x, pt1_world.point.y, pt1_world.point.z);
        latest_frame_id_ = planning_frame;
        both_endpoints_valid_ = true;

        // Publish transformed endpoints for other nodes
        geometry_msgs::msg::PoseArray endpoints_msg;
        endpoints_msg.header.stamp = this->now();
        endpoints_msg.header.frame_id = planning_frame;

        geometry_msgs::msg::Pose pose_a;
        pose_a.position.x = pt0_world.point.x;
        pose_a.position.y = pt0_world.point.y;
        pose_a.position.z = pt0_world.point.z;
        pose_a.orientation.w = 1.0;
        endpoints_msg.poses.push_back(pose_a);

        geometry_msgs::msg::Pose pose_b;
        pose_b.position.x = pt1_world.point.x;
        pose_b.position.y = pt1_world.point.y;
        pose_b.position.z = pt1_world.point.z;
        pose_b.orientation.w = 1.0;
        endpoints_msg.poses.push_back(pose_b);

        endpoints_pub_->publish(endpoints_msg);

    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "TF transform failed: %s", ex.what());
    }
}

void DloManipulationNode::tracking_timer_callback()
{
    if (!enabled_ || !both_endpoints_valid_ || executing_) return;

    Eigen::Vector3d target;
    std::string frame_id;
    const char* target_name;
    {
        std::lock_guard<std::mutex> lock(endpoint_mutex_);
        if (traversal_state_ == TraversalState::GOTO_A) {
            target = endpoint_a_;
            target_name = "A";
        } else {
            target = endpoint_b_;
            target_name = "B";
        }
        frame_id = latest_frame_id_;
    }

    // Position above endpoint — orientation free (camera on arm looks naturally)
    double goal_x = target.x();
    double goal_y = target.y();
    double goal_z = target.z() + approach_distance_;

    RCLCPP_INFO(this->get_logger(),
        "Moving toward endpoint %s: [%.3f, %.3f, %.3f]",
        target_name, goal_x, goal_y, goal_z);

    executing_ = true;
    move_group_->setPositionTarget(goal_x, goal_y, goal_z);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
        RCLCPP_INFO(this->get_logger(), "Planning succeeded, executing...");
        move_group_->execute(plan);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    }
    if (success) {
        consecutive_failures_ = 0;
        // Execution completed — switch to other endpoint
        if (traversal_state_ == TraversalState::GOTO_A) {
            traversal_state_ = TraversalState::GOTO_B;
            RCLCPP_INFO(this->get_logger(), "Reached endpoint A, heading to B");
        } else {
            traversal_state_ = TraversalState::GOTO_A;
            RCLCPP_INFO(this->get_logger(), "Reached endpoint B, heading to A");
        }
    } else {
        consecutive_failures_++;
        RCLCPP_WARN(this->get_logger(),
            "Failed to plan to endpoint %s (%d/%d)",
            target_name, consecutive_failures_, max_consecutive_failures_);
        if (consecutive_failures_ >= max_consecutive_failures_) {
            if (traversal_state_ == TraversalState::GOTO_A) {
                traversal_state_ = TraversalState::GOTO_B;
                RCLCPP_INFO(this->get_logger(),
                    "Switching to endpoint B after %d failures", consecutive_failures_);
            } else {
                traversal_state_ = TraversalState::GOTO_A;
                RCLCPP_INFO(this->get_logger(),
                    "Switching to endpoint A after %d failures", consecutive_failures_);
            }
            consecutive_failures_ = 0;
        }
    }
    executing_ = false;
}

void DloManipulationNode::enable_callback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
{
    enabled_ = request->data;
    response->success = true;
    response->message = enabled_ ? "Autonomous tracking enabled" : "Autonomous tracking disabled";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

void DloManipulationNode::add_collision_objects()
{
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    // Table surface (thin box to avoid colliding with robot links below table)
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = move_group_->getPlanningFrame();
    table.id = "table";

    shape_msgs::msg::SolidPrimitive table_shape;
    table_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    table_shape.dimensions = {1.2, 0.8, 0.02};

    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = 0.5;
    table_pose.position.y = 0.0;
    table_pose.position.z = 0.74;
    table_pose.orientation.w = 1.0;

    table.primitives.push_back(table_shape);
    table.primitive_poses.push_back(table_pose);
    table.operation = moveit_msgs::msg::CollisionObject::ADD;
    collision_objects.push_back(table);

    planning_scene_interface_->addCollisionObjects(collision_objects);
    RCLCPP_INFO(this->get_logger(), "Added %zu collision objects",
                collision_objects.size());
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DloManipulationNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
