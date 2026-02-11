#include "trackdlo_moveit/dlo_manipulation_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

DloManipulationNode::DloManipulationNode()
    : Node("dlo_manipulation"),
      last_detection_time_(0, 0, RCL_ROS_TIME)
{
    this->declare_parameter<std::string>("planning_group", "ur_manipulator");
    this->declare_parameter<std::string>("results_topic", "/trackdlo/results_pc");
    this->declare_parameter<double>("approach_distance", 0.1);
    this->declare_parameter<double>("grasp_offset_z", 0.05);
    this->declare_parameter<double>("tracking_rate", 2.0);
    this->declare_parameter<double>("position_tolerance", 0.02);
    this->declare_parameter<int>("max_consecutive_failures", 3);
    this->declare_parameter<double>("detection_timeout", 5.0);
    this->declare_parameter<double>("startup_delay", 60.0);
    this->declare_parameter<double>("tracking_velocity_scale", 0.05);

    planning_group_ = this->get_parameter("planning_group").as_string();
    results_topic_ = this->get_parameter("results_topic").as_string();
    approach_distance_ = this->get_parameter("approach_distance").as_double();
    grasp_offset_z_ = this->get_parameter("grasp_offset_z").as_double();
    tracking_rate_ = this->get_parameter("tracking_rate").as_double();
    position_tolerance_ = this->get_parameter("position_tolerance").as_double();
    max_consecutive_failures_ = this->get_parameter("max_consecutive_failures").as_int();
    detection_timeout_ = this->get_parameter("detection_timeout").as_double();
    startup_delay_ = this->get_parameter("startup_delay").as_double();
    tracking_velocity_scale_ = this->get_parameter("tracking_velocity_scale").as_double();

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

    move_group_->setPlanningPipelineId("ompl");
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(20);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);
    move_group_->setGoalPositionTolerance(0.02);
    move_group_->setWorkspace(-1.0, -1.0, 0.0, 1.0, 1.0, 2.0);

    last_detection_time_ = this->now();

    add_collision_objects();

    // Start subscriber immediately so DLO can be detected during startup delay
    results_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        results_topic_, 10,
        std::bind(&DloManipulationNode::results_callback, this, std::placeholders::_1));

    endpoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/trackdlo/endpoints", 10);

    enable_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "~/enable",
        std::bind(&DloManipulationNode::enable_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
        "DLO following initialized: group=%s, rate=%.1fHz, tolerance=%.3fm, "
        "detection_timeout=%.1fs, startup_delay=%.1fs",
        planning_group_.c_str(), tracking_rate_, position_tolerance_,
        detection_timeout_, startup_delay_);

    // Defer tracking timer start to let TrackDLO / RViz initialize
    RCLCPP_INFO(this->get_logger(),
        "Waiting %.1fs for perception pipeline to initialize...", startup_delay_);

    tracking_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(startup_delay_ * 1000.0)),
        [this]() {
            // One-shot: cancel this startup timer, then replace with real timer
            tracking_timer_->cancel();

            // Check if DLO was already detected during the wait
            {
                std::lock_guard<std::mutex> lock(endpoint_mutex_);
                if (dlo_nodes_valid_) {
                    RCLCPP_INFO(this->get_logger(),
                        "DLO already detected during startup delay! "
                        "Starting in FORWARD mode.");
                    traversal_state_ = TraversalState::FORWARD;
                }
            }

            double period_ms = 1000.0 / tracking_rate_;
            tracking_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(period_ms)),
                std::bind(&DloManipulationNode::tracking_timer_callback, this));

            RCLCPP_INFO(this->get_logger(),
                "Startup delay complete. Tracking timer started (%.1fHz).",
                tracking_rate_);
        });
}

void DloManipulationNode::results_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(*msg, pcl_cloud);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(pcl_cloud, cloud);

    if (cloud.size() < 2) return;

    std::string planning_frame = move_group_->getPlanningFrame();
    std::vector<Eigen::Vector3d> nodes;
    nodes.reserve(cloud.size());

    try {
        for (auto & pt : cloud.points) {
            geometry_msgs::msg::PointStamped pt_cam, pt_world;
            pt_cam.header = msg->header;
            pt_cam.point.x = pt.x;
            pt_cam.point.y = pt.y;
            pt_cam.point.z = pt.z;
            pt_world = tf_buffer_->transform(
                pt_cam, planning_frame, tf2::durationFromSec(0.1));
            nodes.emplace_back(
                pt_world.point.x, pt_world.point.y, pt_world.point.z);
        }
    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "TF transform failed: %s", ex.what());
        return;
    }

    // Compute DLO center for dynamic search waypoints
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (auto & n : nodes) center += n;
    center /= static_cast<double>(nodes.size());

    // Build endpoint message before moving nodes
    Eigen::Vector3d front = nodes.front();
    Eigen::Vector3d back = nodes.back();

    {
        std::lock_guard<std::mutex> lock(endpoint_mutex_);
        dlo_nodes_ = std::move(nodes);
        latest_frame_id_ = planning_frame;
        dlo_nodes_valid_ = true;
        last_detection_time_ = this->now();
        last_known_dlo_center_ = center;
    }

    // Publish endpoints (first & last nodes)
    geometry_msgs::msg::PoseArray endpoints_msg;
    endpoints_msg.header.stamp = this->now();
    endpoints_msg.header.frame_id = planning_frame;

    geometry_msgs::msg::Pose pose_a;
    pose_a.position.x = front.x();
    pose_a.position.y = front.y();
    pose_a.position.z = front.z();
    pose_a.orientation.w = 1.0;
    endpoints_msg.poses.push_back(pose_a);

    geometry_msgs::msg::Pose pose_b;
    pose_b.position.x = back.x();
    pose_b.position.y = back.y();
    pose_b.position.z = back.z();
    pose_b.orientation.w = 1.0;
    endpoints_msg.poses.push_back(pose_b);

    endpoints_pub_->publish(endpoints_msg);
}

void DloManipulationNode::tracking_timer_callback()
{
    if (!enabled_ || executing_) return;

    // DLO lost detection: switch back to SEARCHING after timeout
    if (traversal_state_ != TraversalState::SEARCHING && dlo_nodes_valid_) {
        double elapsed = (this->now() - last_detection_time_).seconds();
        if (elapsed > detection_timeout_) {
            RCLCPP_WARN(this->get_logger(),
                "DLO lost for %.1fs, switching to SEARCHING", elapsed);
            traversal_state_ = TraversalState::SEARCHING;
            dlo_nodes_valid_ = false;
        }
    }

    executing_ = true;

    if (traversal_state_ == TraversalState::SEARCHING) {
        search_for_dlo();
    } else {
        track_along_dlo();
    }

    executing_ = false;
}

void DloManipulationNode::search_for_dlo()
{
    Eigen::Vector3d center;
    // Check if DLO was detected while we waited
    {
        std::lock_guard<std::mutex> lock(endpoint_mutex_);
        if (dlo_nodes_valid_) {
            RCLCPP_INFO(this->get_logger(),
                "DLO detected! Switching to FORWARD");
            traversal_state_ = TraversalState::FORWARD;
            return;
        }
        center = last_known_dlo_center_;
    }

    // Generate search waypoints dynamically around last known DLO center
    // Small patrol pattern (±0.1m) above the last known position
    double z = center.z() + approach_distance_;
    double r = 0.10;  // search radius
    search_waypoints_ = {
        Eigen::Vector3d(center.x(),     center.y(),     z),           // center
        Eigen::Vector3d(center.x() - r, center.y() - r, z),          // front-left
        Eigen::Vector3d(center.x() + r, center.y() - r, z),          // back-left
        Eigen::Vector3d(center.x() + r, center.y() + r, z),          // back-right
        Eigen::Vector3d(center.x() - r, center.y() + r, z),          // front-right
    };

    auto & wp = search_waypoints_[search_index_ % search_waypoints_.size()];

    move_group_->setMaxVelocityScalingFactor(0.1);
    move_group_->setMaxAccelerationScalingFactor(0.1);
    move_group_->setPositionTarget(wp.x(), wp.y(), wp.z());

    auto plan_result = moveit::planning_interface::MoveGroupInterface::Plan{};
    if (move_group_->plan(plan_result) ==
        moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(),
            "Searching for DLO near [%.3f, %.3f, %.3f]: waypoint %zu",
            center.x(), center.y(), center.z(), search_index_);
        move_group_->execute(plan_result);
    }

    search_index_++;

    // Re-check after moving
    std::lock_guard<std::mutex> lock(endpoint_mutex_);
    if (dlo_nodes_valid_) {
        RCLCPP_INFO(this->get_logger(),
            "DLO detected! Switching to FORWARD");
        traversal_state_ = TraversalState::FORWARD;
    }
}

void DloManipulationNode::track_along_dlo()
{
    std::vector<Eigen::Vector3d> nodes;
    {
        std::lock_guard<std::mutex> lock(endpoint_mutex_);
        if (!dlo_nodes_valid_) return;
        nodes = dlo_nodes_;
    }

    // Generate waypoints: above each node by approach_distance_
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(nodes.size());
    for (auto & n : nodes) {
        geometry_msgs::msg::Pose p;
        p.position.x = n.x();
        p.position.y = n.y();
        p.position.z = n.z() + approach_distance_;
        // Orientation: pointing downward (180 deg rotation around X)
        p.orientation.x = 1.0;
        p.orientation.y = 0.0;
        p.orientation.z = 0.0;
        p.orientation.w = 0.0;
        waypoints.push_back(p);
    }

    if (traversal_state_ == TraversalState::BACKWARD) {
        std::reverse(waypoints.begin(), waypoints.end());
    }

    const char * direction =
        (traversal_state_ == TraversalState::FORWARD) ? "FWD" : "BWD";

    // Set slow velocity for Cartesian path computation
    move_group_->setMaxVelocityScalingFactor(tracking_velocity_scale_);
    move_group_->setMaxAccelerationScalingFactor(tracking_velocity_scale_);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_->computeCartesianPath(
        waypoints, 0.01, 0.0, trajectory, true);

    RCLCPP_INFO(this->get_logger(),
        "Cartesian path [%s]: %.1f%% achieved (%zu waypoints)",
        direction, fraction * 100.0, waypoints.size());

    if (fraction >= 0.5) {
        // Slow down trajectory to avoid losing DLO from camera view
        scale_trajectory_speed(trajectory, tracking_velocity_scale_);
        move_group_->execute(trajectory);
        // Reverse direction
        traversal_state_ = (traversal_state_ == TraversalState::FORWARD)
            ? TraversalState::BACKWARD : TraversalState::FORWARD;
        consecutive_failures_ = 0;
    } else {
        consecutive_failures_++;
        RCLCPP_WARN(this->get_logger(),
            "Cartesian path [%s] insufficient (%.1f%%), failure %d/%d",
            direction, fraction * 100.0,
            consecutive_failures_, max_consecutive_failures_);
        if (consecutive_failures_ >= max_consecutive_failures_) {
            traversal_state_ = (traversal_state_ == TraversalState::FORWARD)
                ? TraversalState::BACKWARD : TraversalState::FORWARD;
            consecutive_failures_ = 0;
        }
    }
}

void DloManipulationNode::scale_trajectory_speed(
    moveit_msgs::msg::RobotTrajectory & trajectory, double scale)
{
    for (auto & point : trajectory.joint_trajectory.points) {
        // Stretch time_from_start by 1/scale
        double t = point.time_from_start.sec +
                   point.time_from_start.nanosec * 1e-9;
        t /= scale;
        point.time_from_start.sec = static_cast<int32_t>(t);
        point.time_from_start.nanosec =
            static_cast<uint32_t>((t - point.time_from_start.sec) * 1e9);

        // Scale velocities and accelerations
        for (auto & v : point.velocities) v *= scale;
        for (auto & a : point.accelerations) a *= scale * scale;
    }
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
