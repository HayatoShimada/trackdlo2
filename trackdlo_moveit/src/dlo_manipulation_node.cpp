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

    planning_group_ = this->get_parameter("planning_group").as_string();
    results_topic_ = this->get_parameter("results_topic").as_string();
    approach_distance_ = this->get_parameter("approach_distance").as_double();
    grasp_offset_z_ = this->get_parameter("grasp_offset_z").as_double();
    tracking_rate_ = this->get_parameter("tracking_rate").as_double();
    position_tolerance_ = this->get_parameter("position_tolerance").as_double();

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

    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);
    move_group_->setGoalPositionTolerance(0.01);
    move_group_->setGoalOrientationTolerance(0.05);

    add_collision_objects();

    results_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        results_topic_, 10,
        std::bind(&DloManipulationNode::results_callback, this, std::placeholders::_1));

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

    if (cloud.empty()) return;

    // Get both DLO endpoints
    auto & first_pt = cloud.points[0];
    auto & last_pt = cloud.points[cloud.size() - 1];

    // Pick endpoint closer to robot base (0, 0, 0.75 in world)
    // Points are in camera frame; transform to world to compare
    Eigen::Vector3d ep0(first_pt.x, first_pt.y, first_pt.z);
    Eigen::Vector3d ep1(last_pt.x, last_pt.y, last_pt.z);

    // Try to transform both endpoints to planning frame
    try {
        geometry_msgs::msg::PointStamped pt0_cam, pt1_cam, pt0_world, pt1_world;
        pt0_cam.header = msg->header;
        pt0_cam.point.x = ep0.x(); pt0_cam.point.y = ep0.y(); pt0_cam.point.z = ep0.z();
        pt1_cam.header = msg->header;
        pt1_cam.point.x = ep1.x(); pt1_cam.point.y = ep1.y(); pt1_cam.point.z = ep1.z();

        std::string planning_frame = move_group_->getPlanningFrame();
        pt0_world = tf_buffer_->transform(pt0_cam, planning_frame, tf2::durationFromSec(0.1));
        pt1_world = tf_buffer_->transform(pt1_cam, planning_frame, tf2::durationFromSec(0.1));

        // Robot base is at origin in planning frame; pick the closer endpoint
        double d0 = std::hypot(pt0_world.point.x, pt0_world.point.y);
        double d1 = std::hypot(pt1_world.point.x, pt1_world.point.y);

        std::lock_guard<std::mutex> lock(endpoint_mutex_);
        if (d0 <= d1) {
            latest_endpoint_ = Eigen::Vector3d(
                pt0_world.point.x, pt0_world.point.y, pt0_world.point.z);
        } else {
            latest_endpoint_ = Eigen::Vector3d(
                pt1_world.point.x, pt1_world.point.y, pt1_world.point.z);
        }
        latest_frame_id_ = planning_frame;
        endpoint_valid_ = true;

    } catch (tf2::TransformException & ex) {
        // Fallback: use first endpoint in camera frame
        std::lock_guard<std::mutex> lock(endpoint_mutex_);
        latest_endpoint_ = ep0;
        latest_frame_id_ = msg->header.frame_id;
        endpoint_valid_ = true;
    }
}

void DloManipulationNode::tracking_timer_callback()
{
    if (!endpoint_valid_ || executing_) return;

    Eigen::Vector3d endpoint;
    std::string frame_id;
    {
        std::lock_guard<std::mutex> lock(endpoint_mutex_);
        endpoint = latest_endpoint_;
        frame_id = latest_frame_id_;
    }

    auto target_pose = create_approach_pose(endpoint, frame_id);

    // Check if already close enough
    auto current_pose = move_group_->getCurrentPose();
    double dx = target_pose.pose.position.x - current_pose.pose.position.x;
    double dy = target_pose.pose.position.y - current_pose.pose.position.y;
    double dz = target_pose.pose.position.z - current_pose.pose.position.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (distance < position_tolerance_) return;

    RCLCPP_INFO(this->get_logger(),
        "Following DLO endpoint: [%.3f, %.3f, %.3f], distance=%.3fm",
        target_pose.pose.position.x, target_pose.pose.position.y,
        target_pose.pose.position.z, distance);

    executing_ = true;
    bool success = plan_and_execute(target_pose);
    if (!success) {
        RCLCPP_WARN(this->get_logger(), "Failed to plan to DLO endpoint");
    }
    executing_ = false;
}

geometry_msgs::msg::PoseStamped DloManipulationNode::create_approach_pose(
    const Eigen::Vector3d & endpoint, const std::string & frame_id)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = this->now();

    // Position: above the endpoint by approach_distance
    pose.pose.position.x = endpoint.x();
    pose.pose.position.y = endpoint.y();
    pose.pose.position.z = endpoint.z() + approach_distance_;

    // Orientation: tool pointing straight down (180 deg rotation about Y)
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 1.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 0.0;

    return pose;
}

bool DloManipulationNode::plan_and_execute(
    const geometry_msgs::msg::PoseStamped & target_pose)
{
    move_group_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(this->get_logger(), "Planning succeeded, executing...");
        move_group_->execute(plan);
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        return false;
    }
}

void DloManipulationNode::add_collision_objects()
{
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    // Table
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = move_group_->getPlanningFrame();
    table.id = "table";

    shape_msgs::msg::SolidPrimitive table_shape;
    table_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    table_shape.dimensions = {1.2, 0.8, 0.75};

    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = 0.5;
    table_pose.position.y = 0.0;
    table_pose.position.z = 0.375;
    table_pose.orientation.w = 1.0;

    table.primitives.push_back(table_shape);
    table.primitive_poses.push_back(table_pose);
    table.operation = moveit_msgs::msg::CollisionObject::ADD;
    collision_objects.push_back(table);

    // Camera mount
    moveit_msgs::msg::CollisionObject camera_mount;
    camera_mount.header.frame_id = move_group_->getPlanningFrame();
    camera_mount.id = "camera_mount";

    shape_msgs::msg::SolidPrimitive mount_shape;
    mount_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    mount_shape.dimensions = {0.05, 0.05, 0.15};

    geometry_msgs::msg::Pose mount_pose;
    mount_pose.position.x = 0.5309;
    mount_pose.position.y = 0.0301;
    mount_pose.position.z = 1.2624;
    mount_pose.orientation.w = 1.0;

    camera_mount.primitives.push_back(mount_shape);
    camera_mount.primitive_poses.push_back(mount_pose);
    camera_mount.operation = moveit_msgs::msg::CollisionObject::ADD;
    collision_objects.push_back(camera_mount);

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
