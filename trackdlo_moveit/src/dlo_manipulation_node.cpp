#include "trackdlo_moveit/dlo_manipulation_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

DloManipulationNode::DloManipulationNode()
    : Node("dlo_manipulation")
{
    // Declare parameters
    this->declare_parameter<std::string>("planning_group", "ur_manipulator");
    this->declare_parameter<std::string>("results_topic", "/trackdlo/results_pc");
    this->declare_parameter<double>("approach_distance", 0.1);
    this->declare_parameter<double>("grasp_offset_z", 0.05);

    planning_group_ = this->get_parameter("planning_group").as_string();
    results_topic_ = this->get_parameter("results_topic").as_string();
    approach_distance_ = this->get_parameter("approach_distance").as_double();
    grasp_offset_z_ = this->get_parameter("grasp_offset_z").as_double();

    // Deferred init for MoveGroupInterface (needs shared_from_this())
    init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(0),
        std::bind(&DloManipulationNode::deferred_init, this));
}

void DloManipulationNode::deferred_init()
{
    init_timer_->cancel();

    // Initialize MoveIt interfaces
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_);
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // Set planning parameters
    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);

    // Add collision objects (table, camera mount)
    add_collision_objects();

    // Subscribe to TrackDLO results
    results_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        results_topic_, 10,
        std::bind(&DloManipulationNode::results_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "DLO Manipulation node initialized with planning group: %s",
                planning_group_.c_str());
}

void DloManipulationNode::results_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
    // Convert point cloud to PCL
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(*msg, pcl_cloud);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(pcl_cloud, cloud);

    if (cloud.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty results point cloud");
        return;
    }

    // Get the first endpoint of the DLO (index 0)
    auto & first_pt = cloud.points[0];
    // Get the last endpoint of the DLO
    auto & last_pt = cloud.points[cloud.size() - 1];

    RCLCPP_INFO_STREAM(this->get_logger(),
        "DLO endpoints: [" << first_pt.x << ", " << first_pt.y << ", " << first_pt.z << "] -> ["
        << last_pt.x << ", " << last_pt.y << ", " << last_pt.z << "]");
}

bool DloManipulationNode::plan_and_execute(
    const geometry_msgs::msg::PoseStamped & target_pose)
{
    move_group_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

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

    // Table collision object
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

    // Camera mount collision object
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
    RCLCPP_INFO(this->get_logger(), "Added %zu collision objects to planning scene",
                collision_objects.size());
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DloManipulationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
