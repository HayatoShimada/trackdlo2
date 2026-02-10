cmake_minimum_required(VERSION 3.8)
project(trackdlo_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(dlo_manipulation_node src/dlo_manipulation_node.cpp)
target_link_libraries(dlo_manipulation_node
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)
ament_target_dependencies(dlo_manipulation_node
  rclcpp
  sensor_msgs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_core
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

install(TARGETS
  dlo_manipulation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
