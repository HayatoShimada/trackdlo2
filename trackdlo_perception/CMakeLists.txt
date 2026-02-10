cmake_minimum_required(VERSION 3.8)
project(trackdlo_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL 1.12 REQUIRED COMPONENTS common io filters)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Core algorithm library
add_library(trackdlo_core SHARED
  src/trackdlo.cpp
  src/utils.cpp
  src/evaluator.cpp
)
target_link_libraries(trackdlo_core
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)
ament_target_dependencies(trackdlo_core
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  cv_bridge
  image_transport
  pcl_conversions
)

# Main tracking node
add_executable(trackdlo src/trackdlo_node.cpp)
target_link_libraries(trackdlo trackdlo_core)
ament_target_dependencies(trackdlo
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  cv_bridge
  image_transport
  pcl_conversions
  message_filters
  tf2_ros
)

# Evaluation node
add_executable(run_evaluation src/run_evaluation.cpp)
target_link_libraries(run_evaluation trackdlo_core)
ament_target_dependencies(run_evaluation
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  cv_bridge
  image_transport
  pcl_conversions
  message_filters
  tf2_ros
  rosbag2_cpp
  ament_index_cpp
)

# Install C++ targets
install(TARGETS
  trackdlo
  run_evaluation
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS trackdlo_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/init_tracker
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
