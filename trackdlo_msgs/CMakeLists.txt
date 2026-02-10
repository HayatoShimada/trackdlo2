cmake_minimum_required(VERSION 3.8)
project(trackdlo_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Future: uncomment when custom messages are added
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/DLOState.msg"
#   DEPENDENCIES std_msgs geometry_msgs
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
