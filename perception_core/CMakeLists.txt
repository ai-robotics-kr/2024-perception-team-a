cmake_minimum_required(VERSION 3.5)
project(perception_core)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)

# find_package(rosidl_default_generators REQUIRED)

nav2_package()

set(dependencies
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  visualization_msgs
  nav_msgs
  tf2_ros
)

install(DIRECTORY include/
  DESTINATION include/
)


# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Object.msg"
#   DEPENDENCIES builtin_interfaces nav_msgs geometry_msgs
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
