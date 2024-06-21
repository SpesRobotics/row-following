cmake_minimum_required(VERSION 3.8)
project(row_following_behavior)

get_filename_component(BEHAVIOR_DIRECTORY behaviors ABSOLUTE CACHE)
add_compile_definitions(BEHAVIOR_DIRECTORY="${BEHAVIOR_DIRECTORY}")

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(row_following_msgs REQUIRED)

# Add executables
add_executable(behavior src/behavior.cpp)
ament_target_dependencies(
  behavior
  nav_msgs
  nav2_msgs
  row_following_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  tf2_msgs
  geometry_msgs
  behaviortree_cpp
  behaviortree_ros2
)

add_executable(generate_path src/generate_path.cpp)
ament_target_dependencies(
  generate_path
  nav_msgs
  nav2_msgs
  row_following_msgs
  tf2_geometry_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  tf2_msgs
  geometry_msgs
  behaviortree_cpp
  behaviortree_ros2
)

# Include directories
target_include_directories(behavior PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(generate_path PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS behavior
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS generate_path
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Package exports
ament_export_dependencies(
  nav2
  nav_msgs
  nav2_msgs
  tf2_geometry_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  tf2_msgs
  geometry_msgs
  behaviortree_cpp
  behaviortree_ros2
  row_following_msgs
)

ament_export_include_directories(
  include
)

ament_package()
