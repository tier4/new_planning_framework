cmake_minimum_required(VERSION 3.14)
project(autoware_valid_trajectory_filter)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  SYSTEM
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
  src/utils.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::trajectory_selector::valid_trajectory_filter::ValidTrajectoryFilterNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package()
