cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_ranker)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  SYSTEM
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::trajectory_selector::feasible_trajectory_filter::TrajectoryRankerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
