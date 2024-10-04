cmake_minimum_required(VERSION 3.14)
project(autoware_offline_evaluation_tools)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  SYSTEM
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
  src/utils.cpp
  src/bag_handler.cpp
  src/evaluation.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::trajectory_selector::offline_evaluation_tools::OfflineEvaluatorNode"
  EXECUTABLE offline_evaluator_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  rviz
)
