cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_concatenater)

find_package(autoware_cmake REQUIRED)
autoware_package()

generate_parameter_library(trajectory_concatenater_param
  param/trajectory_concatenater_param.yaml
)

include_directories(
  SYSTEM
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
)

target_link_libraries(${PROJECT_NAME}
  trajectory_concatenater_param
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::trajectory_selector::trajectory_concatenater::TrajectoryConcatenaterNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
