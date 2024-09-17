cmake_minimum_required(VERSION 3.14)
project(autoware_dummy_trajectory_publisher)

find_package(autoware_cmake REQUIRED)
autoware_package()

generate_parameter_library(dummy_trajectory_publisher_param
  param/dummy_trajectory_publisher_param.yaml
)

include_directories(
  SYSTEM
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
)

target_link_libraries(${PROJECT_NAME}
  dummy_trajectory_publisher_param
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::trajectory_selector::dummy_trajectory_publisher::DummyTrajectoryPublisherNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
