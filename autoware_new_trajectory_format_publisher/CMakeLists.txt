cmake_minimum_required(VERSION 3.14)
project(autoware_new_trajectory_format_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  SYSTEM
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::trajectory_selector::new_trajectory_format_publisher::NewTrajectoryFormatPublisher"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package()
