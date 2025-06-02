cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_ranker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-error=deprecated-declarations)
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

generate_parameter_library(${PROJECT_NAME}_param
  param/parameter_struct.yaml
)

include_directories(
  SYSTEM
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_param
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::trajectory_selector::trajectory_ranker::TrajectoryRankerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
