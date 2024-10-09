cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_metrics)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_trajectory_evaluator plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/metrics.cpp
  src/utils.cpp
)

ament_auto_package()
