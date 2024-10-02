cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_selector_common)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  SYSTEM
)

ament_auto_package()
