cmake_minimum_required(VERSION 3.5)
project(autoware_new_planning_msgs_converter)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/trajectory_to_trajectories.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::trajectory_selector::new_planning_msgs_converter::TrajectoryToTrajectories"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package()
