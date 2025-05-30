cmake_minimum_required(VERSION 3.5)
project(autoware_new_planning_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrajectoryGeneratorInfo.msg"
  "msg/Trajectories.msg"
  "msg/Trajectory.msg"
  "msg/EvaluationInfo.msg"
  "msg/TrajectoriesDebug.msg"
  DEPENDENCIES
  geometry_msgs
  std_msgs
  autoware_planning_msgs
  unique_identifier_msgs
  builtin_interfaces
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
