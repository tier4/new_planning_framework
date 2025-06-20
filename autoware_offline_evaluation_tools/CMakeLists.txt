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

if(BUILD_TESTING)
  ament_add_ros_isolated_gmock(test_${PROJECT_NAME}
    test/test_bag_handler.cpp
    test/test_replay_evaluation.cpp
    test/test_ground_truth_generation.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
  ament_target_dependencies(test_${PROJECT_NAME}
    autoware_test_utils
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  rviz
)
