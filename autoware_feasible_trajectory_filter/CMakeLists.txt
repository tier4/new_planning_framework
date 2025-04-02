cmake_minimum_required(VERSION 3.14)
project(autoware_feasible_trajectory_filter)

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
  src/utils.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_param
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::trajectory_selector::feasible_trajectory_filter::FeasibleTrajectoryFilterNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gmock(test_${PROJECT_NAME}
    test/test_util.cpp
  )

  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )

  ament_target_dependencies(test_${PROJECT_NAME}
    autoware_test_utils
  )
endif()

ament_auto_package()
