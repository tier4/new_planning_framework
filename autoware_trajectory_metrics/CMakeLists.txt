cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_metrics)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_trajectory_selector_common plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/metrics.cpp
  src/utils.cpp
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gmock(test_${PROJECT_NAME}
    test/test_utils.cpp
  )

  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )

  target_include_directories(test_${PROJECT_NAME} PRIVATE src)
endif()

ament_auto_package()
