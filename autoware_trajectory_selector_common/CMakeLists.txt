cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_selector_common)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  SYSTEM
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/utils.cpp
  src/evaluation.cpp
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gmock(test_${PROJECT_NAME}
    test/test_utils.cpp
  )

  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )

  ament_target_dependencies(test_${PROJECT_NAME}
    autoware_test_utils
  )
endif()

ament_auto_package()
