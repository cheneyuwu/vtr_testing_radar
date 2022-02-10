cmake_minimum_required(VERSION 3.16)
project(vtr_testing_lidar)

## Common setup for vtr packages
include("$ENV{VTRSRC}/main/src/vtr_common/vtr_include.cmake")


## Find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_lidar REQUIRED)

## C++ Libraries

## Executables
include_directories(
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(${PROJECT_NAME}_boreas_preprocessing src/boreas_preprocessing.cpp)
ament_target_dependencies(${PROJECT_NAME}_boreas_preprocessing rclcpp vtr_common vtr_logging vtr_lidar)

add_executable(${PROJECT_NAME}_boreas_odometry src/boreas_odometry.cpp)
ament_target_dependencies(${PROJECT_NAME}_boreas_odometry rclcpp vtr_common vtr_logging vtr_lidar)

add_executable(${PROJECT_NAME}_boreas_localization src/boreas_localization.cpp)
ament_target_dependencies(${PROJECT_NAME}_boreas_localization rclcpp vtr_common vtr_logging vtr_lidar)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_boreas_preprocessing
    ${PROJECT_NAME}_boreas_odometry
    ${PROJECT_NAME}_boreas_localization
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
