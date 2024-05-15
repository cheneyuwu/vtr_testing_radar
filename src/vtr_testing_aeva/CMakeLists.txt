cmake_minimum_required(VERSION 3.16)
project(vtr_testing_aeva)

## Common setup for vtr packages
include("$ENV{VTRSRC}/main/src/vtr_common/vtr_include.cmake")


## Find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_lidar REQUIRED)
find_package(vtr_torch REQUIRED)

## C++ Libraries

## Executables
include_directories(
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(${PROJECT_NAME}_aeva_preprocessing src/aeva_preprocessing.cpp)
ament_target_dependencies(${PROJECT_NAME}_aeva_preprocessing rclcpp std_msgs vtr_common vtr_logging vtr_lidar vtr_torch)

add_executable(${PROJECT_NAME}_aeva_odometry src/aeva_odometry.cpp)
ament_target_dependencies(${PROJECT_NAME}_aeva_odometry rclcpp std_msgs vtr_common vtr_logging vtr_lidar vtr_torch)

add_executable(${PROJECT_NAME}_aeva_localization src/aeva_localization.cpp)
ament_target_dependencies(${PROJECT_NAME}_aeva_localization rclcpp std_msgs vtr_common vtr_logging vtr_lidar vtr_torch)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_aeva_preprocessing
    ${PROJECT_NAME}_aeva_odometry
    ${PROJECT_NAME}_aeva_localization
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
