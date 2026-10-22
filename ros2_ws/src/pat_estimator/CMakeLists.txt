cmake_minimum_required(VERSION 3.8)
project(pat_estimator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pat_msgs REQUIRED)
find_package(pat_common REQUIRED)
find_package(Threads REQUIRED)

# ROS-free estimator core (A.3.3, TD-4, TD-7).
add_library(${PROJECT_NAME}_core src/residual_los_estimator.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Lifecycle node wrapper and the two-executor executable (A.4.2).
add_library(${PROJECT_NAME}_node src/state_estimator_node.cpp)
target_link_libraries(${PROJECT_NAME}_node PUBLIC
  ${PROJECT_NAME}_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${lifecycle_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${pat_msgs_TARGETS}
  pat_common::pat_common)

add_executable(state_estimator src/state_estimator_main.cpp)
target_link_libraries(state_estimator ${PROJECT_NAME}_node Threads::Threads)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS state_estimator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_residual_los_estimator test/test_residual_los_estimator.cpp TIMEOUT 120)
  target_link_libraries(test_residual_los_estimator ${PROJECT_NAME}_core Threads::Threads)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle lifecycle_msgs sensor_msgs pat_msgs pat_common)
ament_package()
