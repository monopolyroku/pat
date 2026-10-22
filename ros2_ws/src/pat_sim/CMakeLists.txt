cmake_minimum_required(VERSION 3.8)
project(pat_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pat_msgs REQUIRED)
find_package(pat_common REQUIRED)

# ROS-free physics.
add_library(plant_model src/plant_model.cpp)
target_include_directories(plant_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Node as a component, plus a generated standalone executable.
add_library(synthetic_plant_component SHARED src/synthetic_plant_node.cpp)
target_link_libraries(synthetic_plant_component PUBLIC
  plant_model
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${pat_msgs_TARGETS}
  pat_common::pat_common)
rclcpp_components_register_node(synthetic_plant_component
  PLUGIN "pat_sim::SyntheticPlantNode"
  EXECUTABLE synthetic_plant)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS plant_model synthetic_plant_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/score_estimate.py scripts/run_scenario.sh DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_plant_model test/test_plant_model.cpp)
  target_link_libraries(test_plant_model plant_model)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs pat_msgs pat_common)
ament_package()
