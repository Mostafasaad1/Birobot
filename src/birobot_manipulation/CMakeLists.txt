cmake_minimum_required(VERSION 3.8)
project(birobot_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(birobot_interfaces REQUIRED)

include_directories(
  include
)

add_library(mtc_pick_place_lib SHARED
  src/mtc_pick_place_node.cpp
)
target_include_directories(mtc_pick_place_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mtc_pick_place_lib
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  geometry_msgs
  sensor_msgs
  diagnostic_msgs
  lifecycle_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  birobot_interfaces
)

add_executable(mtc_pick_place_node
  src/main.cpp
)
target_link_libraries(mtc_pick_place_node
  mtc_pick_place_lib
)
ament_target_dependencies(mtc_pick_place_node
  rclcpp
  rclcpp_lifecycle
)

install(TARGETS mtc_pick_place_lib mtc_pick_place_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mtc_pipeline test/test_mtc_pipeline.cpp)
  target_link_libraries(test_mtc_pipeline mtc_pick_place_lib)
  ament_target_dependencies(test_mtc_pipeline rclcpp rclcpp_lifecycle birobot_interfaces)
endif()

ament_package()
