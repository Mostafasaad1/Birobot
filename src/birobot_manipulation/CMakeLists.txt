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
find_package(shape_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(birobot_interfaces REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_msgs REQUIRED)

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
  shape_msgs
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

# ── BehaviorTree.CPP v4 Handover Nodes & Coordinator ──────────────────────────
add_library(handover_bt_nodes_lib SHARED
  src/bt_nodes/handover_bt_nodes.cpp
)
target_include_directories(handover_bt_nodes_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(handover_bt_nodes_lib
  rclcpp
  geometry_msgs
  shape_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_ros
  behaviortree_cpp
  std_msgs
  std_srvs
  moveit_msgs
)

add_executable(birobot_bt_coordinator_node
  src/birobot_bt_coordinator_node.cpp
)
target_link_libraries(birobot_bt_coordinator_node
  handover_bt_nodes_lib
)
ament_target_dependencies(birobot_bt_coordinator_node
  rclcpp
  rclcpp_action
  control_msgs
  ament_index_cpp
  tf2_ros
  behaviortree_cpp
)

# ── BehaviorTree.CPP v4 AutoPick Nodes & Coordinator ──────────────────────────
add_library(auto_pick_bt_nodes_lib SHARED
  src/bt_nodes/auto_pick_bt_nodes.cpp
)
target_include_directories(auto_pick_bt_nodes_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(auto_pick_bt_nodes_lib
  rclcpp
  geometry_msgs
  shape_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_ros
  behaviortree_cpp
  std_msgs
  birobot_interfaces
)

add_executable(auto_pick_coordinator
  src/auto_pick_coordinator.cpp
)
target_link_libraries(auto_pick_coordinator
  auto_pick_bt_nodes_lib
  handover_bt_nodes_lib
)
ament_target_dependencies(auto_pick_coordinator
  rclcpp
  rclcpp_action
  control_msgs
  ament_index_cpp
  tf2_ros
  behaviortree_cpp
  birobot_interfaces
)

install(TARGETS
  mtc_pick_place_lib
  mtc_pick_place_node
  handover_bt_nodes_lib
  birobot_bt_coordinator_node
  auto_pick_bt_nodes_lib
  auto_pick_coordinator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/randomize_object.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mtc_pipeline test/test_mtc_pipeline.cpp)
  target_link_libraries(test_mtc_pipeline mtc_pick_place_lib)
  ament_target_dependencies(test_mtc_pipeline rclcpp rclcpp_lifecycle birobot_interfaces)

  ament_add_gtest(test_bt_nodes test/test_bt_nodes.cpp)
  target_link_libraries(test_bt_nodes handover_bt_nodes_lib)
  ament_target_dependencies(test_bt_nodes rclcpp tf2_ros behaviortree_cpp)
endif()

ament_package()
