cmake_minimum_required(VERSION 3.8)
project(birobot_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common segmentation filters search cluster)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Core perception library
add_library(irregular_object_pose_estimator_lib SHARED
  src/irregular_object_pose_estimator.cpp
)
target_include_directories(irregular_object_pose_estimator_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(irregular_object_pose_estimator_lib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
  visualization_msgs
  diagnostic_msgs
  lifecycle_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  pcl_conversions
  cv_bridge
  image_transport
)
target_link_libraries(irregular_object_pose_estimator_lib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Node Executable
add_executable(birobot_perception_node
  src/perception_node.cpp
)
target_link_libraries(birobot_perception_node
  irregular_object_pose_estimator_lib
)
ament_target_dependencies(birobot_perception_node
  rclcpp
  rclcpp_lifecycle
)

# Install rules
install(TARGETS irregular_object_pose_estimator_lib birobot_perception_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/monitor_pick_points.py
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_pointcloud_subscriber test/test_pointcloud_subscriber.cpp)
  target_link_libraries(test_pointcloud_subscriber irregular_object_pose_estimator_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_pointcloud_subscriber rclcpp rclcpp_lifecycle sensor_msgs diagnostic_msgs)

  ament_add_gtest(test_ransac_filtering test/test_ransac_filtering.cpp)
  target_link_libraries(test_ransac_filtering irregular_object_pose_estimator_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_ransac_filtering rclcpp rclcpp_lifecycle sensor_msgs)

  ament_add_gtest(test_pca_pose_estimation test/test_pca_pose_estimation.cpp)
  target_link_libraries(test_pca_pose_estimation irregular_object_pose_estimator_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_pca_pose_estimation rclcpp rclcpp_lifecycle geometry_msgs tf2_ros)

  ament_add_gtest(test_end_to_end_perception test/test_end_to_end_perception.cpp)
  target_link_libraries(test_end_to_end_perception irregular_object_pose_estimator_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_end_to_end_perception rclcpp rclcpp_lifecycle sensor_msgs geometry_msgs tf2_ros)
endif()

ament_package()
