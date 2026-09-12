cmake_minimum_required(VERSION 3.8)
project(birobot_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(birobot_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

include_directories(
  include
)

add_library(birobot_rviz_plugins SHARED
  src/birobot_control_panel.cpp
  include/birobot_rviz_plugins/birobot_control_panel.hpp
)

target_include_directories(birobot_rviz_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(birobot_rviz_plugins
  Qt5::Widgets
)

ament_target_dependencies(birobot_rviz_plugins
  rclcpp
  rviz_common
  pluginlib
  birobot_interfaces
  std_srvs
  std_msgs
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS
  birobot_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
