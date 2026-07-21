cmake_minimum_required(VERSION 3.8)
project(birobot_description)

if(NOT CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch vendor
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright(EXCLUDE vendor)
  ament_flake8(EXCLUDE vendor)
  ament_lint_cmake()
  ament_pep257(EXCLUDE vendor)
  ament_xmllint(package.xml)
endif()

ament_package()
