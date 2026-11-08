cmake_minimum_required(VERSION 3.8)
project(handy_description)

# Handy robot model: URDF/xacro, meshes, ros2_control block, servo table and calibration.

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
