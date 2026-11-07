cmake_minimum_required(VERSION 3.8)
project(handy_calibrate)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(handy_motor_api REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(handy_calibrate src/handy_calibrate.cpp)
ament_target_dependencies(handy_calibrate ament_index_cpp)
target_link_libraries(handy_calibrate handy_motor_api::handy_motor_api yaml-cpp)

install(TARGETS handy_calibrate DESTINATION lib/${PROJECT_NAME})

ament_package()
