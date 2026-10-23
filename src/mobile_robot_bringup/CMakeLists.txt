cmake_minimum_required(VERSION 3.8)
project(mobile_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 1. C++ Safety Zone Node
add_executable(safety_zone_controller_cpp src/safety_zone_controller.cpp)
target_include_directories(safety_zone_controller_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(safety_zone_controller_cpp
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
)

# Install C++ binary
install(TARGETS
  safety_zone_controller_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ headers
install(
  DIRECTORY include/
  DESTINATION include
)

# 2. Python Script
install(PROGRAMS
  scripts/safety_zone_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# 3. Install Launch and Config directories
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
