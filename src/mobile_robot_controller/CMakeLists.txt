cmake_minimum_required(VERSION 3.8)
project(mobile_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 1. C++ Controller Executable
add_executable(diff_drive_controller_cpp src/diff_drive_controller.cpp)
target_include_directories(diff_drive_controller_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(diff_drive_controller_cpp
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

# Install C++ binary
install(TARGETS
  diff_drive_controller_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ headers
install(
  DIRECTORY include/
  DESTINATION include
)

# 2. Python Scripts Installation
install(PROGRAMS
  scripts/diff_drive_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME diff_drive_controller_py
)

install(PROGRAMS
  scripts/diff_drive_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/twist_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# 3. Install Config, Launch directories
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
