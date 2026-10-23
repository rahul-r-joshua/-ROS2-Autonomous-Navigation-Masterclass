cmake_minimum_required(VERSION 3.8)
project(mobile_robot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/serial_hardware_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch firmware
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
