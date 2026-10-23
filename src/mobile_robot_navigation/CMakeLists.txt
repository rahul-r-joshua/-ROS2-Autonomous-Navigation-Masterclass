cmake_minimum_required(VERSION 3.8)
project(mobile_robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY behavior_tree config launch rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  scripts/waypoint_follower.py
  scripts/nav_to_pose.py
  scripts/patrol_loop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

