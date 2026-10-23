cmake_minimum_required(VERSION 3.8)
project(mobile_robot_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

add_library(pd_motion SHARED src/pd_motion.cpp)
ament_target_dependencies(pd_motion
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  pluginlib
  nav2_core
  nav2_costmap_2d
  nav2_util
)

add_library(pure_pursuit SHARED src/pure_pursuit.cpp)
ament_target_dependencies(pure_pursuit
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  pluginlib
  nav2_core
  nav2_costmap_2d
  nav2_util
)

pluginlib_export_plugin_description_file(nav2_core motion_controller_plugins.xml)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/pd_motion.py
  scripts/pure_pursuit.py
  scripts/plan_and_follow.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  pd_motion
  pure_pursuit
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES motion_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  pluginlib
  nav2_core
  nav2_costmap_2d
  nav2_util
)
ament_export_libraries(pd_motion pure_pursuit)
ament_package()
