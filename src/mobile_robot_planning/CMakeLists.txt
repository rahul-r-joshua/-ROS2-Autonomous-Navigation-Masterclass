cmake_minimum_required(VERSION 3.8)
project(mobile_robot_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)

include_directories(include)


add_library(dijkstra_planner SHARED src/dijkstra_planner.cpp)
ament_target_dependencies(dijkstra_planner rclcpp rclcpp_action nav_msgs geometry_msgs tf2_ros nav2_core nav2_util nav2_costmap_2d pluginlib nav2_msgs)

add_library(a_star_planner SHARED src/a_star_planner.cpp)
ament_target_dependencies(a_star_planner rclcpp rclcpp_action nav_msgs geometry_msgs tf2_ros nav2_core nav2_util nav2_costmap_2d pluginlib nav2_msgs)

pluginlib_export_plugin_description_file(nav2_core global_planner_plugins.xml)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/dijkstra_planner.py
  scripts/a_star_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  dijkstra_planner
  a_star_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES global_planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config behavior_tree launch rviz
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  set(ament_cmake_copyright_FOUND TRUE)
  
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp nav_msgs geometry_msgs tf2_ros nav2_core nav2_util nav2_costmap_2d pluginlib)
ament_export_libraries(
  dijkstra_planner
  a_star_planner
)
ament_package()
