#ifndef MOBILE_ROBOT_BRINGUP__SAFETY_ZONE_CONTROLLER_HPP_
#define MOBILE_ROBOT_BRINGUP__SAFETY_ZONE_CONTROLLER_HPP_

#include <cmath>
#include <string>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace mobile_robot_bringup
{

class SafetyZoneControllerCpp : public rclcpp::Node
{
public:
  SafetyZoneControllerCpp();
  virtual ~SafetyZoneControllerCpp() = default;

private:
  // ROS Callbacks
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void cmdRawCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void safetyStopCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void controlLoop();

  // Helper Methods
  void publishMarkers();

  // Subscribers & Publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_raw_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_joy_vel_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_safety_stop_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_safety_enable_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_safe_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  double red_dist_;
  double yellow_dist_;
  double fov_rad_;
  double slowdown_factor_;
  double lidar_x_offset_;

  // State
  double min_obstacle_distance_;
  std::string zone_state_;
  geometry_msgs::msg::Twist raw_cmd_;
  rclcpp::Time last_cmd_time_;
  bool manual_safety_active_{false};
  bool has_fresh_raw_cmd_{false};
  bool safety_zone_enabled_{true};
};

}  // namespace mobile_robot_bringup

#endif  // MOBILE_ROBOT_BRINGUP__SAFETY_ZONE_CONTROLLER_HPP_
