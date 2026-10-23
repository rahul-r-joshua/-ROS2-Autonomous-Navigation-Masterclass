/**
 * ============================================================================
 * Mobile Robot Differential Drive Controller Header (C++)
 * ----------------------------------------------------------------------------
 * Author: Rahul Ramasamy
 * Email: rahul.r.joshua123@gmail.com
 * GitHub: https://github.com/rahul-r-joshua
 * Portfolio: https://rahul-r-joshua.github.io/portfolio/
 *
 * Designed for ROS 2 Mobile Robotics Workshop.
 * ============================================================================
 */

#ifndef MOBILE_ROBOT_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_
#define MOBILE_ROBOT_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

class DiffDriveControllerCpp : public rclcpp::Node
{
public:
  DiffDriveControllerCpp();

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void encoderCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void updateLoop();

  // Physical Parameters
  double r_;
  double L_;
  double rate_;
  std::string odom_frame_;
  std::string base_frame_;
  int cpr_;
  bool publish_joint_states_;
  double cmd_vel_timeout_;

  // Kinematic State
  double cmd_linear_x_;
  double cmd_angular_z_;
  double wheel_speed_left_;
  double wheel_speed_right_;
  double left_wheel_pos_;
  double right_wheel_pos_;
  double x_;
  double y_;
  double theta_;
  rclcpp::Time last_time_;
  rclcpp::Time last_cmd_vel_time_;
  bool has_cmd_vel_;
  int stop_count_;

  // Encoder & Joint Feedback Tracking
  bool has_encoder_data_;
  bool has_prev_ticks_;
  int32_t prev_left_ticks_;
  int32_t prev_right_ticks_;
  bool has_joint_feedback_;
  bool has_prev_joint_pos_;
  double prev_left_joint_pos_;
  double prev_right_joint_pos_;
  double delta_s_left_;
  double delta_s_right_;

  // ROS 2 Interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_gazebo_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // MOBILE_ROBOT_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_
