/*
================================================================================
pd_motion.cpp
--------------------------------------------------------------------------------
Author & Maintainer: Rahul Ramasamy
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/
LinkedIn: https://www.linkedin.com/in/rahul-ramasamy-in/
Email: rahul.r.joshua123@gmail.com

Workshop Edition for ROS 2 Mobile Robotics Workshop.
================================================================================
*/

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "mobile_robot_motion/pd_motion.hpp"

namespace mobile_robot_motion
{
void PDMotion::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;

  auto node = node_.lock();

  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf_buffer;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".kp",
    rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".kd",
    rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_velocity",
    rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_velocity",
    rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".step_size",
    rclcpp::ParameterValue(0.2));

  node->get_parameter(plugin_name_ + ".kp", kp_);
  node->get_parameter(plugin_name_ + ".kd", kd_);
  node->get_parameter(plugin_name_ + ".max_linear_velocity", max_linear_velocity_);
  node->get_parameter(plugin_name_ + ".max_angular_velocity", max_angular_velocity_);
  node->get_parameter(plugin_name_ + ".step_size", step_size_);

  next_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("pd/next_pose", 1);
}

void PDMotion::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up PDMotion");
  next_pose_pub_.reset();
}

void PDMotion::activate()
{
  auto node = node_.lock();
  RCLCPP_INFO(logger_, "Activating PDMotion");
  next_pose_pub_->on_activate();
  last_cycle_time_ = node->get_clock()->now();
}

void PDMotion::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating PDMotion");
  next_pose_pub_->on_deactivate();
}

geometry_msgs::msg::TwistStamped PDMotion::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker *)
{
  auto node = node_.lock();
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = robot_pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();

  if(global_plan_.poses.empty()){
    RCLCPP_ERROR(logger_, "Empty Plan!");
    return cmd_vel;
  }

  if(!transformPlan(robot_pose.header.frame_id)){
    RCLCPP_ERROR(logger_, "Unable to transform Plan in robot's frame");
    return cmd_vel;
  }

  tf2::Transform robot_tf;
  tf2::fromMsg(robot_pose.pose, robot_tf);

  // 1. Goal orientation alignment when reaching target position
  tf2::Transform goal_tf, goal_robot_tf;
  tf2::fromMsg(global_plan_.poses.back().pose, goal_tf);
  goal_robot_tf = robot_tf.inverse() * goal_tf;

  double dist_to_goal = std::hypot(goal_robot_tf.getOrigin().getX(), goal_robot_tf.getOrigin().getY());
  double roll, pitch, goal_yaw_error;
  goal_robot_tf.getBasis().getRPY(roll, pitch, goal_yaw_error);

  double xy_tol = 0.25;

  // Once position is within tolerance, stop linear drive and align heading accurately
  if (dist_to_goal <= xy_tol) {
    cmd_vel.twist.linear.x = 0.0;
    if (std::abs(goal_yaw_error) > 0.05) {
      double rot_speed = 2.0 * goal_yaw_error;
      if (std::abs(rot_speed) < 0.45) {
        rot_speed = (goal_yaw_error > 0.0) ? 0.45 : -0.45;
      }
      cmd_vel.twist.angular.z = std::clamp(rot_speed, -max_angular_velocity_, max_angular_velocity_);
    } else {
      cmd_vel.twist.angular.z = 0.0;
    }
    last_cycle_time_ = node->get_clock()->now();
    return cmd_vel;
  }

  // 2. Select next tracking pose along path
  auto next_pose = getNextPose(robot_pose);
  next_pose_pub_->publish(next_pose);
        
  tf2::Transform next_pose_tf, next_pose_robot_tf;
  tf2::fromMsg(next_pose.pose, next_pose_tf);
  next_pose_robot_tf = robot_tf.inverse() * next_pose_tf;

  double target_x = next_pose_robot_tf.getOrigin().getX();
  double target_y = next_pose_robot_tf.getOrigin().getY();
  double heading_error = std::atan2(target_y, target_x);

  double dt = (node->get_clock()->now() - last_cycle_time_).seconds();
  if (dt <= 0.0 || dt > 0.5) {
    dt = 0.05;
  }

  // 3. If waypoint is substantially behind robot (> 90 deg), rotate in place towards it
  if (std::abs(heading_error) > 1.57) {
    cmd_vel.twist.linear.x = 0.0;
    double rot_speed = 1.5 * heading_error;
    cmd_vel.twist.angular.z = std::clamp(rot_speed, -max_angular_velocity_, max_angular_velocity_);

    last_cycle_time_ = node->get_clock()->now();
    prev_angular_error_ = heading_error;
    prev_linear_error_ = 0.0;
    return cmd_vel;
  }

  // 4. Waypoint is in front hemisphere: Drive forward with PD cross-track and along-track control
  double angular_error = target_y;
  double angular_error_derivative = (angular_error - prev_angular_error_) / dt;
  double linear_error = target_x;
  double linear_error_derivative = (linear_error - prev_linear_error_) / dt;

  cmd_vel.twist.angular.z = std::clamp(kp_ * angular_error + kd_ * angular_error_derivative,
      -max_angular_velocity_, max_angular_velocity_);
  
  double v = kp_ * linear_error + kd_ * linear_error_derivative;
  double min_v = (dist_to_goal < 0.5) ? 0.0 : 0.08;
  v = std::clamp(v, min_v, max_linear_velocity_);
  v *= std::max(0.1, std::cos(heading_error));
  cmd_vel.twist.linear.x = v;
  
  last_cycle_time_ = node->get_clock()->now();
  prev_angular_error_ = angular_error;
  prev_linear_error_ = linear_error;

  return cmd_vel;
}

void PDMotion::setPlan(const nav_msgs::msg::Path & path)
{
  raw_plan_ = path;
  global_plan_ = path;
}

void PDMotion::setSpeedLimit(const double &, const bool &){}

geometry_msgs::msg::PoseStamped PDMotion::getNextPose(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  if (global_plan_.poses.empty()) {
    return robot_pose;
  }

  // 1. Find the closest pose on the global plan to the robot
  size_t closest_idx = 0;
  double min_dist_sq = std::numeric_limits<double>::max();
  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    double dx = global_plan_.poses[i].pose.position.x - robot_pose.pose.position.x;
    double dy = global_plan_.poses[i].pose.position.y - robot_pose.pose.position.y;
    double dist_sq = dx * dx + dy * dy;
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_idx = i;
    }
  }

  // 2. From closest pose, search FORWARD along the path for the look-ahead point
  for (size_t i = closest_idx; i < global_plan_.poses.size(); ++i) {
    double dx = global_plan_.poses[i].pose.position.x - robot_pose.pose.position.x;
    double dy = global_plan_.poses[i].pose.position.y - robot_pose.pose.position.y;
    double dist = std::hypot(dx, dy);
    if (dist >= step_size_) {
      return global_plan_.poses[i];
    }
  }

  // 3. If within step_size_ of the goal, target the goal pose directly
  return global_plan_.poses.back();
}

bool PDMotion::transformPlan(const std::string & frame)
{
  if (raw_plan_.poses.empty()) {
    return false;
  }
  if (raw_plan_.header.frame_id == frame) {
    global_plan_ = raw_plan_;
    return true;
  }
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(frame, raw_plan_.header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(logger_, "Couldn't transform plan from frame " <<
      raw_plan_.header.frame_id << " to frame " << frame << ": " << ex.what());
    return false;
  }
  global_plan_ = raw_plan_;
  for (auto & pose : global_plan_.poses) {
    tf2::doTransform(pose, pose, transform);
  }
  global_plan_.header.frame_id = frame;
  return true;
}

}  // namespace mobile_robot_motion

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mobile_robot_motion::PDMotion, nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(mobile_robot_motion::PDMotionPlanner, nav2_core::Controller)
