/*
================================================================================
pure_pursuit.cpp
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
#include "mobile_robot_motion/pure_pursuit.hpp"

namespace mobile_robot_motion
{
void PurePursuit::configure(
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
    node, plugin_name_ + ".look_ahead_distance",
    rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_velocity",
    rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_velocity",
    rclcpp::ParameterValue(1.0));

  node->get_parameter(plugin_name_ + ".look_ahead_distance", look_ahead_distance_);
  node->get_parameter(plugin_name_ + ".max_linear_velocity", max_linear_velocity_);
  node->get_parameter(plugin_name_ + ".max_angular_velocity", max_angular_velocity_);

  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("pure_pursuit/carrot", 1);
}

void PurePursuit::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up PurePursuit");
  carrot_pub_.reset();
}

void PurePursuit::activate()
{
  RCLCPP_INFO(logger_, "Activating PurePursuit");
  carrot_pub_->on_activate();
}

void PurePursuit::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating PurePursuit");
  carrot_pub_->on_deactivate();
}

geometry_msgs::msg::TwistStamped PurePursuit::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker *)
{
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
    return cmd_vel;
  }

  // 2. Select look-ahead carrot pose along path
  auto carrot_pose = getCarrotPose(robot_pose);
  carrot_pub_->publish(carrot_pose);
        
  tf2::Transform carrot_pose_tf, carrot_pose_robot_tf;
  tf2::fromMsg(carrot_pose.pose, carrot_pose_tf);
  carrot_pose_robot_tf = robot_tf.inverse() * carrot_pose_tf;

  double target_x = carrot_pose_robot_tf.getOrigin().getX();
  double target_y = carrot_pose_robot_tf.getOrigin().getY();
  double heading_error = std::atan2(target_y, target_x);

  // 3. If carrot waypoint is substantially behind robot (> 90 deg), rotate in place towards it
  if (std::abs(heading_error) > 1.57) {
    cmd_vel.twist.linear.x = 0.0;
    double rot_speed = 1.5 * heading_error;
    cmd_vel.twist.angular.z = std::clamp(rot_speed, -max_angular_velocity_, max_angular_velocity_);
    return cmd_vel;
  }

  // 4. Carrot is in front hemisphere: Drive forward while steering with Pure Pursuit curvature
  tf2::toMsg(carrot_pose_robot_tf, carrot_pose.pose);
  double curvature = getCurvature(carrot_pose.pose);
        
  double linear_v = (dist_to_goal < 0.5) ? std::clamp(dist_to_goal, 0.08, max_linear_velocity_) : max_linear_velocity_;
  linear_v *= std::max(0.1, std::cos(heading_error));
  cmd_vel.twist.linear.x = linear_v;
  cmd_vel.twist.angular.z = std::clamp(curvature * max_angular_velocity_, -max_angular_velocity_, max_angular_velocity_);

  return cmd_vel;
}

void PurePursuit::setPlan(const nav_msgs::msg::Path & path)
{
  RCLCPP_INFO_STREAM(logger_, "Path received with " << path.poses.size() << " poses");
  RCLCPP_INFO_STREAM(logger_, "Path frame " << path.header.frame_id);
  raw_plan_ = path;
  global_plan_ = path;
}

void PurePursuit::setSpeedLimit(const double &, const bool &){}

geometry_msgs::msg::PoseStamped PurePursuit::getCarrotPose(const geometry_msgs::msg::PoseStamped & robot_pose)
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

  // 2. From closest pose, search FORWARD along the path for the look-ahead carrot
  for (size_t i = closest_idx; i < global_plan_.poses.size(); ++i) {
    double dx = global_plan_.poses[i].pose.position.x - robot_pose.pose.position.x;
    double dy = global_plan_.poses[i].pose.position.y - robot_pose.pose.position.y;
    double dist = std::hypot(dx, dy);
    if (dist >= look_ahead_distance_) {
      return global_plan_.poses[i];
    }
  }

  // 3. If remaining path is within look-ahead distance, target the goal pose directly
  return global_plan_.poses.back();
}

double PurePursuit::getCurvature(const geometry_msgs::msg::Pose & carrot_pose)
{
  const double carrot_dist =
  (carrot_pose.position.x * carrot_pose.position.x) +
  (carrot_pose.position.y * carrot_pose.position.y);
    
  // Find curvature of circle (k = 2 * y / L^2)
  if (carrot_dist > 0.001) {
    return 2.0 * carrot_pose.position.y / carrot_dist;
  } else {
    return 0.0;
  }
}

bool PurePursuit::transformPlan(const std::string & frame)
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
PLUGINLIB_EXPORT_CLASS(mobile_robot_motion::PurePursuit, nav2_core::Controller)
