/*
================================================================================
a_star_planner.cpp
--------------------------------------------------------------------------------
Author & Maintainer: Rahul Ramasamy
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/
LinkedIn: https://www.linkedin.com/in/rahul-ramasamy-in/
Email: rahul.r.joshua123@gmail.com

Workshop Edition for ROS 2 Mobile Robotics Workshop.
================================================================================
*/

#include <cmath>
#include <algorithm>
#include <vector>
#include <queue>
#include <chrono>

#include "mobile_robot_planning/a_star_planner.hpp"

namespace mobile_robot_planning
{
void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  smooth_client_ = rclcpp_action::create_client<nav2_msgs::action::SmoothPath>(node_, "smooth_path");

  plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/plan", 1);
  a_star_plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/a_star/path", 1);
  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      onGoalReceived(msg);
    });
}

void AStarPlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type AStarPlanner",
    name_.c_str());
  plan_pub_.reset();
  a_star_plan_pub_.reset();
  goal_sub_.reset();
}

void AStarPlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type AStarPlanner",
    name_.c_str());
  if (plan_pub_) {
    plan_pub_->on_activate();
  }
  if (a_star_plan_pub_) {
    a_star_plan_pub_->on_activate();
  }
}

void AStarPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type AStarPlanner",
    name_.c_str());
  if (plan_pub_) {
    plan_pub_->on_deactivate();
  }
  if (a_star_plan_pub_) {
    a_star_plan_pub_->on_deactivate();
  }
}

void AStarPlanner::onGoalReceived(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
{
  geometry_msgs::msg::PoseStamped goal_in_map = *goal;
  if (goal->header.frame_id != global_frame_) {
    if (!nav2_util::transformPoseInTargetFrame(*goal, goal_in_map, *tf_, global_frame_, 1.0)) {
      RCLCPP_WARN(
        node_->get_logger(),
        "AStarPlanner: Could not transform goal from [%s] to [%s].",
        goal->header.frame_id.c_str(), global_frame_.c_str());
      return;
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "AStarPlanner: Transformed goal from [%s] to [%s]: (%.2f, %.2f)",
      goal->header.frame_id.c_str(), global_frame_.c_str(),
      goal_in_map.pose.position.x, goal_in_map.pose.position.y);
  }

  geometry_msgs::msg::PoseStamped start;
  if (!nav2_util::getCurrentPose(start, *tf_, global_frame_, "base_footprint", 1.0)) {
    RCLCPP_WARN(node_->get_logger(), "AStarPlanner: Could not get robot pose for 2D Goal Pose.");
    return;
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "AStarPlanner: 2D Goal Pose received (%.2f, %.2f). Computing path...",
    goal_in_map.pose.position.x, goal_in_map.pose.position.y);

  nav_msgs::msg::Path path = createPlan(start, goal_in_map);
  if (!path.poses.empty()) {
    if (plan_pub_ && plan_pub_->is_activated()) {
      plan_pub_->publish(path);
    }
    if (a_star_plan_pub_ && a_star_plan_pub_->is_activated()) {
      a_star_plan_pub_->publish(path);
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "AStarPlanner: Published path with %zu poses to /plan and /a_star/path",
      path.poses.size());
  }
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  std::vector<std::pair<int, int>> explore_directions = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1},
    {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
  };

  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();
  std::vector<bool> visited(size_x * size_y, false);

  auto snapToFree = [this](GraphNode & n) {
    if (poseOnMap(n) && costmap_->getCost(n.x, n.y) < 253) return true;
    for (int r = 1; r <= 15; ++r) {
      GraphNode best_cand = n;
      double min_d_sq = 1e9;
      bool found = false;
      for (int dx = -r; dx <= r; ++dx) {
        for (int dy = -r; dy <= r; ++dy) {
          if (std::max(std::abs(dx), std::abs(dy)) != r) continue;
          GraphNode cand(n.x + dx, n.y + dy);
          if (poseOnMap(cand) && costmap_->getCost(cand.x, cand.y) < 253) {
            double d_sq = dx * dx + dy * dy;
            if (d_sq < min_d_sq) {
              min_d_sq = d_sq;
              best_cand = cand;
              found = true;
            }
          }
        }
      }
      if (found) {
        n = best_cand;
        return true;
      }
    }
    return false;
  };

  GraphNode start_node = worldToGrid(start.pose);
  GraphNode goal_node = worldToGrid(goal.pose);

  snapToFree(start_node);
  snapToFree(goal_node);

  std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;
  start_node.heuristic = manhattanDistance(start_node, goal_node);
  pending_nodes.push(start_node);
  visited[start_node.y * size_x + start_node.x] = true;

  GraphNode active_node;
  bool goal_reached = false;

  while (!pending_nodes.empty() && rclcpp::ok()) {
    active_node = pending_nodes.top();
    pending_nodes.pop();

    if (active_node == goal_node) {
      goal_reached = true;
      break;
    }

    for (const auto & dir : explore_directions) {
      GraphNode new_node = active_node + dir;
      if (poseOnMap(new_node)) {
        unsigned int idx = new_node.y * size_x + new_node.x;
        if (!visited[idx] && costmap_->getCost(new_node.x, new_node.y) < 253) {
          double step_cost = (dir.first != 0 && dir.second != 0) ? 1.414 : 1.0;
          new_node.cost = active_node.cost + step_cost + costmap_->getCost(new_node.x, new_node.y) * 0.1;
          new_node.heuristic = manhattanDistance(new_node, goal_node);
          new_node.prev = std::make_shared<GraphNode>(active_node);
          pending_nodes.push(new_node);
          visited[idx] = true;
        }
      }
    }
  }

  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  path.header.frame_id = global_frame_;

  if (!goal_reached) {
    RCLCPP_WARN(node_->get_logger(), "AStarPlanner: Unable to find valid path to goal!");
    return path;
  }

  geometry_msgs::msg::Pose last_pose = gridToWorld(active_node);
  geometry_msgs::msg::PoseStamped last_pose_stamped;
  last_pose_stamped.header.stamp = path.header.stamp;
  last_pose_stamped.header.frame_id = global_frame_;
  last_pose_stamped.pose = last_pose;
  path.poses.push_back(last_pose_stamped);

  while (active_node.prev && rclcpp::ok()) {
    active_node = *active_node.prev;
    last_pose = gridToWorld(active_node);
    last_pose_stamped.pose = last_pose;
    path.poses.push_back(last_pose_stamped);
  }
  std::reverse(path.poses.begin(), path.poses.end());

  // Compute orientations tangent to the path
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double yaw = 0.0;
    if (i + 1 < path.poses.size()) {
      double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
      double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
      yaw = std::atan2(dy, dx);
    } else if (i > 0) {
      double dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
      double dy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;
      yaw = std::atan2(dy, dx);
    }
    path.poses[i].pose.orientation.x = 0.0;
    path.poses[i].pose.orientation.y = 0.0;
    path.poses[i].pose.orientation.z = std::sin(yaw / 2.0);
    path.poses[i].pose.orientation.w = std::cos(yaw / 2.0);
  }

  // Snap path start to exact robot position and end to clicked goal pose
  if (!path.poses.empty()) {
    path.poses.front().pose.position = start.pose.position;
    GraphNode raw_goal = worldToGrid(goal.pose);
    if (poseOnMap(raw_goal) && costmap_->getCost(raw_goal.x, raw_goal.y) < 253) {
      path.poses.back().pose.position = goal.pose.position;
    }
    double q_norm = std::hypot(goal.pose.orientation.z, goal.pose.orientation.w);
    if (q_norm > 0.1) {
      path.poses.back().pose.orientation = goal.pose.orientation;
    }
  }

  if (smooth_client_->action_server_is_ready()) {
    nav2_msgs::action::SmoothPath::Goal path_smooth;
    path_smooth.path = path;
    path_smooth.check_for_collisions = false;
    path_smooth.smoother_id = "simple_smoother";
    path_smooth.max_smoothing_duration.sec = 10;
    auto future = smooth_client_->async_send_goal(path_smooth);

    if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
      auto goal_handle = future.get();
      if (goal_handle) {
        auto result_future = smooth_client_->async_get_result(goal_handle);
        if (result_future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
          auto result_path = result_future.get();
          if (result_path.code == rclcpp_action::ResultCode::SUCCEEDED) {
            path = result_path.result->path;
          }
        }
      }
    }
  }

  return path;
}

double AStarPlanner::manhattanDistance(const GraphNode &node, const GraphNode &goal_node)
{
    return abs(node.x - goal_node.x) + abs(node.y - goal_node.y);
}

bool AStarPlanner::poseOnMap(const GraphNode & node)
{
    return node.x < static_cast<int>(costmap_->getSizeInCellsX()) && node.x >= 0 &&
        node.y < static_cast<int>(costmap_->getSizeInCellsY()) && node.y >= 0;
}

GraphNode AStarPlanner::worldToGrid(const geometry_msgs::msg::Pose & pose)
{
    int grid_x = static_cast<int>((pose.position.x - costmap_->getOriginX()) / costmap_->getResolution());
    int grid_y = static_cast<int>((pose.position.y - costmap_->getOriginY()) / costmap_->getResolution());
    return GraphNode(grid_x, grid_y);
}

geometry_msgs::msg::Pose AStarPlanner::gridToWorld(const GraphNode & node)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = node.x * costmap_->getResolution() + costmap_->getOriginX();
    pose.position.y = node.y * costmap_->getResolution() + costmap_->getOriginY();
    pose.orientation.w = 1.0;
    return pose;
}

unsigned int AStarPlanner::poseToCell(const GraphNode & node)
{
    return node.y * costmap_->getSizeInCellsX() + node.x;
}

}  // namespace mobile_robot_planning

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mobile_robot_planning::AStarPlanner, nav2_core::GlobalPlanner)
