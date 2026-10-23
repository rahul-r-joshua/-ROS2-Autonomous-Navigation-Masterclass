#include "mobile_robot_bringup/safety_zone_controller.hpp"

namespace mobile_robot_bringup
{

SafetyZoneControllerCpp::SafetyZoneControllerCpp()
: Node("safety_zone_controller"),
  min_obstacle_distance_(std::numeric_limits<double>::infinity()),
  zone_state_("UNKNOWN")
{
  // 1. Declare & Get Parameters
  this->declare_parameter<double>("red_zone_distance", 0.45);
  this->declare_parameter<double>("yellow_zone_distance", 0.90);
  this->declare_parameter<double>("fov_angle_deg", 90.0);
  this->declare_parameter<double>("slowdown_factor", 0.5);
  this->declare_parameter<double>("lidar_x_offset", 0.10);

  red_dist_ = this->get_parameter("red_zone_distance").as_double();
  yellow_dist_ = this->get_parameter("yellow_zone_distance").as_double();
  double fov_deg = this->get_parameter("fov_angle_deg").as_double();
  fov_rad_ = fov_deg * M_PI / 180.0;
  slowdown_factor_ = this->get_parameter("slowdown_factor").as_double();
  lidar_x_offset_ = this->get_parameter("lidar_x_offset").as_double();

  last_cmd_time_ = this->now();

  // 2. Subscribers
  sub_cmd_raw_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_raw", 10,
    std::bind(&SafetyZoneControllerCpp::cmdRawCallback, this, std::placeholders::_1));

  sub_joy_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/joy_vel", 10,
    std::bind(&SafetyZoneControllerCpp::cmdRawCallback, this, std::placeholders::_1));

  // Use SensorDataQoS (Best Effort) for universal compatibility with Gazebo & physical LiDARs (RPLiDAR/YDLidar)
  sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&SafetyZoneControllerCpp::scanCallback, this, std::placeholders::_1));

  sub_safety_stop_ = this->create_subscription<std_msgs::msg::Bool>(
    "/safety_stop", 10,
    std::bind(&SafetyZoneControllerCpp::safetyStopCallback, this, std::placeholders::_1));

  sub_safety_enable_ = this->create_subscription<std_msgs::msg::Bool>(
    "/safety_zone_enable", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg->data != safety_zone_enabled_) {
        safety_zone_enabled_ = msg->data;
        if (safety_zone_enabled_) {
          RCLCPP_INFO(this->get_logger(), "🛡️  Safety Zone ENABLED: Collision avoidance active.");
        } else {
          RCLCPP_WARN(this->get_logger(), "⚠️  Safety Zone DISABLED: Bypass mode active (100%% speed pass-through).");
        }
      }
    });

  // 3. Publishers
  pub_cmd_safe_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/safety_zone_markers", 10);

  // 4. Periodic Timer at 20 Hz (50 ms)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&SafetyZoneControllerCpp::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "============================================================");
  RCLCPP_INFO(this->get_logger(), "🛡️  Safety Zone Controller (C++) Initialized");
  RCLCPP_INFO(this->get_logger(), "   🔴 Red Zone (1x Robot Size):    %.2f m  -> FULL STOP", red_dist_);
  RCLCPP_INFO(this->get_logger(), "   🟡 Yellow Zone (2x Robot Size): %.2f m  -> 2x SLOWDOWN", yellow_dist_);
  RCLCPP_INFO(this->get_logger(), "   🟢 Green Zone:                  > %.2f m -> FULL SPEED (Auto-Resume)", yellow_dist_);
  RCLCPP_INFO(this->get_logger(), "============================================================");
}

void SafetyZoneControllerCpp::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  double closest_dist = std::numeric_limits<double>::infinity();
  double angle = msg->angle_min;
  double half_fov = fov_rad_ / 2.0;

  for (size_t i = 0; i < msg->ranges.size(); ++i, angle += msg->angle_increment) {
    double r = msg->ranges[i];
    // Safeguard: Explicit finite & range sanity checks
    if (!std::isfinite(r) || std::isnan(r) || std::isinf(r)) {
      continue;
    }
    if (r < msg->range_min || r > msg->range_max) {
      continue;
    }

    // Wrap angle to [-pi, pi]
    double norm_angle = std::atan2(std::sin(angle), std::cos(angle));

    // Calculate obstacle point coordinates in robot base_footprint frame
    double px = lidar_x_offset_ + r * std::cos(norm_angle);
    double py = r * std::sin(norm_angle);
    double d_robot = std::sqrt(px * px + py * py);
    double angle_from_base = std::atan2(py, px);

    // Check if within forward Field of View relative to robot base
    if (px > 0.0 && std::abs(angle_from_base) <= half_fov) {
      if (d_robot < closest_dist) {
        closest_dist = d_robot;
      }
    }
  }

  min_obstacle_distance_ = closest_dist;
}

void SafetyZoneControllerCpp::safetyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data != manual_safety_active_) {
    manual_safety_active_ = msg->data;
    has_fresh_raw_cmd_ = false;
    if (manual_safety_active_) {
      RCLCPP_WARN(this->get_logger(), "🛑 Safety ACTIVE (/safety_stop: true) -> Robot stopped & movement blocked!");
    } else {
      RCLCPP_INFO(this->get_logger(), "🟢 Safety INACTIVE (/safety_stop: false) -> Movement unlocked. Waiting for fresh /cmd_vel_raw.");
    }
  }
}

void SafetyZoneControllerCpp::cmdRawCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  raw_cmd_ = *msg;
  last_cmd_time_ = this->now();
  has_fresh_raw_cmd_ = true;
}

void SafetyZoneControllerCpp::controlLoop()
{
  // 1. Evaluate safety zone state
  std::string previous_state = zone_state_;
  if (min_obstacle_distance_ <= red_dist_) {
    zone_state_ = "RED";
  } else if (min_obstacle_distance_ <= yellow_dist_) {
    zone_state_ = "YELLOW";
  } else {
    zone_state_ = "GREEN";
  }

  if (zone_state_ != previous_state) {
    if (zone_state_ == "RED") {
      RCLCPP_WARN(this->get_logger(), "🔴 RED ZONE: Obstacle at %.2f m -> EMERGENCY STOP ACTIVATED!", min_obstacle_distance_);
    } else if (zone_state_ == "YELLOW") {
      RCLCPP_INFO(this->get_logger(), "🟡 YELLOW ZONE: Obstacle at %.2f m -> Velocity reduced by 2x.", min_obstacle_distance_);
    } else {
      RCLCPP_INFO(this->get_logger(), "🟢 GREEN ZONE: Path Clear (%.2f m) -> Full speed restored.", min_obstacle_distance_);
    }
  }

  // 2. Compute safe commanded twist
  geometry_msgs::msg::Twist safe_cmd;

  if (manual_safety_active_ || !has_fresh_raw_cmd_) {
    // Safety protection active OR waiting for fresh velocity after release: output zero
    safe_cmd.linear.x = 0.0;
    safe_cmd.angular.z = 0.0;
  } else {
    // Check watchdog timeout on raw input (0.5s)
    double dt = (this->now() - last_cmd_time_).seconds();
    if (dt < 0.5) {
      if (!safety_zone_enabled_) {
        // Bypass mode: direct pass-through at full speed
        safe_cmd = raw_cmd_;
      } else if (zone_state_ == "RED") {
        // Red Zone: FULL EMERGENCY STOP (Zero linear & zero angular motion)
        safe_cmd.linear.x = 0.0;
        safe_cmd.angular.z = 0.0;
      } else if (zone_state_ == "YELLOW") {
        safe_cmd.linear.x = raw_cmd_.linear.x * slowdown_factor_;
        safe_cmd.angular.z = raw_cmd_.angular.z;
      } else {
        safe_cmd = raw_cmd_;
      }
    }
  }

  pub_cmd_safe_->publish(safe_cmd);

  // 3. Publish RViz visualization markers
  publishMarkers();
}

void SafetyZoneControllerCpp::publishMarkers()
{
  visualization_msgs::msg::MarkerArray markers;

  // Yellow warning cylinder boundary
  visualization_msgs::msg::Marker yellow_marker;
  yellow_marker.header.frame_id = "base_footprint";
  yellow_marker.header.stamp = rclcpp::Time(0);
  yellow_marker.ns = "safety_zones";
  yellow_marker.id = 0;
  yellow_marker.type = visualization_msgs::msg::Marker::CYLINDER;
  yellow_marker.action = visualization_msgs::msg::Marker::ADD;
  yellow_marker.pose.position.x = 0.0;
  yellow_marker.pose.position.y = 0.0;
  yellow_marker.pose.position.z = 0.005;
  yellow_marker.pose.orientation.w = 1.0;
  yellow_marker.scale.x = yellow_dist_ * 2.0;
  yellow_marker.scale.y = yellow_dist_ * 2.0;
  yellow_marker.scale.z = 0.01;
  yellow_marker.color.r = 1.0f;
  yellow_marker.color.g = 0.85f;
  yellow_marker.color.b = 0.0f;
  yellow_marker.color.a = !safety_zone_enabled_ ? 0.04f : ((zone_state_ == "YELLOW") ? 0.35f : 0.12f);
  markers.markers.push_back(yellow_marker);

  // Red emergency stop cylinder boundary
  visualization_msgs::msg::Marker red_marker;
  red_marker.header.frame_id = "base_footprint";
  red_marker.header.stamp = rclcpp::Time(0);
  red_marker.ns = "safety_zones";
  red_marker.id = 1;
  red_marker.type = visualization_msgs::msg::Marker::CYLINDER;
  red_marker.action = visualization_msgs::msg::Marker::ADD;
  red_marker.pose.position.x = 0.0;
  red_marker.pose.position.y = 0.0;
  red_marker.pose.position.z = 0.01;
  red_marker.pose.orientation.w = 1.0;
  red_marker.scale.x = red_dist_ * 2.0;
  red_marker.scale.y = red_dist_ * 2.0;
  red_marker.scale.z = 0.015;
  red_marker.color.r = 1.0f;
  red_marker.color.g = 0.0f;
  red_marker.color.b = 0.0f;
  red_marker.color.a = !safety_zone_enabled_ ? 0.04f : ((zone_state_ == "RED") ? 0.45f : 0.15f);
  markers.markers.push_back(red_marker);

  pub_markers_->publish(markers);
}

}  // namespace mobile_robot_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mobile_robot_bringup::SafetyZoneControllerCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
