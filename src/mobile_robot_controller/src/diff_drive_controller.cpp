/**
 * ============================================================================
 * Mobile Robot Differential Drive Controller (C++)
 * ----------------------------------------------------------------------------
 * Author: Rahul Ramasamy
 * Email: rahul.r.joshua123@gmail.com
 * GitHub: https://github.com/rahul-r-joshua
 * Portfolio: https://rahul-r-joshua.github.io/portfolio/
 *
 * Designed for ROS 2 Mobile Robotics Workshop.
 * This node performs:
 *  1. Inverse Kinematics: Translates /cmd_vel into Left/Right wheel angular speeds.
 *  2. Forward Kinematics (Dead-Reckoning Odometry): Computes robot pose (x, y, theta).
 *  3. Joint State Publishing: Publishes wheel positions to animate 4 wheels in RViz.
 *  4. TF Broadcasting: Broadcasts dynamic transformation 'odom' -> 'base_footprint'.
 *  5. Hardware Command Publishing: Publishes wheel speeds for Arduino firmware.
 * ============================================================================
 */

#include "mobile_robot_controller/diff_drive_controller.hpp"

using namespace std::chrono_literals;

DiffDriveControllerCpp::DiffDriveControllerCpp()
: Node("diff_drive_controller_cpp")
{
  // -------------------------------------------------------------------------
  // 1. Declare and Read ROS 2 Parameters
  // -------------------------------------------------------------------------
  this->declare_parameter<double>("wheel_radius", 0.05);         // meters
  this->declare_parameter<double>("wheel_separation", 0.457);    // meters (track width calibrated)
  this->declare_parameter<double>("publish_rate", 50.0);         // Hz
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("base_frame_id", "base_footprint");
  this->declare_parameter<int>("encoder_cpr", 330);
  this->declare_parameter<bool>("publish_joint_states", false);
  this->declare_parameter<double>("cmd_vel_timeout", 0.5);

  r_ = this->get_parameter("wheel_radius").as_double();
  L_ = this->get_parameter("wheel_separation").as_double();
  rate_ = this->get_parameter("publish_rate").as_double();
  odom_frame_ = this->get_parameter("odom_frame_id").as_string();
  base_frame_ = this->get_parameter("base_frame_id").as_string();
  cpr_ = this->get_parameter("encoder_cpr").as_int();
  publish_joint_states_ = this->get_parameter("publish_joint_states").as_bool();
  cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();

  // -------------------------------------------------------------------------
  // 2. Initialize Variables
  // -------------------------------------------------------------------------
  cmd_linear_x_ = 0.0;
  cmd_angular_z_ = 0.0;
  wheel_speed_left_ = 0.0;
  wheel_speed_right_ = 0.0;
  left_wheel_pos_ = 0.0;
  right_wheel_pos_ = 0.0;
  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;
  last_time_ = this->now();
  last_cmd_vel_time_ = this->now();
  has_cmd_vel_ = false;
  stop_count_ = 0;

  has_encoder_data_ = false;
  has_prev_ticks_ = false;
  prev_left_ticks_ = 0;
  prev_right_ticks_ = 0;
  has_joint_feedback_ = false;
  has_prev_joint_pos_ = false;
  prev_left_joint_pos_ = 0.0;
  prev_right_joint_pos_ = 0.0;
  delta_s_left_ = 0.0;
  delta_s_right_ = 0.0;

  // -------------------------------------------------------------------------
  // 3. Subscribers, Publishers & TF Broadcaster
  // -------------------------------------------------------------------------
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&DiffDriveControllerCpp::cmdVelCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  encoder_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
    "/wheel_encoder_ticks", 10,
    std::bind(&DiffDriveControllerCpp::encoderCallback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&DiffDriveControllerCpp::jointStateCallback, this, std::placeholders::_1));

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  wheel_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_speed_commands", 10);
  wheel_cmd_gazebo_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_group_velocity_controller/commands", 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Periodic Update Timer (using simulation-aware ROS clock timer for smooth TF sync)
  auto timer_period = rclcpp::Duration::from_seconds(1.0 / rate_);
  timer_ = rclcpp::create_timer(
    this,
    this->get_clock(),
    timer_period,
    std::bind(&DiffDriveControllerCpp::updateLoop, this));

  RCLCPP_INFO(this->get_logger(), "============================================================");
  RCLCPP_INFO(this->get_logger(), "🟢 [ACTIVE] C++ Diff Drive Kinematics Controller Activated!");
  RCLCPP_INFO(this->get_logger(), "   Wheel Radius:     %.3f m", r_);
  RCLCPP_INFO(this->get_logger(), "   Wheel Separation: %.3f m", L_);
  RCLCPP_INFO(this->get_logger(), "   Update Frequency: %.1f Hz", rate_);
  RCLCPP_INFO(this->get_logger(), "============================================================");
}

void DiffDriveControllerCpp::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (std::abs(msg->linear.x) < 1e-4 && std::abs(msg->angular.z) < 1e-4) {
    if (!has_cmd_vel_) {
      return;
    }
    cmd_linear_x_ = 0.0;
    cmd_angular_z_ = 0.0;
    wheel_speed_left_ = 0.0;
    wheel_speed_right_ = 0.0;
    has_cmd_vel_ = false;
    return;
  }
  last_cmd_vel_time_ = this->now();
  has_cmd_vel_ = true;

  cmd_linear_x_ = msg->linear.x;
  cmd_angular_z_ = msg->angular.z;

  // -------------------------------------------------------------------------
  // INVERSE KINEMATICS:
  // V_left  = V - (W * L / 2)
  // V_right = V + (W * L / 2)
  // W_left  = V_left / R
  // W_right = V_right / R
  // -------------------------------------------------------------------------
  double v_left = cmd_linear_x_ - (cmd_angular_z_ * L_ / 2.0);
  double v_right = cmd_linear_x_ + (cmd_angular_z_ * L_ / 2.0);

  wheel_speed_left_ = v_left / r_;
  wheel_speed_right_ = v_right / r_;
}

void DiffDriveControllerCpp::encoderCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2) {
    return;
  }
  int32_t left_ticks = msg->data[0];
  int32_t right_ticks = msg->data[1];

  if (has_prev_ticks_) {
    int32_t d_left = left_ticks - prev_left_ticks_;
    int32_t d_right = right_ticks - prev_right_ticks_;

    double meters_per_tick = (2.0 * M_PI * r_) / static_cast<double>(cpr_);
    delta_s_left_ = d_left * meters_per_tick;
    delta_s_right_ = d_right * meters_per_tick;
    has_encoder_data_ = true;
  }

  prev_left_ticks_ = left_ticks;
  prev_right_ticks_ = right_ticks;
  has_prev_ticks_ = true;
}

void DiffDriveControllerCpp::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  double left_pos = 0.0;
  double right_pos = 0.0;
  bool found_left = false;
  bool found_right = false;

  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "front_left_wheel_joint" || msg->name[i] == "rear_left_wheel_joint") {
      left_pos = msg->position[i];
      found_left = true;
    } else if (msg->name[i] == "front_right_wheel_joint" || msg->name[i] == "rear_right_wheel_joint") {
      right_pos = msg->position[i];
      found_right = true;
    }
  }

  if (found_left && found_right) {
    if (has_prev_joint_pos_) {
      delta_s_left_ += (left_pos - prev_left_joint_pos_) * r_;
      delta_s_right_ += (right_pos - prev_right_joint_pos_) * r_;
      has_joint_feedback_ = true;
    }
    prev_left_joint_pos_ = left_pos;
    prev_right_joint_pos_ = right_pos;
    has_prev_joint_pos_ = true;
  }
}

void DiffDriveControllerCpp::updateLoop()
{
  rclcpp::Time current_time = this->now();

  // -------------------------------------------------------------------------
  // Safety Watchdog: If /cmd_vel publisher stops, auto-stop robot after 0.5s
  // -------------------------------------------------------------------------
  if (has_cmd_vel_ && (current_time - last_cmd_vel_time_).seconds() > cmd_vel_timeout_) {
    cmd_linear_x_ = 0.0;
    cmd_angular_z_ = 0.0;
    wheel_speed_left_ = 0.0;
    wheel_speed_right_ = 0.0;
    has_cmd_vel_ = false;
    stop_count_ = 25;  // Actively send zero twists for 0.5s at 50Hz to halt simulation
  }

  if (stop_count_ > 0) {
    stop_count_--;
    geometry_msgs::msg::Twist stop_twist;
    cmd_vel_pub_->publish(stop_twist);

    std_msgs::msg::Float64MultiArray stop_wheel_cmds;
    stop_wheel_cmds.data = {0.0, 0.0, 0.0, 0.0};
    wheel_cmd_gazebo_pub_->publish(stop_wheel_cmds);
  }

  double dt = (current_time - last_time_).seconds();
  if (dt <= 0.0) {
    return;
  }
  if (dt > 1.0) {
    last_time_ = current_time;
    return;
  }
  last_time_ = current_time;

  // -------------------------------------------------------------------------
  // FORWARD KINEMATICS & POSE INTEGRATION:
  // -------------------------------------------------------------------------
  double delta_s = 0.0;
  double delta_theta = 0.0;
  double v_robot = 0.0;
  double w_robot = 0.0;

  if (has_encoder_data_ && (std::abs(delta_s_left_) > 1e-6 || std::abs(delta_s_right_) > 1e-6)) {
    delta_s = (delta_s_right_ + delta_s_left_) / 2.0;
    delta_theta = (delta_s_right_ - delta_s_left_) / L_;
    delta_s_left_ = 0.0;
    delta_s_right_ = 0.0;

    v_robot = delta_s / dt;
    w_robot = delta_theta / dt;
  } else {
    double v_left_linear = wheel_speed_left_ * r_;
    double v_right_linear = wheel_speed_right_ * r_;

    v_robot = (v_right_linear + v_left_linear) / 2.0;
    w_robot = (v_right_linear - v_left_linear) / L_;

    delta_s = v_robot * dt;
    delta_theta = w_robot * dt;
  }

  // Euler Integration (Midpoint Runge-Kutta 2nd Order)
  x_ += delta_s * std::cos(theta_ + delta_theta / 2.0);
  y_ += delta_s * std::sin(theta_ + delta_theta / 2.0);
  theta_ += delta_theta;

  // Normalize theta to [-pi, pi]
  theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

  // Integrate wheel rotations for RViz animation
  left_wheel_pos_ += wheel_speed_left_ * dt;
  right_wheel_pos_ += wheel_speed_right_ * dt;

  // Quaternion from yaw
  double qz = std::sin(theta_ / 2.0);
  double qw = std::cos(theta_ / 2.0);

  // -------------------------------------------------------------------------
  // 1. Publish TF: 'odom' -> 'base_footprint'
  // -------------------------------------------------------------------------
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = current_time;
  t.header.frame_id = odom_frame_;
  t.child_frame_id = base_frame_;

  t.transform.translation.x = x_;
  t.transform.translation.y = y_;
  t.transform.translation.z = 0.0;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = qz;
  t.transform.rotation.w = qw;

  tf_broadcaster_->sendTransform(t);

  // -------------------------------------------------------------------------
  // 2. Publish /odom Topic
  // -------------------------------------------------------------------------
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;

  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = qz;
  odom.pose.pose.orientation.w = qw;

  // Set covariance to prevent oversized RViz covariance bubbles
  odom.pose.covariance = {
    0.001, 0.0,   0.0, 0.0, 0.0, 0.0,
    0.0,   0.001, 0.0, 0.0, 0.0, 0.0,
    0.0,   0.0,   1e6, 0.0, 0.0, 0.0,
    0.0,   0.0,   0.0, 1e6, 0.0, 0.0,
    0.0,   0.0,   0.0, 0.0, 1e6, 0.0,
    0.0,   0.0,   0.0, 0.0, 0.0, 0.01
  };

  odom.twist.twist.linear.x = v_robot;
  odom.twist.twist.angular.z = w_robot;
  odom.twist.covariance = odom.pose.covariance;

  odom_pub_->publish(odom);

  // -------------------------------------------------------------------------
  // 3. Publish /joint_states Topic (all 4 wheels)
  // -------------------------------------------------------------------------
  sensor_msgs::msg::JointState js;
  js.header.stamp = current_time;
  js.name = {
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint"
  };
  js.position = {left_wheel_pos_, right_wheel_pos_, left_wheel_pos_, right_wheel_pos_};
  js.velocity = {wheel_speed_left_, wheel_speed_right_, wheel_speed_left_, wheel_speed_right_};

  if (publish_joint_states_) {
    joint_state_pub_->publish(js);
  }

  // -------------------------------------------------------------------------
  // 4. Publish /wheel_speed_commands (for Arduino firmware)
  // -------------------------------------------------------------------------
  std_msgs::msg::Float32MultiArray wheel_cmds;
  wheel_cmds.data = {static_cast<float>(wheel_speed_left_), static_cast<float>(wheel_speed_right_)};
  wheel_cmd_pub_->publish(wheel_cmds);

  // 5. Publish to Gazebo actuator controller (/joint_group_velocity_controller/commands)
  std_msgs::msg::Float64MultiArray gazebo_wheel_cmds;
  gazebo_wheel_cmds.data = {wheel_speed_left_, wheel_speed_right_, wheel_speed_left_, wheel_speed_right_};
  wheel_cmd_gazebo_pub_->publish(gazebo_wheel_cmds);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DiffDriveControllerCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
