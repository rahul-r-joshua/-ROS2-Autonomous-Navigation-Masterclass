/*
================================================================================
Probabilistic Odometry Motion Model Simulation
--------------------------------------------------------------------------------
Author & Maintainer: Rahul Ramasamy
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/
LinkedIn: https://www.linkedin.com/in/rahul-ramasamy-in/
Email: rahul.r.joshua123@gmail.com

Workshop Edition for ROS 2 Mobile Robotics Workshop.
================================================================================
*/

#ifndef ODOMETRY_MOTION_MODEL_HPP
#define ODOMETRY_MOTION_MODEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

class OdometryMotionModel : public rclcpp::Node
{
public: 
    OdometryMotionModel(const std::string & name);

private: 
    void OdomCallback(const nav_msgs::msg::Odometry &);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;

    geometry_msgs::msg::PoseArray samples_;
    double alpha1_;
    double alpha2_;
    double alpha3_;
    double alpha4_;
    int nr_samples_;

    double last_odom_x_;
    double last_odom_y_;
    double last_odom_theta_;
    bool is_first_odom_;
};

#endif
