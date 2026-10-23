/*
================================================================================
Extended Kalman Filter Sensor Fusion Implementation
--------------------------------------------------------------------------------
Author & Maintainer: Rahul Ramasamy
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/
LinkedIn: https://www.linkedin.com/in/rahul-ramasamy-in/
Email: rahul.r.joshua123@gmail.com

Workshop Edition for ROS 2 Mobile Robotics Workshop.
================================================================================
*/

#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

class KalmanFilter : public rclcpp::Node
{
public: 
    KalmanFilter(const std::string & name);

private: 
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    double mean_;
    double variance_;
    double imu_angular_z_;
    bool is_first_odom_;
    double last_angular_z_;
    double motion_;

    nav_msgs::msg::Odometry kalman_odom_;

    double motion_variance_;
    double measurement_variance_;

    void measurementUpdate();
    void statePrediction();
    void OdomCallback(const nav_msgs::msg::Odometry & odom);
    void imuCallback(const sensor_msgs::msg::Imu & imu);
};

#endif
