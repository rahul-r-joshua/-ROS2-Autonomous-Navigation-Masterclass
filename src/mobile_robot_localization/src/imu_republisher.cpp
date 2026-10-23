/*
================================================================================
IMU Data Republisher Node (rclcpp)
--------------------------------------------------------------------------------
Author & Maintainer: Rahul Ramasamy
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/
LinkedIn: https://www.linkedin.com/in/rahul-ramasamy-in/
Email: rahul.r.joshua123@gmail.com

Workshop Edition for ROS 2 Mobile Robotics Workshop.
================================================================================
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

using namespace std::chrono_literals;

rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

void imuCallback(const sensor_msgs::msg::Imu& imu)
{
    sensor_msgs::msg::Imu new_imu;
    new_imu = imu;
    new_imu.header.frame_id = "base_footprint_ekf";
    if (new_imu.orientation_covariance[0] == 0.0) {
        new_imu.orientation_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
    }
    if (new_imu.angular_velocity_covariance[0] == 0.0) {
        new_imu.angular_velocity_covariance = {0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001};
    }
    if (new_imu.linear_acceleration_covariance[0] == 0.0) {
        new_imu.linear_acceleration_covariance = {0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1};
    }
    imu_pub->publish(new_imu);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("imu_republisher_node");
    rclcpp::sleep_for(1s);

    imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu_ekf", 10);
    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, imuCallback);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
