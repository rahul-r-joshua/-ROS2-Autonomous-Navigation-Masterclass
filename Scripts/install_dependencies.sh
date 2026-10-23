#!/usr/bin/env bash
# ==============================================================================
# 01. Install System, ROS 2, and Python Dependencies for Workshop 3
# ------------------------------------------------------------------------------
# Author & Maintainer: Rahul Ramasamy
# GitHub: https://github.com/rahul-r-joshua
# Portfolio: https://rahul-r-joshua.github.io/portfolio/
# LinkedIn: https://www.linkedin.com/in/rahul-ramasamy-in/
# Email: rahul.r.joshua123@gmail.com
#
# Workshop Edition for ROS 2 Mobile Robotics Workshop.
# ==============================================================================

set -e

echo "=========================================================="
echo "Installing Dependencies for ROS 2 Autonomous Navigation WS"
echo "=========================================================="

sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-twist-mux \
  ros-humble-joy \
  ros-humble-teleop-twist-joy \
  ros-humble-teleop-twist-keyboard \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-map-server \
  ros-humble-nav2-amcl \
  ros-humble-nav2-controller \
  ros-humble-nav2-planner \
  ros-humble-nav2-behaviors \
  ros-humble-nav2-bt-navigator \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-rviz-plugins \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-behaviortree-cpp-v3 \
  ros-humble-tf2-tools \
  ros-humble-tf-transformations

pip3 install transforms3d pyserial

# Initialize and update rosdep if needed
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init || true
fi
rosdep update || true

echo "✅ All dependencies installed successfully!"
