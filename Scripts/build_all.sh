#!/usr/bin/env bash
# ==============================================================================
# 03. Two-Stage Topological Workspace Build
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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WS_DIR}"

echo "=========================================================="
echo "Stage 1: Building Custom Interfaces (mobile_robot_msgs)..."
echo "=========================================================="
source /opt/ros/humble/setup.bash
colcon build --packages-select mobile_robot_msgs --symlink-install

echo "=========================================================="
echo "Stage 2: Building All Downstream Packages..."
echo "=========================================================="
source "${WS_DIR}/install/setup.bash"
colcon build --symlink-install

echo "=========================================================="
echo "✅ Entire Workspace Built Successfully!"
echo "=========================================================="
