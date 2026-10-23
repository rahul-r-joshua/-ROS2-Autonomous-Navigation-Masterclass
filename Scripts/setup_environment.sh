#!/usr/bin/env bash
# ==============================================================================
# 02. Workspace Environment Setup and DDS Configuration
# ------------------------------------------------------------------------------
# Author & Maintainer: Rahul Ramasamy
# GitHub: https://github.com/rahul-r-joshua
# Portfolio: https://rahul-r-joshua.github.io/portfolio/
# LinkedIn: https://www.linkedin.com/in/rahul-ramasamy-in/
# Email: rahul.r.joshua123@gmail.com
#
# Workshop Edition for ROS 2 Mobile Robotics Workshop.
# ==============================================================================

# Sourcing standard ROS 2 Humble installation
source /opt/ros/humble/setup.bash

# Configure isolated DDS Domain ID (default: 30)
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0

# Source workspace install overlay if present
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  source "${WS_DIR}/install/setup.bash"
  echo "✅ Sourced ${WS_DIR}/install/setup.bash (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})"
else
  echo "⚠️ Workspace not built yet. Run ./Scripts/build_all.sh first."
fi
