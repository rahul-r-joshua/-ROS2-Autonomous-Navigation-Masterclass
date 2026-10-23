#!/usr/bin/env bash
# ==============================================================================
# 04. Clean Purge and Rebuild Script
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

echo "Purging build/, install/, log/ directories..."
rm -rf build install log

echo "Triggering fresh topological build..."
"${SCRIPT_DIR}/build_all.sh"
