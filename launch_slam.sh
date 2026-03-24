#!/bin/bash
# AVL SLAM Launch Wrapper
# Run this instead of ros2 launch directly.
# Handles pre-launch cleanup and camera initialization every time.

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[avl_slam] Killing any existing ROS/SLAM processes..."
pkill -9 -f "ros2|rtabmap|rviz2|zed_container|velodyne|xsens_mti|icp_odometry|static_transform_publisher" 2>/dev/null || true
sleep 2

echo "[avl_slam] Restarting nvargus-daemon (required for ZED X cameras)..."
sudo systemctl restart nvargus-daemon
sleep 4

echo "[avl_slam] Sourcing ROS 2 and workspace..."
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

echo "[avl_slam] Launching SLAM..."
exec ros2 launch avl_slam slam.launch.py "$@"
