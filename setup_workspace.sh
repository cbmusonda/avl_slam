#!/bin/bash
# ============================================================
#  AVL SLAM Workspace Setup
#  Sensors: Velodyne VLP-16 | ZED X (x2) | Xsens IMU
#  ROS 2 Humble | RTAB-Map
#
#  NOTE: Intel RealSense D455 install is commented out pending
#        ROS connection fix. Re-enable section [2b] when ready.
# ============================================================

set -e

ROS_DISTRO=${ROS_DISTRO:-humble}
WS_DIR="$HOME/avl_slam_ws"
SRC_DIR="$WS_DIR/src"

echo "================================================"
echo "  AVL SLAM Workspace Setup"
echo "  ROS Distro: $ROS_DISTRO"
echo "  Workspace:  $WS_DIR"
echo "================================================"

# ── 1. System dependencies ──────────────────────────────────
echo "[1/6] Installing system dependencies..."

sudo apt update && sudo apt install -y \
  ros-$ROS_DISTRO-rtabmap \
  ros-$ROS_DISTRO-velodyne \
  ros-$ROS_DISTRO-velodyne-driver \
  ros-$ROS_DISTRO-velodyne-laserscan \
  ros-$ROS_DISTRO-velodyne-pointcloud \
  ros-$ROS_DISTRO-imu-tools \
  ros-$ROS_DISTRO-imu-filter-madgwick \
  ros-$ROS_DISTRO-robot-localization \
  ros-$ROS_DISTRO-tf2-ros \
  ros-$ROS_DISTRO-tf2-tools \
  ros-$ROS_DISTRO-pcl-ros \
  ros-$ROS_DISTRO-pcl-conversions \
  ros-$ROS_DISTRO-nav2-map-server \
  ros-$ROS_DISTRO-rviz2 \
  python3-colcon-common-extensions \
  python3-rosdep

# ── 2a. ZED SDK (requires manual install if not present) ────
echo "[2a/6] Checking ZED SDK..."
if [ ! -f "/usr/local/zed/tools/ZED_Diagnostic" ]; then
  echo "  ⚠️  ZED SDK not found."
  echo "  Please download and install it from:"
  echo "  https://www.stereolabs.com/developers/release"
  echo "  Then re-run this script."
  exit 1
else
  echo "  ✅ ZED SDK found."
fi

# ── 2b. Intel RealSense — COMMENTED OUT (pending ROS fix) ───
# TODO: Re-enable when RealSense ROS connection is resolved.
#
# sudo apt install -y \
#   ros-$ROS_DISTRO-realsense2-camera \
#   ros-$ROS_DISTRO-realsense2-description

# ── 3. Clone source packages ─────────────────────────────────
echo "[3/6] Setting up workspace source directory..."

mkdir -p "$SRC_DIR"
cd "$SRC_DIR"

# rtabmap_ros — ROS 2 wrapper (build from source for latest features)
if [ ! -d "rtabmap_ros" ]; then
  git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git
else
  echo "  rtabmap_ros already cloned, skipping."
fi

# ros2_xsens_mti_driver — DEMCON ROS 2 driver
if [ ! -d "ros2_xsens_mti_driver" ]; then
  git clone https://github.com/DEMCON/ros2_xsens_mti_driver.git
else
  echo "  ros2_xsens_mti_driver already cloned, skipping."
fi

# zed-ros2-wrapper — Stereolabs official ROS 2 wrapper
if [ ! -d "zed-ros2-wrapper" ]; then
  git clone --recurse-submodules https://github.com/stereolabs/zed-ros2-wrapper.git
else
  echo "  zed-ros2-wrapper already cloned, skipping."
fi

# zed-ros2-examples (optional but useful for testing)
if [ ! -d "zed-ros2-examples" ]; then
  git clone https://github.com/stereolabs/zed-ros2-examples.git
else
  echo "  zed-ros2-examples already cloned, skipping."
fi

# ── 4. rosdep install ───────────────────────────────────────
echo "[4/6] Running rosdep..."
cd "$WS_DIR"
source /opt/ros/$ROS_DISTRO/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# ── 5. Build ────────────────────────────────────────────────
echo "[5/6] Building workspace..."
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select \
    ros2_xsens_mti_driver \
    rtabmap_ros \
    zed_wrapper \
    zed_components \
    zed_ros2_interfaces

# ── 6. Source overlay ───────────────────────────────────────
echo "[6/6] Adding workspace to ~/.bashrc..."
BASHRC_LINE="source $WS_DIR/install/setup.bash"
if ! grep -qF "$BASHRC_LINE" ~/.bashrc; then
  echo "" >> ~/.bashrc
  echo "# AVL SLAM Workspace" >> ~/.bashrc
  echo "$BASHRC_LINE" >> ~/.bashrc
fi

echo ""
echo "✅ Workspace setup complete!"
echo ""
echo "   Next steps:"
echo "   1. source ~/.bashrc"
echo "   2. Set your ZED X serial numbers in:"
echo "        src/avl_slam/config/zed_left.yaml  (serial_number field)"
echo "        src/avl_slam/config/zed_right.yaml (serial_number field)"
echo "   3. Verify both cameras: /usr/local/zed/tools/ZED_Explorer"
echo "   4. ros2 launch avl_slam slam.launch.py"