#!/bin/bash
# ============================================================
#  AVL SLAM Workspace Setup
#  Sensors: Velodyne VLP-16 | ZED X (x2) | Intel RealSense D455 | Xsens IMU
#  ROS 2 Humble | RTAB-Map
#
#  RealSense D455 requires librealsense 2.57.6 built from source
#  with RSUSB backend and firmware downgraded to 5.13.0.50.
#  See docs/realsense_d455_setup.tex for full setup instructions.
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

# ── 2b. Intel RealSense D455 (source build) ──────────────────
# The apt packages do NOT work on JetPack 6 (R36.x, kernel 5.15-tegra).
# Prerequisites (done once, outside this script):
#   1. Build librealsense 2.57.6 from source with RSUSB backend
#   2. Downgrade D455 firmware to 5.13.0.50
#   3. Remove apt packages: sudo apt remove ros-humble-librealsense2
# See docs/realsense_d455_setup.tex for full instructions.
echo "[2b/6] Checking librealsense source install..."
if [ -f "/usr/local/lib/cmake/realsense2/realsense2Config.cmake" ]; then
  echo "  ✅ librealsense 2.x found at /usr/local/lib/cmake/realsense2"
else
  echo "  ⚠️  librealsense not found at /usr/local/lib/cmake/realsense2."
  echo "  RealSense support requires librealsense 2.57.6 built from source."
  echo "  See docs/realsense_d455_setup.tex for build instructions."
  echo "  Continuing without RealSense (ZED X cameras will still work)."
fi

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

# realsense-ros 4.56.4 — built from source against local librealsense 2.57.6
# Do NOT use 4.57.6 (requires RS2_STREAM_SAFETY not in librealsense 2.57.6)
if [ ! -d "realsense-ros" ]; then
  git clone https://github.com/IntelRealSense/realsense-ros.git
  cd realsense-ros
  git checkout 4.56.4
  cd ..
else
  echo "  realsense-ros already cloned, skipping."
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
    -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
  --packages-select \
    ros2_xsens_mti_driver \
    rtabmap_ros \
    zed_wrapper \
    zed_components \
    zed_ros2_interfaces \
    realsense2_camera_msgs \
    realsense2_description \
    realsense2_camera

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
echo ""
echo "   To use Intel RealSense D455 instead of ZED X Left:"
echo "   1. Build librealsense 2.57.6 from source (see docs/realsense_d455_setup.tex)"
echo "   2. Downgrade D455 firmware to 5.13.0.50"
echo "   3. ros2 launch avl_slam slam.launch.py use_realsense:=true"