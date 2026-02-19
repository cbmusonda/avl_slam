#!/bin/bash
# ============================================================
#  AVL SLAM Workspace Setup
#  Sensors: Velodyne VLP-16 | Intel RealSense D455 | Xsens IMU
#  ROS 2 Humble | RTAB-Map
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
echo "[1/5] Installing system dependencies..."

sudo apt update && sudo apt install -y \
  ros-$ROS_DISTRO-rtabmap \
  ros-$ROS_DISTRO-velodyne \
  ros-$ROS_DISTRO-velodyne-driver \
  ros-$ROS_DISTRO-velodyne-laserscan \
  ros-$ROS_DISTRO-velodyne-pointcloud \
  ros-$ROS_DISTRO-realsense2-camera \
  ros-$ROS_DISTRO-realsense2-description \
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

# ── 2. Create workspace & clone source packages ─────────────
echo "[2/5] Setting up workspace source directory..."

mkdir -p "$SRC_DIR"
cd "$SRC_DIR"

# rtabmap_ros — ROS 2 wrapper (build from source for latest features)
if [ ! -d "rtabmap_ros" ]; then
  git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git
else
  echo "  rtabmap_ros already cloned, skipping."
fi

# ros2_xsens_mti_driver — DEMCON ROS 2 driver (fork of bluespace-ai)
if [ ! -d "ros2_xsens_mti_driver" ]; then
  git clone https://github.com/DEMCON/ros2_xsens_mti_driver.git
else
  echo "  ros2_xsens_mti_driver already cloned, skipping."
fi

# ── 3. rosdep install ───────────────────────────────────────
echo "[3/5] Running rosdep..."
cd "$WS_DIR"
source /opt/ros/$ROS_DISTRO/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# ── 4. Build ────────────────────────────────────────────────
echo "[4/5] Building workspace..."
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select ros2_xsens_mti_driver rtabmap_ros

# ── 5. Source overlay ───────────────────────────────────────
echo "[5/5] Adding workspace to ~/.bashrc..."
BASHRC_LINE="source $WS_DIR/install/setup.bash"
if ! grep -qF "$BASHRC_LINE" ~/.bashrc; then
  echo "" >> ~/.bashrc
  echo "# AVL SLAM Workspace" >> ~/.bashrc
  echo "$BASHRC_LINE" >> ~/.bashrc
fi

echo ""
echo "✅ Workspace setup complete!"
echo "   Run: source ~/.bashrc"
echo "   Then: ros2 launch avl_slam slam.launch.py"