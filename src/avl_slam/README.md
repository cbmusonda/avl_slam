# AVL SLAM

LiDAR-inertial-visual SLAM system for the AVL autonomous vehicle platform, running on a **Jetson Orin AGX** (JetPack 6, ROS 2 Humble).

## Sensor Suite

| Sensor | Model | Interface | Topic Namespace |
|---|---|---|---|
| LiDAR | Velodyne VLP-16 | Ethernet (`192.168.13.11:2368`) | `/velodyne_points` |
| Camera (front) | Intel RealSense D455 | USB (RSUSB backend) | `/camera/camera/...` |
| Camera (left) | ZED X | GMSL (serial `43779087`) | `/zed_left/zed_node/...` |
| Camera (right) | ZED X | GMSL (serial `47753729`) | `/zed_right/zed_node/...` |
| Camera (back) | ZED X | GMSL (serial `49910017`) | `/zed_back/zed_node/...` |
| IMU | Xsens MTi-680G | USB (`/dev/ttyUSB0`) | `/imu/data` |

## Architecture

```
Velodyne VLP-16
      │
      ▼
icp_odometry  ──────────────────────────────────────────────────────┐
      │                                                              │
      │   RealSense D455 ──► rgbd_sync_realsense ──► rgbd_image0    │
      │   ZED X Left     ──► rgbd_sync_zed_left  ──► rgbd_image1    │
      │   ZED X Right    ──► rgbd_sync_zed_right ──► rgbd_image2    │
      │   ZED X Back     ──► rgbd_sync_zed_back  ──► rgbd_image3    │
      │                                                              ▼
      └──────────────────────────────────────────────────────► rtabmap
                                                                     │
                                                             Xsens IMU /imu/data
```

- **Odometry**: LiDAR ICP (`rtabmap_odom/icp_odometry`)
- **SLAM**: RTAB-Map with 4-camera RGB-D multicam fusion (`rgbd_cameras: 4`)
- **Features**: SURF (compatible with existing database)
- **GPU**: All ZED cameras pinned to GPU 0; Jetson-tuned RTAB-Map parameters

## Quick Start

```bash
# Source workspace
source /opt/ros/humble/setup.bash
source ~/avl_slam_ws/install/setup.bash

# Launch full system (all 4 cameras + LiDAR + IMU)
ros2 launch avl_slam slam.launch.py
```

The map is saved to `~/.ros/rtabmap.db` automatically.

## Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `use_realsense` | `true` | Enable RealSense D455 |
| `use_multicamera` | `true` | Fuse all cameras in one RTAB-Map graph |
| `use_rviz` | `true` | Open RViz2 + rtabmap_viz |
| `use_xsens` | `true` | Launch Xsens IMU driver |
| `use_zed_left` | `true` | Launch ZED X Left |
| `use_zed_right` | `true` | Launch ZED X Right |
| `use_zed_back` | `true` | Launch ZED X Back |
| `database_path` | `~/.ros/rtabmap.db` | RTAB-Map database path |
| `localization` | `false` | `true` = localization only (no new nodes) |
| `velodyne_ip` | `192.168.13.11` | Velodyne LiDAR IP address |

### Examples

```bash
# ZED cameras only (no RealSense)
ros2 launch avl_slam slam.launch.py use_realsense:=false

# Localization against a saved map
ros2 launch avl_slam slam.launch.py localization:=true

# Custom database path
ros2 launch avl_slam slam.launch.py database_path:=/path/to/my_map.db

# Single camera fallback (ZED right only)
ros2 launch avl_slam slam.launch.py use_multicamera:=false use_realsense:=false
```

## View a Saved Map

```bash
rtabmap-databaseViewer ~/.ros/rtabmap.db
```

## Configuration Files

| File | Description |
|---|---|
| `config/rtabmap.yaml` | RTAB-Map SLAM parameters (ICP, loop closure, grid map) |
| `config/xsens.yaml` | Xsens MTi-680G IMU driver parameters |
| `config/zed_left.yaml` | ZED X Left camera parameters |
| `config/zed_right.yaml` | ZED X Right camera parameters |
| `config/zed_back.yaml` | ZED X Back camera parameters |
| `config/vlp16.yaml` | Velodyne VLP-16 point cloud conversion parameters |
| `config/slam.rviz` | RViz2 visualization config |

## RealSense D455 Prerequisites

The RealSense requires a specific build setup on JetPack 6:

- **librealsense**: v2.57.6 built from source with RSUSB backend (`-DFORCE_RSUSB_BACKEND=ON`, `-DBUILD_ACCELERATE_GPU_WITH_GLSL=OFF`)
- **Firmware**: D455 downgraded to `5.13.0.50`
- **Remove**: `apt remove ros-humble-librealsense2` (conflicts with source build)
- **realsense-ros**: v4.56.4 built from source in this workspace
- **LD_LIBRARY_PATH**: `/usr/local/lib` must be in path (add to `~/.bashrc`)

See `docs/realsense_d455_setup.tex` for full setup instructions.

## Static TF Tree

```
base_link
├── velodyne          (x=0.75, y=0.0,  z=0.3,  yaw=0°)
├── imu_link          (x=0.7,  y=0.0,  z=0.0,  yaw=0°)
├── zed_left_camera_center   (x=-0.6, y=0.35, z=0.6, yaw=+90°)
├── zed_right_camera_center  (x=0.6,  y=0.35, z=0.6, yaw=-90°)
├── zed_back_camera_center   (x=-0.75, y=0.0, z=0.6, yaw=180°)
└── camera_link       (x=0.35, y=0.0,  z=0.75, yaw=0°)  ← RealSense only
```

## RTAB-Map Parameters (GPU Profile)

| Parameter | Value | Notes |
|---|---|---|
| `Icp/Iterations` | 20 | Reduced from 30 for GPU throughput |
| `Vis/MaxFeatures` | 400 | Reduced from 600 for GPU throughput |
| `Optimizer/Iterations` | 60 | Reduced from 100 for GPU throughput |
| `Rtabmap/DetectionRate` | 1 Hz | Loop closure check rate |
| `Grid/CellSize` | 0.05 m | Occupancy grid resolution |
| `RGBD/LinearUpdate` | 0.1 m | Min travel before new map node |
