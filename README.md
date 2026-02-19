# AVL SLAM Workspace
**Sensors:** Velodyne VLP-16 · Intel RealSense D455 · Xsens MTi IMU  
**SLAM:** RTAB-Map (LiDAR-ICP odometry + RGB-D loop closure + IMU fusion)  
**ROS 2 Humble**

---

## Workspace Structure

```
avl_slam_ws/
├── setup_workspace.sh          ← run this first
└── src/
    ├── avl_slam/               ← your custom package (configs + launches)
    │   ├── config/
    │   │   ├── rtabmap.yaml    ← RTAB-Map tuning params
    │   │   ├── vlp16.yaml      ← Velodyne conversion settings
    │   │   ├── xsens.yaml      ← Xsens IMU driver settings
    │   │   └── imu_filter.yaml ← Madgwick filter settings
    │   └── launch/
    │       ├── slam.launch.py          ← full SLAM pipeline
    │       └── localization.launch.py  ← localization against saved map
    ├── rtabmap_ros/            ← cloned by setup script
    └── xsens_mti_driver/       ← cloned by setup script
```

---

## Quick Start

### 1. Run setup script (once)
```bash
chmod +x setup_workspace.sh
./setup_workspace.sh
source ~/.bashrc
```

### 2. Hardware checklist before launch

| Sensor | Check |
|--------|-------|
| VLP-16 | Set static IP `192.168.1.201` on your machine; verify with `ping 192.168.1.201` |
| RealSense D455 | `rs-enumerate-devices` should list serial number |
| Xsens IMU | Check port: `ls /dev/ttyUSB*` → update `xsens.yaml` if not `ttyUSB0` |

### 3. Build
```bash
cd ~/avl_slam_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 4. Launch SLAM
```bash
ros2 launch avl_slam slam.launch.py
```

Optional args:
```bash
# Custom Velodyne IP
ros2 launch avl_slam slam.launch.py velodyne_ip:=192.168.1.201

# Specific RealSense serial number
ros2 launch avl_slam slam.launch.py realsense_serial:=_215122252880

# Headless (no RTAB-Map visualizer)
ros2 launch avl_slam slam.launch.py use_rviz:=false
```

### 5. Localization against a saved map
```bash
ros2 launch avl_slam localization.launch.py database_path:=~/.ros/rtabmap.db
```

---

## Topic Map

```
/velodyne_points        ← VLP-16 raw point cloud (sensor_msgs/PointCloud2)
/camera/color/image_raw ← D455 RGB (sensor_msgs/Image)
/camera/color/camera_info
/camera/aligned_depth_to_color/image_raw ← D455 depth aligned to color
/xsens/imu/data         ← raw Xsens IMU (sensor_msgs/Imu)
/imu/filtered           ← Madgwick-filtered IMU → RTAB-Map input

/rtabmap/mapData        ← SLAM graph
/rtabmap/grid_map       ← 2D occupancy grid
/rtabmap/cloud_map      ← 3D point cloud map
/rtabmap/odom           ← LiDAR odometry
/tf                     ← transform tree
```

---

## TF Tree

```
map
 └── odom (published by rtabmap)
      └── base_link
           ├── velodyne     [0, 0, 0.3]  ← adjust to your mount
           ├── camera_link  [0.2, 0, 0.25]
           └── imu_link     [0, 0, 0.1]
```
**Important:** Update static TF offsets in `slam.launch.py` to match your actual sensor positions.

---

## Tuning Guide

### If LiDAR odometry drifts
- Decrease `Icp/VoxelSize` (try `0.1`)
- Increase `Icp/MaxCorrespondenceDistance` (try `0.2`)
- Lower `Rtabmap/DetectionRate` to give more compute time

### If loop closures are missed
- Lower `Rtabmap/LoopThr` (try `0.15`)
- Increase `Vis/MaxFeatures` (try `1000`)

### For indoor campus environments
- Set `Grid/RangeMax: "20.0"` to reduce noise from far returns
- `Grid/CellSize: "0.05"` gives good detail at reasonable memory cost

### Jetson AGX Orin tips
- Use `--cmake-args -DCMAKE_BUILD_TYPE=Release` (already set in setup script)
- Monitor with `tegrastats` during first runs to check thermal throttling
- Consider reducing `Icp/Iterations` to `20` if CPU is bottlenecking

---

## Saving & Exporting the Map

```bash
# Map saves automatically to ~/.ros/rtabmap.db on shutdown

# Export to PCD point cloud
ros2 run rtabmap_util map_assembler  # or use RTAB-Map GUI: File > Export 3D Map

# Export to OctoMap
ros2 topic echo /rtabmap/octomap_full
```

---

## Common Issues

**Velodyne not found:**  
Check your ethernet connection and `ping 192.168.1.201`. Set your PC's IP to `192.168.1.100/24`.

**RealSense driver error:**  
Update firmware: `rs-fw-update -l` then follow prompts. Also try `realsense-viewer` to verify hardware.

**Xsens port permission denied:**  
```bash
sudo chmod 666 /dev/ttyUSB0
# or permanently:
sudo usermod -aG dialout $USER  # logout required
```

**IMU and LiDAR timestamps not syncing:**  
Increase `approx_sync_max_interval` in `slam.launch.py` to `0.05` (50ms).
