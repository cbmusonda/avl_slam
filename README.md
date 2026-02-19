# AVL SLAM Workspace

**Sensors:** Velodyne VLP-16 · ZED X (Left) · ZED X (Right) · Xsens MTi IMU
**SLAM:** RTAB-Map (LiDAR-ICP odometry + RGB-D loop closure + IMU fusion)
**ROS 2 Humble**

> **Note:** Intel RealSense D455 is temporarily disabled pending ROS connection fix.
> All RealSense code is preserved and commented out — see `slam.launch.py` and `setup_workspace.sh`.

---

## Workspace Structure

```
avl_slam_ws/
├── setup_workspace.sh              ← run this first
└── src/
    ├── avl_slam/                   ← custom package (configs + launches)
    │   ├── config/
    │   │   ├── rtabmap.yaml        ← RTAB-Map tuning params
    │   │   ├── vlp16.yaml          ← Velodyne conversion settings
    │   │   ├── zed_left.yaml       ← ZED X left camera config
    │   │   ├── zed_right.yaml      ← ZED X right camera config
    │   │   ├── xsens.yaml          ← Xsens IMU driver settings
    │   │   └── imu_filter.yaml     ← Madgwick filter settings
    │   └── launch/
    │       ├── slam.launch.py          ← full SLAM pipeline
    │       └── localization.launch.py  ← localization against saved map
    ├── rtabmap/                    ← cloned from source (required: apt version too old)
    ├── rtabmap_ros/                ← cloned from source (required: apt version too old)
    ├── zed-ros2-wrapper/           ← cloned by setup script
    └── xsens_mti_driver/           ← cloned by setup script
```

---

## Quick Start

### 1. Run setup script (once)

```bash
chmod +x setup_workspace.sh
./setup_workspace.sh
source ~/.bashrc
```

### 2. Serial numbers

Your ZED X serial numbers are:
- **Left camera (zed_left):** `43779087`
- **Right camera (zed_right):** `47753729`

These are already set in `slam.launch.py`. To verify or change them:
```bash
grep "serial_number" ~/avl_slam_ws/src/avl_slam/launch/slam.launch.py
```

### 3. Hardware checklist before launch

| Sensor | Check |
|---|---|
| VLP-16 | Set static IP `192.168.13.11` on your machine; verify with `ping 192.168.13.11` |
| ZED X Left | SN: 43779087 — GMSL port 7 (`/dev/i2c-10`) |
| ZED X Right | SN: 47753729 — GMSL port 6 (`/dev/i2c-9`) |
| Xsens IMU | Check port: `ls /dev/ttyUSB*` → update `xsens.yaml` if not `ttyUSB0` |
| GMSL daemon | `sudo systemctl status nvargus-daemon` — must be **active (running)** |

### 4. Build

> Always deactivate conda before building to avoid Python conflicts:
> ```bash
> conda deactivate
> ```

```bash
cd ~/avl_slam_ws
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-skip zed_debug librealsense2
source install/setup.bash
```

### 5. Launch SLAM

```bash
ros2 launch avl_slam slam.launch.py
```

Optional args:

```bash
# Headless (no RTAB-Map visualizer — useful on Jetson)
ros2 launch avl_slam slam.launch.py use_rviz:=false

# Custom Velodyne IP
ros2 launch avl_slam slam.launch.py velodyne_ip:=192.168.13.11
```

### 6. Localization against a saved map

```bash
ros2 launch avl_slam localization.launch.py database_path:=~/.ros/rtabmap.db
```

---

## RTAB-Map Version Note

The apt version of RTAB-Map (`0.22.1`) is incompatible with the database format written by
newer builds (`0.23.4`). Both `rtabmap` core and `rtabmap_ros` are therefore built from source:

```
~/avl_slam_ws/src/rtabmap      ← core library
~/avl_slam_ws/src/rtabmap_ros  ← ROS 2 wrapper
```

If you ever see this error on launch:
```
Opened database version (0.23.4) is more recent than rtabmap installed version (0.22.1)
```
Delete the old database and relaunch:
```bash
rm ~/.ros/rtabmap.db
ros2 launch avl_slam slam.launch.py
```

---

## Camera Configuration

Both ZED X cameras connect via GMSL and run simultaneously in their own ROS namespaces.
RTAB-Map uses the **left camera** as its primary RGB-D input. The right camera publishes
full depth + RGB data and is available for future fusion.

| Camera | Serial | Namespace | Frame | Mount |
|---|---|---|---|---|
| ZED X Left | 43779087 | `/zed_left` | `zed_left_camera_center` | Left side, facing left (yaw +90°) |
| ZED X Right | 47753729 | `/zed_right` | `zed_right_camera_center` | Right side, facing right (yaw -90°) |

To change which camera RTAB-Map uses, update the remappings in `slam.launch.py` under the `rtabmap` node.

---

## Odometry Pipeline

LiDAR-based odometry is provided by the `icp_odometry` node (part of `rtabmap_odom`).
It consumes `/velodyne_points` and publishes to `/odom`.

```
velodyne_driver_node → /velodyne_packets
                    → velodyne_transform_node → /velodyne_points (~10 Hz assembled scans)
                                             → icp_odometry → /odom
```

> The ZED wrapper also publishes `/odometry` (its internal visual odometry) but this is
> **not** used by RTAB-Map. RTAB-Map subscribes to `/odom` from the ICP node.

Key ICP parameters (set in `slam.launch.py` and `rtabmap.yaml`):
- `Icp/MaxTranslation: 1.0` — allows up to 1m motion between frames
- `Icp/MaxCorrespondenceDistance: 0.15` — point matching tolerance
- `Icp/VoxelSize: 0.3` — downsampling for outdoor use

---

## Topic Map

```
/velodyne_packets                             ← raw UDP packets from VLP-16
/velodyne_points                              ← assembled point cloud (~10 Hz)
/zed_left/zed_node/rgb/image_rect_color       ← ZED X Left RGB (primary RTAB-Map input)
/zed_left/zed_node/rgb/camera_info
/zed_left/zed_node/depth/depth_registered     ← ZED X Left depth
/zed_right/zed_node/rgb/image_rect_color      ← ZED X Right RGB
/zed_right/zed_node/rgb/camera_info
/zed_right/zed_node/depth/depth_registered    ← ZED X Right depth
/xsens/imu/data                               ← raw Xsens IMU
/imu/filtered                                 ← Madgwick-filtered IMU → RTAB-Map input
/odom                                         ← LiDAR ICP odometry → RTAB-Map input
/odometry                                     ← ZED visual odometry (not used by RTAB-Map)

/rtabmap/mapData                              ← SLAM graph
/rtabmap/grid_map                             ← 2D occupancy grid
/rtabmap/cloud_map                            ← 3D point cloud map
/rtabmap/odom                                 ← RTAB-Map corrected odometry
/tf                                           ← transform tree
```

---

## TF Tree

```
map
 └── odom (published by rtabmap)
      └── base_link
           ├── velodyne                [0.75,  0.0,  0.3]   ← adjust to your mount
           ├── zed_left_camera_center  [-0.6,  0.35, 0.6]   ← left side, yaw +90°
           ├── zed_right_camera_center [0.6,   0.35, 0.6]   ← right side, yaw -90°
           └── imu_link                [0.7,   0.0,  0.0]
```

> **Important:** Update all static TF offsets in `slam.launch.py` to match your actual
> sensor positions measured from `base_link`.

---

## Package Verification

```bash
# ZED cameras available
/usr/local/zed/tools/ZED_Explorer -a

# GMSL daemon running
sudo systemctl status nvargus-daemon

# RTAB-Map version (should be 0.23.x from source build)
ros2 run rtabmap_slam rtabmap --version 2>&1 | grep "RTAB-Map:"

# All packages present
ros2 pkg list | grep -E "rtabmap|velodyne|zed|xsens|imu_filter"

# No missing deps
cd ~/avl_slam_ws
rosdep check --from-paths src --ignore-src -r
```

---

## Tuning Guide

### If LiDAR odometry drifts or fails

- Check `ros2 topic hz /velodyne_points` — must be ~10 Hz (not 70 Hz)
- If 70 Hz: duplicate nodes running, kill and relaunch cleanly
- Decrease `Icp/VoxelSize` (try `0.1`) for tighter registration
- Increase `Icp/MaxCorrespondenceDistance` (try `0.2`) for sparser environments
- Lower `Rtabmap/DetectionRate` to give more compute time per frame

### If loop closures are missed

- Lower `Rtabmap/LoopThr` (try `0.15`)
- Increase `Vis/MaxFeatures` (try `1000`)

### For outdoor/campus environments

- `Grid/RangeMax: "50.0"` for open areas (already set)
- `Grid/CellSize: "0.05"` gives good detail at reasonable memory cost

### ZED X depth quality

- Switch `depth.depth_mode` to `QUALITY` or `ULTRA` in `zed_left.yaml` for better accuracy
- `depth.depth_stabilization: 30` reduces flickering (already set)

### Jetson AGX Orin tips

- Monitor with `tegrastats` during first runs to check thermal throttling
- Lower ZED grab frame rate to `15` Hz (already set)
- Consider reducing `Icp/Iterations` to `20` if CPU is bottlenecking
- Use `use_rviz:=false` to reduce GPU load during field mapping

---

## Saving & Exporting the Map

```bash
# Map saves automatically to ~/.ros/rtabmap.db on shutdown

# Export to PCD point cloud — use RTAB-Map GUI: File > Export 3D Map

# Export to OctoMap
ros2 topic echo /rtabmap/octomap_full
```

---

## Re-enabling Intel RealSense D455

When the ROS connection is fixed, re-enable by uncommenting the following sections:

1. **`setup_workspace.sh`** — section `[2b]`: apt install for `realsense2-camera` and `realsense2-description`
2. **`slam.launch.py`** — the `realsense` `IncludeLaunchDescription` block and `realsense_serial_arg`
3. **`slam.launch.py`** — swap RTAB-Map remappings back to `/camera/...` topics if needed
4. **`slam.launch.py`** — uncomment `tf_base_to_camera` static TF node

---

## Common Issues

**Velodyne not found:**
Check your ethernet connection and `ping 192.168.13.11`. Set your PC's IP to `192.168.13.100/24`.

**Velodyne publishing at 70 Hz instead of 10 Hz:**
Duplicate nodes from a previous launch. Run:
```bash
killall velodyne_driver_node velodyne_transform_node
pkill -f "ros2 launch"
pkill -f component_container_isolated
```
Then relaunch.

**ZED X not detected / CAMERA STREAM FAILED TO START:**
```bash
sudo systemctl restart nvargus-daemon
```
Wait a few seconds, then relaunch.

**Both ZED X cameras show same serial:**
Check `slam.launch.py` — both `serial_number` arguments must be set uniquely.

**ZED depth and RGB out of sync:**
Increase `approx_sync_max_interval` in `rtabmap.yaml` from `0.05` to `0.1`.

**ICP odometry `ratio=0.000000` always:**
Either the Velodyne is at 70 Hz (see above) or `Icp/MaxTranslation` is too small.
Verify: `ros2 param get /icp_odometry Icp/MaxTranslation` — should be `1.0`.

**RTAB-Map database version mismatch:**
```bash
rm ~/.ros/rtabmap.db
```
The apt version (0.22.1) wrote an incompatible database. With source-built 0.23.x this won't recur.

**Build fails with librealsense2 error:**
Always skip it: `--packages-skip zed_debug librealsense2`. The source build of librealsense2 is broken; the system apt version works fine for when RealSense is re-enabled.

**conda Python breaks build:**
```bash
conda deactivate
colcon build ...
```

**Xsens port permission denied:**
```bash
sudo chmod 666 /dev/ttyUSB0
# or permanently:
sudo usermod -aG dialout $USER   # logout required
```

**RTAB-Map not creating map nodes:**
The vehicle must move at least `0.1m` or `0.05rad` before the first node is created. Drive the vehicle and check `/rtabmap/mapData`.