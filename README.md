# AVL SLAM Workspace

**Sensors:** Velodyne VLP-16 · ZED X (Left) · ZED X (Right) · ZED X (Back) · Intel RealSense D455 · Xsens MTi IMU
**SLAM:** RTAB-Map (LiDAR-ICP odometry + RGB-D loop closure + IMU fusion)
**ROS 2 Humble**

> **RGB-D source** is selectable at launch time:
> - `use_realsense:=false` (default) → ZED X Left camera
> - `use_realsense:=true` → Intel RealSense D455
>
> All ZED X cameras always run regardless of this setting.

---

## Workspace Structure

```
avl_slam_ws/
├── setup_workspace.sh              ← run this first
├── docs/
│   └── realsense_d455_setup.tex    ← RealSense D455 setup guide (JetPack 6)
└── src/
    ├── avl_slam/                   ← custom package (configs + launches)
    │   ├── config/
    │   │   ├── rtabmap.yaml        ← RTAB-Map tuning params
    │   │   ├── vlp16.yaml          ← Velodyne conversion settings
    │   │   ├── zed_left.yaml       ← ZED X left camera config
    │   │   ├── zed_right.yaml      ← ZED X right camera config
    │   │   ├── zed_back.yaml       ← ZED X rear camera config
    │   │   ├── xsens.yaml          ← Xsens IMU driver settings
    │   │   └── imu_filter.yaml     ← Madgwick filter settings
    │   └── launch/
    │       ├── slam.launch.py          ← full SLAM pipeline
    │       └── localization.launch.py  ← localization against saved map
    ├── realsense-ros/              ← v4.56.4 from source (required for JetPack 6)
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
- **Back camera (zed_back):** set in `src/avl_slam/config/zed_back.yaml`

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
| ZED X Back | SN: set in `zed_back.yaml` — verify with `ZED_Explorer -a` |
| RealSense D455 | USB 3.2 connection; FW 5.13.0.50; `rs-enumerate-devices --compact` |
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
    -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
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

# Use RealSense D455 as primary RGB-D source
ros2 launch avl_slam slam.launch.py use_realsense:=true

# RealSense + headless
ros2 launch avl_slam slam.launch.py use_realsense:=true use_rviz:=false

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

All ZED X cameras connect via GMSL and run simultaneously in their own ROS namespaces.
The Intel RealSense D455 connects via USB 3.2. RTAB-Map uses either the ZED X Left or
the RealSense D455 as its primary RGB-D input, controlled by the `use_realsense` launch argument.

| Camera | Serial | Namespace | Frame | Mount |
|---|---|---|---|---|
| ZED X Left | 43779087 | `/zed_left` | `zed_left_camera_center` | Left side, facing left (yaw +90°) |
| ZED X Right | 47753729 | `/zed_right` | `zed_right_camera_center` | Right side, facing right (yaw -90°) |
| ZED X Back | set in `zed_back.yaml` | `/zed_back` | `zed_back_camera_center` | Rear center, facing backward (yaw 180°) |
| RealSense D455 | (auto) | `/camera/camera` | `camera_link` | Front, facing forward |

To change which camera RTAB-Map uses, set `use_realsense:=true` or `use_realsense:=false` at launch.

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
/zed_left/zed_node/rgb/image_rect_color       ← ZED X Left RGB
/zed_left/zed_node/rgb/camera_info
/zed_left/zed_node/depth/depth_registered     ← ZED X Left depth
/zed_right/zed_node/rgb/image_rect_color      ← ZED X Right RGB
/zed_right/zed_node/rgb/camera_info
/zed_right/zed_node/depth/depth_registered    ← ZED X Right depth
/zed_back/zed_node/rgb/image_rect_color       ← ZED X Back RGB
/zed_back/zed_node/rgb/camera_info
/zed_back/zed_node/depth/depth_registered     ← ZED X Back depth
/camera/camera/color/image_raw                ← RealSense D455 RGB (when enabled)
/camera/camera/color/camera_info
/camera/camera/aligned_depth_to_color/image_raw  ← RealSense D455 aligned depth
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
           ├── zed_back_camera_center  [-0.75, 0.0,  0.6]   ← rear side, yaw 180°
           ├── camera_link             [0.35,  0.0,  0.75]  ← RealSense D455 (when enabled)
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

# RealSense D455 (when connected)
rs-enumerate-devices --compact
# Expected: Intel RealSense D455  5.13.0.50  USB 3.2

# RTAB-Map version (should be 0.23.x from source build)
ros2 run rtabmap_slam rtabmap --version 2>&1 | grep "RTAB-Map:"

# All packages present
ros2 pkg list | grep -E "rtabmap|velodyne|zed|xsens|imu_filter|realsense"

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

## Intel RealSense D455 Setup

The standard apt packages for librealsense/realsense-ros do **not** work on JetPack 6. The D455 crashes with `std::bad_optional_access` because firmware 5.16+ exposes HID descriptors that the RSUSB userspace backend cannot handle. The fix requires building from source and downgrading firmware.

| Component | Required Version |
|---|---|
| librealsense | 2.57.6 (source, RSUSB backend) |
| realsense-ros | 4.56.4 (source, cloned by `setup_workspace.sh`) |
| D455 firmware | 5.13.0.50 |

### Setup steps (one-time)

Full instructions are in `docs/realsense_d455_setup.tex`. Summary:

```bash
# 1. Build librealsense 2.57.6 from source
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense && git checkout v2.57.6
mkdir -p build && cd build
cmake .. -DFORCE_RSUSB_BACKEND=ON -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=true -DBUILD_WITH_CUDA=true
make -j6
sudo make install && sudo ldconfig

# 2. Downgrade D455 firmware to 5.13.0.50
wget https://librealsense.intel.com/Releases/RS4xx/FW/D4XX_FW_Image-5.13.0.50.bin \
  -O ~/D4XX_FW_Image-5.13.0.50.bin
rs-fw-update -f ~/D4XX_FW_Image-5.13.0.50.bin

# 3. Remove apt librealsense (header conflicts)
sudo apt remove ros-humble-librealsense2 ros-humble-librealsense2-dbgsym

# 4. Rebuild workspace (realsense-ros 4.56.4 is already cloned by setup script)
cd ~/avl_slam_ws
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
  --packages-skip zed_debug librealsense2

# 5. Verify
rs-enumerate-devices --compact
# Expected: Intel RealSense D455  5.13.0.50  USB 3.2
```

### Launch with RealSense

```bash
ros2 launch avl_slam slam.launch.py use_realsense:=true
```

### Important notes

- The D455 IMU does **not** work with the RSUSB backend on JetPack 6. Keep `enable_gyro` and `enable_accel` set to `false`. Use the Xsens external IMU instead.
- Do **not** use realsense-ros tag 4.57.6 — it requires `RS2_STREAM_SAFETY` which doesn't exist in librealsense 2.57.6.
- `control_transfer returned error` warnings are normal with RSUSB on JetPack 6 and don't affect streaming.

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
Always skip it: `--packages-skip zed_debug librealsense2`. The source-built librealsense2 at `/usr/local/lib/` is used directly; the ROS package wrapper is not needed.

**RealSense `bad_optional_access` crash on launch:**
D455 firmware is ≥5.16. Downgrade to 5.13.0.50 — see `docs/realsense_d455_setup.tex` Step 2.

**RealSense "Built with LibRealSense v2.56.4" version mismatch:**
The apt `ros-humble-librealsense2` headers are shadowing the source install. Remove them:
```bash
sudo apt remove ros-humble-librealsense2
```

**RealSense "No HID info provided, IMU is disabled":**
Expected with RSUSB backend on JetPack 6. The D455 IMU is not available — use the Xsens external IMU.

**RealSense USB interface busy on relaunch:**
Previous node still holds the device:
```bash
pkill -f realsense2_camera_node && sleep 2
```

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
---

## RealSense D455 Standalone Test

Step-by-step procedure to verify the RealSense D455 is working independently before running the full SLAM pipeline.

### 1. Kill any existing camera processes

```bash
pkill -f realsense2_camera_node
sleep 2
```

### 2. Source the workspace

```bash
source /opt/ros/humble/setup.bash
source ~/avl_slam_ws/install/setup.bash
```

### 3. Verify the camera is detected

```bash
rs-enumerate-devices --compact
# Expected output:
#   Intel RealSense D455  215122251311  5.13.0.50
```

### 4. Launch the RealSense node

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=camera \
  enable_color:=true \
  enable_depth:=true \
  enable_gyro:=false \
  enable_accel:=false \
  align_depth.enable:=true \
  depth_module.profile:=640x480x30 \
  rgb_camera.profile:=640x480x30
```

Expected output:
```
RealSense ROS v4.56.4
Built with LibRealSense v2.57.6
Running with LibRealSense v2.57.6
Device Name: Intel RealSense D455
Device FW version: 5.13.0.50
Device USB type: 3.2
RealSense Node Is Up!
```

The `control_transfer returned error` warnings are normal with RSUSB backend on JetPack 6 and do not affect streaming.

### 5. Check topics are publishing (in a second terminal)

```bash
source /opt/ros/humble/setup.bash
source ~/avl_slam_ws/install/setup.bash

# List camera topics
ros2 topic list | grep camera

# Expected topics:
#   /camera/camera/color/image_raw
#   /camera/camera/color/camera_info
#   /camera/camera/aligned_depth_to_color/image_raw
#   /camera/camera/aligned_depth_to_color/camera_info
#   /camera/camera/depth/image_rect_raw
#   /camera/camera/depth/camera_info
```

### 6. Verify frame rates

```bash
# Color — expect ~30 Hz
ros2 topic hz /camera/camera/color/image_raw

# Aligned depth — expect ~29 Hz
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```

### 7. Quick image check (if display available)

```bash
ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw
```

### Notes

- If `aligned_depth_to_color` topics are missing, make sure `align_depth.enable:=true` is set. RTAB-Map requires this topic.
- If color fps is significantly below 30 with high jitter, check for `Left MIPI error` warnings in the launch output — this indicates a USB connection issue. Try a different USB 3.2 port or cable.
- The D455 IMU is disabled (`enable_gyro:=false`, `enable_accel:=false`) because the RSUSB backend on JetPack 6 cannot access HID. The Xsens external IMU is used instead.
