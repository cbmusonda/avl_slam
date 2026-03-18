# AVL SLAM Workspace

**Sensors:** Velodyne VLP-16 · ZED X Left · ZED X Right · ZED X Back · Intel RealSense D455 · Xsens MTi-680G IMU  
**SLAM:** RTAB-Map (LiDAR-ICP odometry + RGB-D loop closure + IMU fusion)  
**Platform:** ROS 2 Humble · Ubuntu aarch64 · Jetson / JetPack 6

> **RGB-D source** is selectable at launch time:
> - `use_realsense:=false` (default) → ZED X **Right** camera (reliable; confirmed working)
> - `use_realsense:=true` → Intel RealSense D455
>
> All three ZED X cameras always run regardless of this setting.

---

## Bug Fix Summary (vs previous version)

| # | File | What was wrong | Fix |
|---|------|----------------|-----|
| 1 | `slam.launch.py` | IMU remap pointed to `/imu/data` (raw, no orientation) | Changed to `/filter/imu/data` (fused, with orientation) in all 4 nodes |
| 2 | `slam.launch.py` | RealSense profile params `depth_module.profile` / `rgb_camera.profile` are realsense-ros 3.x API — silently ignored in 4.x | Changed to `depth_module.depth_profile` / `rgb_camera.color_profile` |
| 3 | `slam.launch.py` | No `initial_reset` — USB EAGAIN race on Jetson boot | Added `initial_reset:=true` to RealSense launch args |
| 4 | `slam.launch.py` | Default RGB-D source was ZED left (flaky) | Swapped to ZED right (confirmed reliable) in `rtabmap_zed` + both viz nodes |
| 5 | `slam.launch.py` | `rtabmap_viz_realsense` had no condition guard — always launched | Added `condition=IfCondition(use_realsense)` |
| 6 | `rtabmap.yaml` | `imu_tf_prefix: "imu_link"` is not a valid RTAB-Map parameter | Removed; IMU frame comes from message header + static TF |
| 7 | `zed_*.yaml` | `sensors.sensors_image_sync: false` — depth and RGB timestamps can drift | Changed to `true` in all three ZED configs |
| 8 | `zed_back.yaml` | `use_sim_time` field missing | Added `use_sim_time: false` |
| 9 | `localization.launch.py` | `localization:=true` and `database_path` were never forwarded to `slam.launch.py` | Fixed pass-through of both arguments |

---

## Workspace Structure

```
avl_slam_ws/
├── setup_workspace.sh
├── docs/
│   └── realsense_d455_setup.tex
└── src/
    ├── avl_slam/                       ← custom package (configs + launches)
    │   ├── config/
    │   │   ├── rtabmap.yaml            ← RTAB-Map tuning params
    │   │   ├── vlp16.yaml              ← Velodyne conversion settings
    │   │   ├── xsens.yaml              ← Xsens IMU driver settings
    │   │   ├── imu_filter.yaml         ← Madgwick filter (reserved; not used)
    │   │   ├── zed_left.yaml           ← ZED X left camera config
    │   │   ├── zed_right.yaml          ← ZED X right camera config (primary)
    │   │   └── zed_back.yaml           ← ZED X rear camera config
    │   └── launch/
    │       ├── slam.launch.py          ← full SLAM pipeline
    │       └── localization.launch.py  ← localization against saved map
    ├── realsense-ros/                  ← v4.56.4 from source (JetPack 6 required)
    ├── rtabmap/                        ← from source (apt version 0.22.1 too old)
    ├── rtabmap_ros/                    ← from source
    ├── zed-ros2-wrapper/               ← Stereolabs official wrapper
    └── ros2_xsens_mti_driver/          ← DEMCON fork
```

---

## Quick Start

### 1. Run setup script (first time only)

```bash
chmod +x setup_workspace.sh
./setup_workspace.sh
source ~/.bashrc
```

### 2. Serial numbers

Your ZED X serial numbers are already set in `slam.launch.py` and config files:

| Camera | Serial | GMSL Port |
|--------|--------|-----------|
| zed_left | 43779087 | port 7 (`/dev/i2c-10`) |
| zed_right | 47753729 | port 6 (`/dev/i2c-9`) |
| zed_back | 49910017 | set in `zed_back.yaml` |

To verify: `grep serial_number ~/avl_slam_ws/src/avl_slam/launch/slam.launch.py`

### 3. Hardware checklist before every launch

| Sensor | Check |
|--------|-------|
| VLP-16 | Ping `192.168.13.11` — your PC must be on `192.168.13.100/24` |
| ZED X cameras | `sudo systemctl status nvargus-daemon` must be **active** |
| Xsens IMU | `ls /dev/ttyUSB*` — note port, update `xsens.yaml` if not `ttyUSB0` |
| RealSense D455 | `rs-enumerate-devices --compact` — FW must be `5.13.0.50` |

### 4. Build

> Always deactivate conda first:
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
# Standard (ZED right as RGB-D, with visualizer)
ros2 launch avl_slam slam.launch.py

# Headless — recommended on Jetson to save GPU memory
ros2 launch avl_slam slam.launch.py use_rviz:=false

# RealSense D455 as RGB-D source
ros2 launch avl_slam slam.launch.py use_realsense:=true use_rviz:=false

# Custom Velodyne IP
ros2 launch avl_slam slam.launch.py velodyne_ip:=192.168.13.11
```

### 6. Localization against a saved map

```bash
ros2 launch avl_slam localization.launch.py database_path:=~/.ros/rtabmap.db
```

---

## IMU Integration — Important Notes

The **DEMCON `ros2_xsens_mti_driver`** publishes two different IMU topics:

| Topic | Content | Use for |
|-------|---------|---------|
| `/filter/imu/data` | Fused: orientation + angular velocity + linear accel | **RTAB-Map** — this is what you want |
| `/imu/data` | Raw: angular velocity + linear accel only (no orientation) | Debugging raw data only |

RTAB-Map requires the **fused** orientation quaternion to integrate IMU into the pose graph.  
All `imu` remaps in `slam.launch.py` now correctly point to `/filter/imu/data`.

To verify IMU is reaching RTAB-Map after launch:
```bash
ros2 topic hz /filter/imu/data        # Expected: ~100 Hz
ros2 topic echo /filter/imu/data --once | grep orientation
# Expected: x/y/z/w values that are NOT all 0.0
```

---

## RealSense D455 Notes

The D455 requires specific setup on JetPack 6. See `docs/realsense_d455_setup.tex` for the full procedure. Summary:

- **librealsense:** Must be built from source v2.57.6 with RSUSB backend
- **Firmware:** Must be exactly `5.13.0.50` — v5.16+ causes `bad_optional_access` crash
- **D455 IMU disabled** — RSUSB backend on JetPack 6 cannot access HID. Xsens handles IMU.
- **`control_transfer returned error` warnings** are normal with RSUSB and do not affect streaming.

Standalone test (before running full SLAM):
```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=camera \
  enable_color:=true \
  enable_depth:=true \
  enable_gyro:=false \
  enable_accel:=false \
  align_depth.enable:=true \
  depth_module.depth_profile:=640x480x30 \
  rgb_camera.color_profile:=640x480x30 \
  initial_reset:=true
```

---

## ZED X Camera Notes

All three ZED X cameras connect via GMSL. If any camera fails to start:

```bash
# Restart GMSL daemon (fixes most ZED startup failures)
sudo systemctl restart nvargus-daemon
sleep 3
ros2 launch avl_slam slam.launch.py
```

ZED left camera (`43779087`) has intermittent startup failures in some configurations.  
ZED right (`47753729`) is the confirmed-reliable primary RGB-D source for RTAB-Map.

---

## RTAB-Map Version Note

Both `rtabmap` core and `rtabmap_ros` are built from source (`~0.23.x`).  
The apt version (`0.22.1`) is incompatible with the database format written by newer builds.

If you see:
```
Opened database version (0.23.x) is more recent than rtabmap installed version (0.22.1)
```
Delete the old database:
```bash
rm ~/.ros/rtabmap.db
ros2 launch avl_slam slam.launch.py
```

---

## Common Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| IMU silent / RTAB-Map not using IMU | Old `/imu/data` remap (fixed in this version) | Confirm topic: `ros2 topic hz /filter/imu/data` |
| RealSense depth streams don't start | Wrong profile param names (fixed) or USB EAGAIN | Fixed with correct profile params + `initial_reset:=true` |
| RealSense `bad_optional_access` crash | Firmware ≥5.16 | Downgrade to 5.13.0.50 |
| Velodyne at 70 Hz instead of 10 Hz | Duplicate driver nodes from previous launch | `killall velodyne_driver_node velodyne_transform_node && pkill -f "ros2 launch"` |
| ZED camera not detected | nvargus-daemon not running | `sudo systemctl restart nvargus-daemon` |
| ICP `ratio=0.000000` | Velodyne at 70 Hz or `Icp/MaxTranslation` too small | Kill duplicate nodes; verify `ros2 param get /icp_odometry Icp/MaxTranslation` = `1.0` |
| RTAB-Map not creating map nodes | Vehicle hasn't moved enough | Move ≥0.1m or ≥0.05rad, then check `/rtabmap/mapData` |
| conda Python breaks build | conda Python intercepts catkin_pkg | `conda deactivate` before `colcon build` |
| RealSense USB busy on relaunch | Previous node still holds device | `pkill -f realsense2_camera_node && sleep 2` |
| Localization mode ignored | `localization` arg not forwarded (fixed in this version) | Use updated `localization.launch.py` |
| `imu_tf_prefix` warning in RTAB-Map | Not a valid param (fixed in this version) | Use updated `rtabmap.yaml` |