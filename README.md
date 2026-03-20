# AVL SLAM Workspace

**Sensors:** Velodyne VLP-16 · ZED X Left · ZED X Right · ZED X Back · Intel RealSense D455 · Xsens MTi-680G IMU
**SLAM:** RTAB-Map (LiDAR-ICP odometry + RGB-D loop closure + IMU fusion)
**Platform:** ROS 2 Humble · Ubuntu aarch64 · Jetson Orin AGX / JetPack 6

> **Default mode:** All 4 cameras (RealSense D455 + ZED X Left/Right/Back) fused simultaneously in RTAB-Map multicam mode (`rgbd_cameras: 4`). No single primary source — all cameras contribute equally to loop closure and mapping.
>
> Override with `use_realsense:=false` to use ZED cameras only (3-camera multicam).

---

## Bug Fix Summary

| # | File | What was wrong | Fix |
|---|------|----------------|-----|
| 1 | `slam.launch.py` | IMU remap pointed to `/filter/imu/data` — never published by DEMCON driver | Changed to `/imu/data` (fused orientation + gyro + accel) in all RTAB-Map nodes |
| 2 | `slam.launch.py` | RealSense profile params `depth_module.profile` / `rgb_camera.profile` are realsense-ros 3.x API — silently ignored in 4.x | Changed to `depth_module.depth_profile` / `rgb_camera.color_profile` |
| 3 | `slam.launch.py` | No `initial_reset` — USB EAGAIN race on Jetson boot | Added `initial_reset:=true` to RealSense launch args |
| 4 | `slam.launch.py` | Default RGB-D source was ZED left (flaky) | Swapped to ZED right in single-camera fallback mode |
| 5 | `slam.launch.py` | `rtabmap_viz_realsense` had no condition guard — always launched | Added matching condition guards to all viz nodes |
| 6 | `rtabmap.yaml` | `imu_tf_prefix: "imu_link"` is not a valid RTAB-Map parameter | Removed; IMU frame comes from message header + static TF |
| 7 | `zed_*.yaml` | `sensors.sensors_image_sync: false` — depth and RGB timestamps can drift | Changed to `true` in all three ZED configs |
| 8 | `zed_back.yaml` | `use_sim_time` field missing | Added `use_sim_time: false` |
| 9 | `localization.launch.py` | `localization:=true` and `database_path` were never forwarded to `slam.launch.py` | Fixed pass-through of both arguments |
| 10 | `slam.launch.py` | No RTAB-Map node launched with default args (multicam ZED-only case missing) | Added `rtabmap_multicam_zed3` node for `use_multicamera=true, use_realsense=false` |
| 11 | `slam.launch.py` | `rtabmap_multicam_3/4` missing `rtabmap_common_params` — GPU tuning not applied | Fixed to use `rtabmap_common_params` |
| 12 | `slam.launch.py` + `rtabmap.yaml` | `PythonExpression` GPU overrides caused ROS 2 param type conflict (string vs integer) | Removed overrides; set GPU profile values directly in `rtabmap.yaml` |
| 13 | `realsense2_camera` | `librealsense2-gl.so.2.56` ABI-incompatible with runtime `librealsense2.so.2.57.6` | Rebuilt with `-DBUILD_ACCELERATE_GPU_WITH_GLSL=OFF`; `realsense_glsl` default set to `false` |

---

## Workspace Structure

```
avl_slam_ws/
├── setup_workspace.sh
├── README.md
├── SLAM.md                             ← hardware checklist + unit tests + launch reference
├── docs/
│   └── realsense_d455_setup.tex        ← full RealSense build/firmware setup guide
└── src/
    ├── avl_slam/                       ← custom package (configs + launches)
    │   ├── config/
    │   │   ├── rtabmap.yaml            ← RTAB-Map tuning params (GPU profile defaults)
    │   │   ├── vlp16.yaml              ← Velodyne conversion settings
    │   │   ├── xsens.yaml              ← Xsens IMU driver settings
    │   │   ├── imu_filter.yaml         ← Madgwick filter (reserved; not used)
    │   │   ├── slam.rviz               ← RViz2 visualization config
    │   │   ├── zed_left.yaml           ← ZED X left camera config
    │   │   ├── zed_right.yaml          ← ZED X right camera config
    │   │   └── zed_back.yaml           ← ZED X rear camera config
    │   └── launch/
    │       ├── slam.launch.py          ← full SLAM pipeline
    │       └── localization.launch.py  ← localization against saved map
    ├── realsense-ros/                  ← v4.56.4 from source (JetPack 6 required)
    ├── rtabmap/                        ← from source (apt version 0.22.1 too old)
    ├── rtabmap_ros/                    ← from source
    ├── zed-ros2-wrapper/               ← Stereolabs official wrapper
    ├── zed-ros2-interfaces/            ← ZED ROS 2 message definitions
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
# Full system — all 4 cameras (default)
ros2 launch avl_slam slam.launch.py

# ZED cameras only (no RealSense)
ros2 launch avl_slam slam.launch.py use_realsense:=false

# Headless — recommended on Jetson to save GPU memory
ros2 launch avl_slam slam.launch.py use_rviz:=false

# Custom database path
ros2 launch avl_slam slam.launch.py database_path:=/path/to/my_map.db
```

### 6. Localization against a saved map

```bash
ros2 launch avl_slam localization.launch.py database_path:=~/.ros/rtabmap.db
```

### 7. View a saved map

```bash
rtabmap-databaseViewer ~/.ros/rtabmap.db
```

---

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

---

## RTAB-Map Multicam Modes

The launch file selects the RTAB-Map node automatically based on launch args:

| Mode | Args | Cameras fused |
|---|---|---|
| **4-camera (default)** | `use_multicamera=true use_realsense=true` | RealSense + ZED Left/Right/Back |
| 3-camera ZED only | `use_multicamera=true use_realsense=false` | ZED Left/Right/Back |
| Single RealSense | `use_multicamera=false use_realsense=true` | RealSense only |
| Single ZED right | `use_multicamera=false use_realsense=false` | ZED Right only |

---

## IMU Integration Notes

The **DEMCON `ros2_xsens_mti_driver`** publishes to these topics:

| Topic | Content |
|-------|---------|
| `/imu/data` | **Fused:** orientation (quaternion) + angular velocity + linear accel ← **RTAB-Map uses this** |
| `/imu/mag` | Magnetometer |
| `/filter/quaternion` | Orientation only |
| `/filter/free_acceleration` | Acceleration without gravity |
| `/imu/time_ref` | Most accurate timestamps |

> **Note:** `/filter/imu/data` is **NOT published** by this driver. All RTAB-Map `imu` remaps point to `/imu/data`.

---

## Static TF Tree

```
base_link
├── velodyne                   (x=0.75,  y=0.0,  z=0.3,  yaw=0°)
├── imu_link                   (x=0.7,   y=0.0,  z=0.0,  yaw=0°)
├── zed_left_camera_center     (x=-0.6,  y=0.35, z=0.6,  yaw=+90°)
├── zed_right_camera_center    (x=0.6,   y=0.35, z=0.6,  yaw=-90°)
├── zed_back_camera_center     (x=-0.75, y=0.0,  z=0.6,  yaw=180°)
└── camera_link                (x=0.35,  y=0.0,  z=0.75, yaw=0°)  ← RealSense only
```

---

## GPU Profile (Jetson Orin AGX)

All ZED X cameras are pinned to GPU 0 (`general.gpu_id: 0`). RTAB-Map parameters are tuned for Jetson GPU throughput:

| Parameter | Value | Non-GPU value |
|---|---|---|
| `Icp/Iterations` | 20 | 30 |
| `Vis/MaxFeatures` | 400 | 600 |
| `Optimizer/Iterations` | 60 | 100 |

RealSense GLSL GPU acceleration is **disabled** (`realsense_glsl=false`) — `librealsense2-gl.so.2.56` is ABI-incompatible with runtime `librealsense2.so.2.57.6` on this system.

---

## RealSense D455 Prerequisites

- **librealsense:** v2.57.6 built from source with RSUSB backend (`-DFORCE_RSUSB_BACKEND=ON`, `-DBUILD_ACCELERATE_GPU_WITH_GLSL=OFF`)
- **Firmware:** D455 downgraded to `5.13.0.50`
- **Remove:** `sudo apt remove ros-humble-librealsense2` (conflicts with source build)
- **realsense-ros:** v4.56.4 built from source in this workspace
- **LD_LIBRARY_PATH:** `/usr/local/lib` is set in `~/.bashrc` (required for `librealsense2.so.2.56` symlink resolution)

See `docs/realsense_d455_setup.tex` for full setup instructions.

---

## Common Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| IMU silent / RTAB-Map not using IMU | Wrong `/filter/imu/data` remap (fixed) | Confirm: `ros2 topic hz /imu/data` → ~100 Hz |
| RealSense depth streams don't start | Wrong profile param names (fixed) or USB EAGAIN | Fixed with correct profile params + `initial_reset:=true` |
| RealSense `bad_optional_access` crash | Firmware ≥5.16 | Downgrade to 5.13.0.50 |
| RealSense `undefined symbol` crash | `librealsense2-gl` ABI mismatch (fixed) | Rebuild with `-DBUILD_ACCELERATE_GPU_WITH_GLSL=OFF` |
| No RTAB-Map node launches with default args | Missing multicam ZED-only node (fixed) | Fixed: `rtabmap_multicam_zed3` added |
| RTAB-Map `InvalidParameterTypeException` | GPU param overrides serialized as integers (fixed) | Fixed: GPU values set in `rtabmap.yaml` directly |
| Velodyne at 70 Hz instead of 10 Hz | Duplicate driver nodes from previous launch | `pkill -9 -f "ros2\|velodyne\|rtabmap\|zed"` |
| ZED camera not detected | nvargus-daemon not running | `sudo systemctl restart nvargus-daemon` |
| RTAB-Map not creating map nodes | Vehicle hasn't moved enough | Move ≥0.1m (`RGBD/LinearUpdate`) |
| Map empty after 5+ minutes | Car was stationary the whole time | Drive the vehicle |
| conda Python breaks build | conda intercepts catkin_pkg | `conda deactivate` before `colcon build` |
| Database version mismatch | Old apt rtabmap database opened by newer build | `rm ~/.ros/rtabmap.db` |
