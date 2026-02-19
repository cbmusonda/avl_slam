# AVL SLAM — Verification, Build & Launch Command Reference

Run these in order. Each command has its expected output so you know what
good looks like vs. something missing.

---

## PART 1 — Package & Dependency Verification

### Where to run: any terminal, from any directory
```bash
source /opt/ros/humble/setup.bash
source ~/avl_slam_ws/install/setup.bash
```
Run these two lines first in every fresh terminal before the checks below.

---

### 1.1 ROS 2 Environment
```bash
echo $ROS_DISTRO
```
**Expected:**
```
humble
```

---

### 1.2 Velodyne
```bash
ros2 pkg list | grep velodyne
```
**Expected:**
```
velodyne
velodyne_driver
velodyne_laserscan
velodyne_msgs
velodyne_pointcloud
```
If any are missing: `sudo apt install ros-humble-velodyne`

---

### 1.3 RTAB-Map
```bash
ros2 pkg list | grep rtabmap
```
**Expected:**
```
rtabmap_msgs
rtabmap_ros
rtabmap_slam
rtabmap_util
rtabmap_viz
```
> **Note:** `rtabmap` core and `rtabmap_ros` are now built from source in your workspace
> (`~/avl_slam_ws/src/rtabmap` and `~/avl_slam_ws/src/rtabmap_ros`).
> The apt version (0.22.1) was incompatible with the database format (0.23.4).
> If you see a version mismatch error on launch, delete `~/.ros/rtabmap.db` and relaunch.

---

### 1.4 ZED SDK
```bash
/usr/local/zed/tools/ZED_Diagnostic
```
**Expected:** A diagnostics window opens and reports both ZED X cameras detected with firmware versions.
If it says "No ZED camera detected" — check cable connections and rerun.

```bash
# Also verify SDK version
cat /usr/local/zed/include/sl/Camera.hpp | grep ZED_SDK_MAJOR_VERSION
```
**Expected:** `#define ZED_SDK_MAJOR_VERSION 5`

---

### 1.5 ZED ROS 2 Wrapper
```bash
ros2 pkg list | grep zed
```
**Expected:**
```
zed_components
zed_ros2_interfaces
zed_wrapper
```
If missing: the wrapper wasn't built — run the build step in Part 2.

---

### 1.6 Xsens IMU Driver
```bash
ros2 pkg list | grep xsens
```
**Expected:**
```
ros2_xsens_mti_driver
```
If missing: rebuild from source (`ros2_xsens_mti_driver` is cloned by `setup_workspace.sh`)

---

### 1.7 IMU Filter (Madgwick)
```bash
ros2 pkg list | grep imu_filter
```
**Expected:**
```
imu_filter_madgwick
```
If missing: `sudo apt install ros-humble-imu-filter-madgwick`

---

### 1.8 TF2
```bash
ros2 pkg list | grep tf2
```
**Expected (among others):**
```
tf2
tf2_ros
tf2_tools
```
If missing: `sudo apt install ros-humble-tf2-ros ros-humble-tf2-tools`

---

### 1.9 PCL
```bash
ros2 pkg list | grep pcl
```
**Expected:**
```
pcl_conversions
pcl_ros
```
If missing: `sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions`

---

### 1.10 Your avl_slam package
```bash
ros2 pkg list | grep avl_slam
```
**Expected:**
```
avl_slam
```
If missing: the package hasn't been built yet — run Part 2.

---

### 1.11 Full workspace dependency check
```bash
cd ~/avl_slam_ws
rosdep check --from-paths src --ignore-src -r
```
**Expected:**
```
All system dependencies have been satisfied
```
If you see any `ERROR` lines, run:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

### 1.12 Xsens USB port
```bash
ls /dev/ttyUSB*
```
**Expected:** `/dev/ttyUSB0` (or ttyUSB1, ttyUSB2 etc. — update `xsens.yaml` to match)

```bash
sudo chmod 666 /dev/ttyUSB0   # if permission denied
```

---

### 1.13 Velodyne network connection
```bash
ping -c 4 192.168.13.11
```
**Expected:** `4 packets transmitted, 4 received, 0% packet loss`

If all packets lost: check ethernet cable and verify your machine's IP is `192.168.13.100/24`.

---

### 1.14 ZED X cameras detected
```bash
/usr/local/zed/tools/ZED_Explorer -a
```
**Expected:**
```
## Cam  0  ##
 Model :  "ZED X"
 S/N :  43779087
 State :  "AVAILABLE"
 Type :  "GMSL"

## Cam  1  ##
 Model :  "ZED X"
 S/N :  47753729
 State :  "AVAILABLE"
 Type :  "GMSL"
```
If cameras show `State: "NOT AVAILABLE"`, the GMSL daemon isn't running:
```bash
sudo systemctl status nvargus-daemon
sudo systemctl start nvargus-daemon
```

---

### 1.15 RTAB-Map version
```bash
ros2 run rtabmap_slam rtabmap --version 2>&1 | grep "RTAB-Map:"
```
**Expected:** `RTAB-Map: 0.23.x`

If it shows `0.22.1`: open a **fresh terminal** (let `.bashrc` auto-source the workspace) and retry.

---

### 1.16 Velodyne pointcloud rate (with SLAM running)
```bash
ros2 topic hz /velodyne_points
```
**Expected:** `average rate: ~10.0 Hz`

If showing ~70 Hz: duplicate velodyne nodes are running from a previous launch. Kill them:
```bash
killall velodyne_driver_node
killall velodyne_transform_node
pkill -f "ros2 launch"
pkill -f component_container_isolated
```

---

## PART 2 — Build

### Where to run: `~/avl_slam_ws`

> **Important:** Always deactivate conda before building:
> ```bash
> conda deactivate
> ```
> Conda's Python intercepts `catkin_pkg` and breaks the build.

```bash
cd ~/avl_slam_ws
```

---

### 2.1 Full workspace build
```bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-skip zed_debug librealsense2
```
**Expected:** `Summary: X packages finished [Xs]` with no `ERROR` lines.

Warnings about C++17, PCL_ROOT, or OpenCV conflicts are normal and can be ignored.

---

### 2.2 Build specific packages only (faster on re-runs)
```bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select avl_slam rtabmap rtabmap_ros zed_wrapper zed_components zed_ros2_interfaces ros2_xsens_mti_driver \
  --packages-skip zed_debug librealsense2
```

---

### 2.3 Source the workspace after building
```bash
source ~/avl_slam_ws/install/setup.bash
```

---

### 2.4 Confirm avl_slam installed correctly
```bash
ls ~/avl_slam_ws/install/avl_slam/share/avl_slam/
```
**Expected:** `config  launch  package.bash  package.dsv ...`

If `config` and `launch` are missing:
```bash
rm -rf ~/avl_slam_ws/build/avl_slam ~/avl_slam_ws/install/avl_slam
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-skip zed_debug librealsense2
```

---

## PART 3 — Launch

```bash
cd ~/avl_slam_ws
source install/setup.bash
```

---

### 3.1 Standard SLAM launch (mapping mode)
```bash
ros2 launch avl_slam slam.launch.py
```
**Expected output:**
```
[velodyne_driver] Velodyne VLP-16 rotating at 600.000000 RPM
[velodyne_driver] publishing 76 packets per scan
[velodyne_driver] expected frequency: 9.921 (Hz)
[zed_left.zed_node] Camera SN: 43779087
[zed_right.zed_node] Camera SN: 47753729
[icp_odometry] Odom: ratio=0.5xx, std dev=0.00Xm|0.00Xrad ...
[rtabmap] SLAM mode (Mem/IncrementalMemory=true)
```

**ICP odometry is working when you see:**
- `ratio > 0` (e.g. `ratio=0.532241`)
- non-zero `std dev` values (e.g. `std dev=0.003887m|0.001229rad`)
- `update time` in milliseconds (not sub-millisecond)

**ICP odometry is NOT working when you see:**
- `ratio=0.000000, std dev=0.000000`
- `update time=0.000xxx` (sub-millisecond = no computation happening)
- `Registration failed: null guess` or `limit out of bounds`

---

### 3.2 Launch headless (no visualizer)
```bash
ros2 launch avl_slam slam.launch.py use_rviz:=false
```

---

### 3.3 Launch with custom Velodyne IP
```bash
ros2 launch avl_slam slam.launch.py velodyne_ip:=192.168.13.11
```

---

### 3.4 Localization only (against a previously saved map)
```bash
ros2 launch avl_slam localization.launch.py database_path:=~/.ros/rtabmap.db
```

---

## PART 4 — Post-Launch Verification

Run these in a **second terminal** while SLAM is running.

```bash
source /opt/ros/humble/setup.bash
source ~/avl_slam_ws/install/setup.bash
```

---

### 4.1 Check all expected topics are live
```bash
ros2 topic list
```
**Expected (among others):**
```
/velodyne_points
/velodyne_packets
/zed_left/zed_node/rgb/image_rect_color
/zed_left/zed_node/rgb/camera_info
/zed_left/zed_node/depth/depth_registered
/zed_right/zed_node/rgb/image_rect_color
/zed_right/zed_node/rgb/camera_info
/zed_right/zed_node/depth/depth_registered
/imu/filtered
/odom
/rtabmap/mapData
/rtabmap/odom
/rtabmap/grid_map
/rtabmap/cloud_map
/tf
/tf_static
```
> Note: `/odom` is published by the `icp_odometry` node (LiDAR odometry).
> `/odometry` is the ZED's internal visual odometry — it exists but is NOT used by RTAB-Map.

---

### 4.2 Check topic publish rates
```bash
ros2 topic hz /velodyne_points     # Expected: ~10.0 Hz
ros2 topic hz /zed_left/zed_node/rgb/image_rect_color   # Expected: ~15.0 Hz
ros2 topic hz /zed_right/zed_node/rgb/image_rect_color  # Expected: ~15.0 Hz
ros2 topic hz /imu/filtered        # Expected: ~100.0 Hz
ros2 topic hz /odom                # Expected: ~10.0 Hz
```

---

### 4.3 Check for duplicate nodes
```bash
ros2 node list | grep velodyne
```
**Expected (exactly):**
```
/velodyne_convert
/velodyne_driver
```
If you see duplicates (`/velodyne_driver_node` alongside `/velodyne_driver`), kill everything and relaunch cleanly.

---

### 4.4 Verify ICP odometry parameters loaded
```bash
ros2 param get /icp_odometry Icp/MaxTranslation    # Expected: 1.0
ros2 param get /velodyne_convert calibration        # Expected: ...VLP16db.yaml path
```

---

### 4.5 Verify TF tree is complete
```bash
ros2 run tf2_tools view_frames
```
Open the generated `frames.pdf` and confirm:
```
map → odom → base_link → velodyne
                       → zed_left_camera_center
                       → zed_right_camera_center
                       → imu_link
```

---

### 4.6 Check RTAB-Map is building the map
```bash
ros2 topic echo /rtabmap/mapData --once
```
**Expected:** a large data dump with graph nodes.

If it hangs: RTAB-Map won't create its first node until the vehicle moves at least `0.1m` or `0.05rad`. Move the vehicle and check again.