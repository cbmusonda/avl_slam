# AVL SLAM — Sensor Unit Tests, Build & Launch Reference

Run every section in order from top to bottom the first time.  
After initial validation, jump to the section you need.

---

## Environment Setup (run in every new terminal)

```bash
source /opt/ros/humble/setup.bash
source ~/avl_slam_ws/install/setup.bash
```

---

## PART 1 — Package Verification

### 1.1 ROS 2 distro
```bash
echo $ROS_DISTRO
```
**Expected:** `humble`

---

### 1.2 Velodyne packages
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
**If missing:** `sudo apt install ros-humble-velodyne`

---

### 1.3 RTAB-Map packages
```bash
ros2 pkg list | grep rtabmap
```
**Expected:**
```
rtabmap_msgs
rtabmap_odom
rtabmap_ros
rtabmap_slam
rtabmap_util
rtabmap_viz
```
> Both `rtabmap` core and `rtabmap_ros` are built from source in `~/avl_slam_ws/src/`.  
> If you see version `0.22.1` from apt, open a fresh terminal (let `.bashrc` re-source workspace).

---

### 1.4 ZED SDK
```bash
cat /usr/local/zed/include/sl/Camera.hpp | grep ZED_SDK_MAJOR_VERSION
```
**Expected:** `#define ZED_SDK_MAJOR_VERSION 5`

```bash
/usr/local/zed/tools/ZED_Diagnostic
```
**Expected:** diagnostics window opens and reports connected cameras.

---

### 1.5 ZED ROS 2 wrapper
```bash
ros2 pkg list | grep zed
```
**Expected:**
```
zed_components
zed_ros2_interfaces
zed_wrapper
```

---

### 1.6 Xsens driver
```bash
ros2 pkg list | grep xsens
```
**Expected:** `ros2_xsens_mti_driver`

---

### 1.7 RealSense packages
```bash
ros2 pkg list | grep realsense
```
**Expected:**
```
realsense2_camera
realsense2_camera_msgs
realsense2_description
```

---

### 1.8 avl_slam package
```bash
ros2 pkg list | grep avl_slam
```
**Expected:** `avl_slam`

---

### 1.9 Full dependency check
```bash
cd ~/avl_slam_ws
rosdep check --from-paths src --ignore-src -r
```
**Expected:** `All system dependencies have been satisfied`  
**If errors:** `rosdep install --from-paths src --ignore-src -r -y`

---

## PART 2 — Hardware Pre-flight

### 2.1 Xsens IMU — port check
```bash
ls /dev/ttyUSB*
```
**Expected:** `/dev/ttyUSB0` (or `ttyUSB1`, etc.)  
Update `src/avl_slam/config/xsens.yaml` → `port:` if different from `ttyUSB0`.

```bash
# Fix permission denied (required after reboot until dialout group takes effect)
sudo chmod 666 /dev/ttyUSB0

# Permanent fix (requires logout after running):
sudo usermod -aG dialout $USER
```

---

### 2.2 Velodyne — network
```bash
ping -c 4 192.168.13.11
```
**Expected:** `4 packets transmitted, 4 received, 0% packet loss`  
**If failing:** set your machine's ethernet IP to `192.168.13.100/24`, subnet `255.255.255.0`.

---

### 2.3 ZED X cameras — GMSL daemon + detection
```bash
sudo systemctl status nvargus-daemon
```
**Expected:** `active (running)`  
**If not running:** `sudo systemctl start nvargus-daemon && sleep 3`

```bash
/usr/local/zed/tools/ZED_Explorer -a
```
**Expected:**
```
## Cam  0  ##  Model: ZED X  S/N: 43779087  State: AVAILABLE  Type: GMSL
## Cam  1  ##  Model: ZED X  S/N: 47753729  State: AVAILABLE  Type: GMSL
## Cam  2  ##  Model: ZED X  S/N: 49910017  State: AVAILABLE  Type: GMSL
```
**If any camera shows `NOT AVAILABLE`:** `sudo systemctl restart nvargus-daemon && sleep 3`

---

### 2.4 RealSense D455 — device enumeration
```bash
rs-enumerate-devices --compact
```
**Expected:**
```
Intel RealSense D455  <serial>  5.13.0.50  USB 3.2
```
**If firmware ≥5.16:** downgrade — see `docs/realsense_d455_setup.tex` Step 2.  
**If not detected:** try a different USB 3.2 port; avoid USB hubs.

---

## PART 3 — Sensor Unit Tests

Run each test in isolation (no other ROS nodes running). Kill everything before each test:

```bash
pkill -f "ros2 launch" ; pkill -f "ros2 run" ; sleep 2
```

---

### TEST A — Xsens IMU

**Goal:** Confirm the driver starts, connects to the device, and publishes fused orientation.

#### A.1 Launch the driver
```bash
ros2 run ros2_xsens_mti_driver xsens_mti_node \
  --ros-args \
  --params-file ~/avl_slam_ws/src/avl_slam/config/xsens.yaml
```
**Expected output:**
```
[xsens_mti_node] Found a device with ID: ...
[xsens_mti_node] Device: MTi-680G, firmware: x.x.x
[xsens_mti_node] Calibration complete
```
**If `[Errno 13] Permission denied /dev/ttyUSB0`:** `sudo chmod 666 /dev/ttyUSB0`  
**If `No device found`:** check cable and `ls /dev/ttyUSB*` — update `port:` in `xsens.yaml`.

#### A.2 Verify topics (second terminal)
```bash
ros2 topic list | grep -E "imu|filter"
```
**Expected:**
```
/filter/free_acceleration
/filter/imu/data
/filter/quaternion
/imu/data
/imu/mag
/imu/time_ref
```

#### A.3 Check publish rate
```bash
ros2 topic hz /filter/imu/data
```
**Expected:** `average rate: ~100.0 Hz` (at baudrate 115200)

```bash
ros2 topic hz /imu/data
```
**Expected:** `average rate: ~100.0 Hz`

#### A.4 Verify fused orientation is present
```bash
ros2 topic echo /filter/imu/data --once
```
**Expected:** `orientation.w` is close to `1.0` (or some non-zero quaternion).  
A fully zero quaternion means the filter hasn't converged yet — let the IMU sit still for ~5 seconds.

> **Critical:** `/filter/imu/data` must have a valid orientation quaternion.  
> `/imu/data` does NOT contain orientation — it is raw accel + gyro only.  
> RTAB-Map subscribes to `/filter/imu/data`. Using `/imu/data` gives RTAB-Map no heading.

#### A.5 Check frame_id
```bash
ros2 topic echo /filter/imu/data --once | grep frame_id
```
**Expected:** `frame_id: imu_link`

---

### TEST B — Intel RealSense D455

**Goal:** Confirm color and aligned depth streams start at 640×480×30.

#### B.1 Kill any existing camera processes
```bash
pkill -f realsense2_camera_node ; sleep 2
```

#### B.2 Launch the RealSense node
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
**Expected output:**
```
RealSense ROS v4.56.4
Built with LibRealSense v2.57.6
Running with LibRealSense v2.57.6
Device Name: Intel RealSense D455
Device FW version: 5.13.0.50
Device USB type: 3.2
RealSense Node Is Up!
```
`control_transfer returned error` warnings are **normal** with RSUSB on JetPack 6.

#### B.3 Verify topics
```bash
ros2 topic list | grep camera
```
**Expected:**
```
/camera/camera/aligned_depth_to_color/camera_info
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
/camera/camera/color/image_raw
/camera/camera/depth/camera_info
/camera/camera/depth/image_rect_raw
```

#### B.4 Check publish rates
```bash
ros2 topic hz /camera/camera/color/image_raw
```
**Expected:** `average rate: ~30.0 Hz`

```bash
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```
**Expected:** `average rate: ~29.0–30.0 Hz`

#### B.5 Verify resolution
```bash
ros2 topic echo /camera/camera/color/camera_info --once | grep -E "width|height"
```
**Expected:** `width: 640` and `height: 480`

#### B.6 Optional image view (if display attached)
```bash
ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw
```

---

### TEST C — ZED X Right Camera (primary)

**Goal:** Confirm ZED right starts, publishes RGB + depth at matched timestamps.

#### C.1 Restart GMSL daemon
```bash
sudo systemctl restart nvargus-daemon ; sleep 3
```

#### C.2 Launch ZED right only
```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zedx \
  camera_name:=zed_right \
  node_name:=zed_node \
  serial_number:=47753729 \
  publish_tf:=false \
  publish_urdf:=false \
  ros_params_override_path:=~/avl_slam_ws/src/avl_slam/config/zed_right.yaml
```
**Expected output:**
```
[zed_node] Camera SN: 47753729
[zed_node] Camera FW version: x.x.x
[zed_node] Using resolution HD720 @ 15 fps
```

#### C.3 Verify topics
```bash
ros2 topic list | grep zed_right
```
**Expected (key topics):**
```
/zed_right/zed_node/depth/depth_registered
/zed_right/zed_node/rgb/camera_info
/zed_right/zed_node/rgb/image_rect_color
```

#### C.4 Check publish rates
```bash
ros2 topic hz /zed_right/zed_node/rgb/image_rect_color
```
**Expected:** `average rate: ~15.0 Hz`

```bash
ros2 topic hz /zed_right/zed_node/depth/depth_registered
```
**Expected:** `average rate: ~15.0 Hz`

#### C.5 Verify depth/RGB timestamp alignment
```bash
ros2 topic echo /zed_right/zed_node/rgb/image_rect_color --once | grep stamp
ros2 topic echo /zed_right/zed_node/depth/depth_registered --once | grep stamp
```
**Expected:** timestamps should match within 5ms (sensors_image_sync: true enforces this).

---

### TEST D — ZED X Left Camera (auxiliary)

Same procedure as Test C with:
- `serial_number:=43779087`
- `camera_name:=zed_left`
- `ros_params_override_path:=.../config/zed_left.yaml`
- Topics under `/zed_left/zed_node/...`

> ZED left has intermittent startup failures. If it doesn't start within 15 seconds,  
> `Ctrl+C`, `sudo systemctl restart nvargus-daemon`, wait 3s, retry.

---

### TEST E — ZED X Back Camera (auxiliary)

Same procedure as Test C with:
- `serial_number:=49910017`
- `camera_name:=zed_back`
- `ros_params_override_path:=.../config/zed_back.yaml`
- Topics under `/zed_back/zed_node/...`

---

### TEST F — Velodyne VLP-16

**Goal:** Confirm 10 Hz point cloud publication with no duplicate nodes.

#### F.1 Launch Velodyne driver + convert
```bash
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
```
Or launch just the two nodes:
```bash
ros2 run velodyne_driver velodyne_driver_node \
  --ros-args \
  -p device_ip:=192.168.13.11 \
  -p model:=VLP16 \
  -p rpm:=600.0 \
  -p frame_id:=velodyne

# Second terminal:
ros2 run velodyne_pointcloud velodyne_transform_node \
  --ros-args \
  --params-file ~/avl_slam_ws/src/avl_slam/config/vlp16.yaml \
  -p calibration:=/opt/ros/humble/share/velodyne_pointcloud/params/VLP16db.yaml
```

#### F.2 Verify 10 Hz publication
```bash
ros2 topic hz /velodyne_points
```
**Expected:** `average rate: ~10.0 Hz`  
**If ~70 Hz:** duplicate nodes running — kill everything and relaunch cleanly:
```bash
killall velodyne_driver_node velodyne_transform_node
pkill -f "ros2 launch" ; pkill -f component_container_isolated
```

#### F.3 Check node list (no duplicates)
```bash
ros2 node list | grep velodyne
```
**Expected exactly:**
```
/velodyne_convert
/velodyne_driver
```

#### F.4 Check point cloud frame
```bash
ros2 topic echo /velodyne_points --once | grep frame_id
```
**Expected:** `frame_id: velodyne`

---

### TEST G — ICP Odometry (requires Velodyne running)

**Goal:** Confirm ICP is computing valid odometry from the point cloud.

#### G.1 Launch ICP node (with Velodyne already running)
```bash
ros2 run rtabmap_odom icp_odometry \
  --ros-args \
  --params-file ~/avl_slam_ws/src/avl_slam/config/rtabmap.yaml \
  -p Icp/MaxTranslation:=1.0 \
  -p Odom/Strategy:=0 \
  --remap scan_cloud:=/velodyne_points \
  --remap odom:=/odom
```

#### G.2 Verify odometry is publishing
```bash
ros2 topic hz /odom
```
**Expected:** `average rate: ~10.0 Hz` (matches Velodyne rate)

#### G.3 Check ICP console output
**Working ICP looks like:**
```
[icp_odometry] ratio=0.532241, std dev=0.003887m|0.001229rad, update time=18.3ms
```
**Broken ICP looks like:**
```
[icp_odometry] ratio=0.000000, std dev=0.000000m|0.000000rad, update time=0.000ms
```
If ICP shows `ratio=0.000000`: Velodyne is at 70 Hz. Kill duplicates and relaunch.

---

## PART 4 — Build

> Always deactivate conda before building:
> ```bash
> conda deactivate
> ```

### 4.1 Full workspace build
```bash
cd ~/avl_slam_ws
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
  --packages-skip zed_debug librealsense2
```
**Expected:** `Summary: X packages finished [Xs]` — no `ERROR` lines.  
Warnings about C++17, PCL_ROOT, or OpenCV are normal.

### 4.2 Rebuild specific packages (faster on iterative changes)
```bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
  --packages-select avl_slam rtabmap rtabmap_ros zed_wrapper zed_components \
      zed_ros2_interfaces ros2_xsens_mti_driver \
      realsense2_camera_msgs realsense2_description realsense2_camera \
  --packages-skip zed_debug librealsense2
```

### 4.3 Source after building
```bash
source ~/avl_slam_ws/install/setup.bash
```

### 4.4 Confirm config files installed
```bash
ls ~/avl_slam_ws/install/avl_slam/share/avl_slam/
```
**Expected:** `config  launch  package.bash  package.dsv ...`

---

## PART 5 — SLAM Launch

### Pre-launch cleanup (run every time)
```bash
pkill -f "ros2 launch"
pkill -f component_container_isolated
pkill -f velodyne_driver_node
pkill -f realsense2_camera_node
sudo systemctl restart nvargus-daemon
sleep 3
conda deactivate
source /opt/ros/humble/setup.bash
source ~/avl_slam_ws/install/setup.bash
```

---

### 5.1 Standard SLAM — ZED right as RGB-D (default)
```bash
ros2 launch avl_slam slam.launch.py
```
**Confirm in output:**
```
[velodyne_driver] expected frequency: 9.921 (Hz)
[zed_right.zed_node] Camera SN: 47753729
[xsens_mti_node] Device: MTi-680G
[icp_odometry] ratio=0.5xx, std dev=...
[rtabmap] SLAM mode (Mem/IncrementalMemory=true)
```

---

### 5.2 Headless (recommended on Jetson)
```bash
ros2 launch avl_slam slam.launch.py use_rviz:=false
```

---

### 5.3 RealSense D455 as RGB-D
```bash
ros2 launch avl_slam slam.launch.py use_realsense:=true use_rviz:=false
```
**Additional expected output:**
```
Device Name: Intel RealSense D455
Device FW version: 5.13.0.50
RealSense Node Is Up!
```

---

### 5.4 Localization against saved map
```bash
ros2 launch avl_slam localization.launch.py \
  database_path:=~/.ros/rtabmap.db \
  use_rviz:=false
```

---

## PART 6 — Post-Launch Verification

Open a second terminal and source the workspace, then run:

### 6.1 All expected topics live
```bash
ros2 topic list | grep -E "velodyne|zed|filter/imu|odom|rtabmap|camera"
```
**Expected (ZED mode):**
```
/filter/imu/data
/odom
/rtabmap/cloud_map
/rtabmap/grid_map
/rtabmap/mapData
/rtabmap/odom
/velodyne_packets
/velodyne_points
/zed_back/zed_node/depth/depth_registered
/zed_back/zed_node/rgb/image_rect_color
/zed_left/zed_node/depth/depth_registered
/zed_left/zed_node/rgb/image_rect_color
/zed_right/zed_node/depth/depth_registered
/zed_right/zed_node/rgb/image_rect_color
```
**Additionally with `use_realsense:=true`:**
```
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/image_raw
```

---

### 6.2 Topic rates
```bash
ros2 topic hz /velodyne_points                               # ~10 Hz
ros2 topic hz /filter/imu/data                              # ~100 Hz
ros2 topic hz /odom                                         # ~10 Hz
ros2 topic hz /zed_right/zed_node/rgb/image_rect_color      # ~15 Hz
ros2 topic hz /zed_right/zed_node/depth/depth_registered    # ~15 Hz
```

---

### 6.3 No duplicate Velodyne nodes
```bash
ros2 node list | grep velodyne
```
**Expected exactly:** `/velodyne_convert` and `/velodyne_driver`

---

### 6.4 TF tree complete
```bash
ros2 run tf2_tools view_frames
```
Open `frames.pdf`. Confirm:
```
map → odom → base_link → velodyne
                       → zed_left_camera_center
                       → zed_right_camera_center
                       → zed_back_camera_center
                       → imu_link
                       → camera_link  (only if use_realsense:=true)
```

---

### 6.5 ICP odometry parameters loaded correctly
```bash
ros2 param get /icp_odometry Icp/MaxTranslation
```
**Expected:** `1.0`

---

### 6.6 IMU actually reaching RTAB-Map
```bash
ros2 topic echo /filter/imu/data --once | grep -A 4 orientation
```
**Expected:** non-zero `x`, `y`, `z`, `w` values. If all zeros, the Xsens filter hasn't converged — let it sit still for 5 seconds.

---

### 6.7 RTAB-Map building the map
```bash
ros2 topic echo /rtabmap/mapData --once
```
**Expected:** large data dump with graph node info.  
**If hangs:** RTAB-Map creates its first node only after `0.1m` or `0.05rad` of motion. Drive the vehicle.

---

## PART 7 — Bag Recording & Replay

### Record a bag
```bash
mkdir -p ~/bags
ros2 bag record \
  /velodyne_points \
  /zed_right/zed_node/rgb/image_rect_color \
  /zed_right/zed_node/rgb/camera_info \
  /zed_right/zed_node/depth/depth_registered \
  /filter/imu/data \
  /odom \
  /tf \
  /tf_static \
  -o ~/bags/avl_slam_$(date +%Y%m%d_%H%M%S)
```

### Replay a bag (without running physical sensors)
```bash
# Terminal 1 — launch SLAM in sim_time mode
ros2 launch avl_slam slam.launch.py use_rviz:=false

# Terminal 2 — play the bag with clock
ros2 bag play ~/bags/<bag_name> --clock
```

---

## PART 8 — RTAB-Map Version Check

```bash
ros2 run rtabmap_slam rtabmap --version 2>&1 | grep "RTAB-Map:"
```
**Expected:** `RTAB-Map: 0.23.x`  
**If shows 0.22.1:** open a **fresh terminal** and source `~/avl_slam_ws/install/setup.bash`.

```bash
# RTAB-Map version written to database
ls -lh ~/.ros/rtabmap.db
```
If you need to reset: `rm ~/.ros/rtabmap.db`