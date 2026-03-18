"""
AVL SLAM Launch File
Sensors: Velodyne VLP-16 | ZED X Left | ZED X Right | ZED X Back | Intel RealSense D455 | Xsens IMU
SLAM:    RTAB-Map (LiDAR-ICP odometry + RGB-D loop closure + IMU fusion)

RGB-D source selection:
  use_realsense:=false (default) → ZED X Right  (reliable camera; topics: /zed_right/zed_node/...)
  use_realsense:=true            → RealSense D455 (topics: /camera/camera/...)

All ZED X cameras always launch (left, right & back). The use_realsense flag
only controls which camera feeds into RTAB-Map as the primary RGB-D input.

RealSense D455 prerequisites (see docs/realsense_d455_setup.tex):
  - librealsense 2.57.6 built from source (RSUSB backend)
  - D455 firmware downgraded to 5.13.0.50
  - apt ros-humble-librealsense2 REMOVED
  - realsense-ros 4.56.4 built from source in this workspace

FIX LOG (vs previous version):
  1. IMU topic remap: /imu/data → /filter/imu/data (DEMCON driver publishes here)
  2. RealSense profile params: depth_module.profile → depth_module.depth_profile
                               rgb_camera.profile   → rgb_camera.color_profile
  3. RealSense initial_reset:=true added to prevent USB EAGAIN race on boot
  4. Default RGB-D source swapped from unreliable ZED left → ZED right
  5. rtabmap_viz condition guards added (was launching unconditioned)
  6. imu_tf_prefix removed from rtabmap.yaml (not a valid param — see config/)
  7. sensors_image_sync set true in all ZED configs for depth/RGB alignment
  8. use_sim_time added to zed_back.yaml (was missing)
    9. Added GPU profile launch args (RealSense GLSL + RTAB-Map tuning defaults)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    # ── Launch Arguments ──────────────────────────────────────────────
    use_rviz_arg      = DeclareLaunchArgument('use_rviz',      default_value='true')
    localization_arg  = DeclareLaunchArgument('localization',  default_value='false',
                            description='true=localization only, false=mapping+localization')
    velodyne_ip_arg   = DeclareLaunchArgument('velodyne_ip',   default_value='192.168.13.11')
    velodyne_port_arg = DeclareLaunchArgument('velodyne_port', default_value='2368')
    use_realsense_arg = DeclareLaunchArgument('use_realsense', default_value='false',
                            description='true=RealSense D455 as primary RGB-D, false=ZED X Right')
    realsense_serial_arg = DeclareLaunchArgument('realsense_serial', default_value='',
                            description='D455 serial number (leave empty for first available)')
    database_path_arg = DeclareLaunchArgument('database_path',
                            default_value=os.path.expanduser('~/.ros/rtabmap.db'),
                            description='Path to RTAB-Map database file')
    use_multicamera_arg = DeclareLaunchArgument('use_multicamera', default_value='true',
                            description='true=fuse RealSense front + enabled ZED cameras via rgbd_sync')
    gpu_profile_arg = DeclareLaunchArgument('gpu_profile', default_value='true',
                            description='true=enable Jetson GPU profile (RealSense GLSL + lower RTAB-Map CPU load)')
    realsense_glsl_arg = DeclareLaunchArgument('realsense_glsl', default_value='true',
                            description='true=enable realsense2_camera accelerate_gpu_with_glsl (requires GLSL build flag)')
    rtabmap_max_features_arg = DeclareLaunchArgument('rtabmap_max_features', default_value='400',
                            description='RTAB-Map Vis/MaxFeatures when gpu_profile=true')
    rtabmap_icp_iterations_arg = DeclareLaunchArgument('rtabmap_icp_iterations', default_value='20',
                            description='RTAB-Map Icp/Iterations when gpu_profile=true')
    rtabmap_optimizer_iterations_arg = DeclareLaunchArgument('rtabmap_optimizer_iterations', default_value='60',
                            description='RTAB-Map Optimizer/Iterations when gpu_profile=true')
    use_xsens_arg = DeclareLaunchArgument('use_xsens', default_value='true',
                            description='true=launch Xsens IMU driver, false=skip it')
    use_zed_left_arg = DeclareLaunchArgument('use_zed_left', default_value='true')
    use_zed_right_arg = DeclareLaunchArgument('use_zed_right', default_value='true')
    use_zed_back_arg = DeclareLaunchArgument('use_zed_back', default_value='true')

    use_rviz         = LaunchConfiguration('use_rviz')
    velodyne_ip      = LaunchConfiguration('velodyne_ip')
    velodyne_port    = LaunchConfiguration('velodyne_port')
    use_realsense    = LaunchConfiguration('use_realsense')
    realsense_serial = LaunchConfiguration('realsense_serial')
    database_path    = LaunchConfiguration('database_path')
    use_multicamera  = LaunchConfiguration('use_multicamera')
    gpu_profile      = LaunchConfiguration('gpu_profile')
    realsense_glsl   = LaunchConfiguration('realsense_glsl')
    rtabmap_max_features = LaunchConfiguration('rtabmap_max_features')
    rtabmap_icp_iterations = LaunchConfiguration('rtabmap_icp_iterations')
    rtabmap_optimizer_iterations = LaunchConfiguration('rtabmap_optimizer_iterations')
    use_xsens        = LaunchConfiguration('use_xsens')
    use_zed_left     = LaunchConfiguration('use_zed_left')
    use_zed_right    = LaunchConfiguration('use_zed_right')
    use_zed_back     = LaunchConfiguration('use_zed_back')

    # ── Paths ─────────────────────────────────────────────────────────
    avl_slam_share = FindPackageShare('avl_slam')
    zed_launch_dir = PathJoinSubstitution([FindPackageShare('zed_wrapper'), 'launch'])

    # ── 1. Velodyne VLP-16 Driver ─────────────────────────────────────
    velodyne_driver = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver',
        parameters=[{
            'device_ip': velodyne_ip,
            'port': velodyne_port,
            'model': 'VLP16',
            'rpm': 600.0,
            'frame_id': 'velodyne',
            'timestamp_first_packet': False,
        }],
        output='screen',
    )

    velodyne_convert = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_convert',
        parameters=[
            PathJoinSubstitution([avl_slam_share, 'config', 'vlp16.yaml']),
            {'calibration': '/opt/ros/humble/share/velodyne_pointcloud/params/VLP16db.yaml'},
            {'organize_cloud': False},
            {'min_range': 0.4},
            {'max_range': 100.0},
        ],
        output='screen',
    )

    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[
            PathJoinSubstitution([avl_slam_share, 'config', 'rtabmap.yaml']),
            {'Icp/MaxTranslation': '1.0'},
            {'Odom/Strategy': '0'},
        ],
        remappings=[
            ('scan_cloud', '/velodyne_points'),
            ('odom',       '/odom'),
        ],
    )

    # ── 2. Intel RealSense D455 ───────────────────────────────────────
    # Launched conditionally when use_realsense:=true
    # Requires: librealsense 2.57.6 (source) + FW 5.13.0.50 + realsense-ros 4.56.4 (source)
    # D455 IMU is disabled — RSUSB backend on JetPack 6 cannot access HID.
    # Use the Xsens external IMU instead.
    #
    # FIX #2: depth_module.profile → depth_module.depth_profile
    #         rgb_camera.profile   → rgb_camera.color_profile  (realsense-ros 4.x API)
    # FIX #3: initial_reset:=true prevents USB EAGAIN race on Jetson boot
    realsense_share = FindPackageShare('realsense2_camera')

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([realsense_share, 'launch', 'rs_launch.py'])
        ]),
        launch_arguments={
            'camera_name':                 'camera',
            'serial_no':                   realsense_serial,
            'enable_color':                'true',
            'enable_depth':                'true',
            'enable_infra1':               'false',
            'enable_infra2':               'false',
            'enable_gyro':                 'false',        # D455 IMU broken with RSUSB backend
            'enable_accel':                'false',        # use Xsens external IMU instead
            'depth_module.depth_profile':  '640x480x30',  # FIX #2 (was depth_module.profile)
            'rgb_camera.color_profile':    '640x480x30',  # FIX #2 (was rgb_camera.profile)
            'align_depth.enable':          'true',
            'accelerate_gpu_with_glsl':    realsense_glsl,
            'pointcloud.enable':           'false',
            'initial_reset':               'true',         # FIX #3 — prevents EAGAIN on boot
            # base_frame_id intentionally left at default ('link') so the full
            # frame becomes 'camera_link' (camera_name + '_' + base_frame_id).
            # Do NOT set base_frame_id='camera_link' → produces 'camera_camera_link'.
        }.items(),
        condition=IfCondition(use_realsense),
    )

    # ── 3. ZED X Left Camera (left side of vehicle) ───────────────────
    zed_left = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([zed_launch_dir, 'zed_camera.launch.py'])
        ]),
        launch_arguments={
            'camera_model':             'zedx',
            'camera_name':              'zed_left',
            'node_name':                'zed_node',
            'serial_number':            '43779087',
            'publish_tf':               'false',
            'publish_map_tf':           'false',
            'publish_imu_tf':           'false',
            'publish_urdf':             'false',
            'ros_params_override_path': PathJoinSubstitution(
                [avl_slam_share, 'config', 'zed_left.yaml']
            ),
        }.items(),
        condition=IfCondition(use_zed_left),
    )

    # ── 4. ZED X Right Camera (right side of vehicle) ─────────────────
    zed_right = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([zed_launch_dir, 'zed_camera.launch.py'])
        ]),
        launch_arguments={
            'camera_model':             'zedx',
            'camera_name':              'zed_right',
            'node_name':                'zed_node',
            'serial_number':            '47753729',
            'publish_tf':               'false',
            'publish_map_tf':           'false',
            'publish_imu_tf':           'false',
            'publish_urdf':             'false',
            'ros_params_override_path': PathJoinSubstitution(
                [avl_slam_share, 'config', 'zed_right.yaml']
            ),
        }.items(),
        condition=IfCondition(use_zed_right),
    )

    # ── 5. ZED X Back Camera (rear of vehicle) ────────────────────────
    zed_back = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([zed_launch_dir, 'zed_camera.launch.py'])
        ]),
        launch_arguments={
            'camera_model':             'zedx',
            'camera_name':              'zed_back',
            'node_name':                'zed_node',
            'serial_number':            '49910017',
            'publish_tf':               'false',
            'publish_map_tf':           'false',
            'publish_imu_tf':           'false',
            'publish_urdf':             'false',
            'ros_params_override_path': PathJoinSubstitution(
                [avl_slam_share, 'config', 'zed_back.yaml']
            ),
        }.items(),
        condition=IfCondition(use_zed_back),
    )

    # ── 6. Xsens IMU Driver ───────────────────────────────────────────
    # The MTi-680G has onboard sensor fusion and publishes a fully fused
    # orientation + angular velocity + linear acceleration directly.
    # No external Madgwick filter is needed.
    #
    # Topic published by DEMCON ros2_xsens_mti_driver:
    #   /filter/imu/data  — sensor_msgs/Imu (fused: orientation + gyro + accel)
    #   /imu/data         — sensor_msgs/Imu (raw accel + gyro, NO orientation)
    #
    # FIX #1: RTAB-Map remaps must point to /filter/imu/data, NOT /imu/data.
    xsens_imu = Node(
        package='ros2_xsens_mti_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        parameters=[PathJoinSubstitution([avl_slam_share, 'config', 'xsens.yaml'])],
        output='screen',
        condition=IfCondition(use_xsens),
    )

    # ── 7. RGB-D Sync Nodes ───────────────────────────────────────────
    # Pre-sync each RGB + Depth + CameraInfo stream into a single RGBDImage.
    # This allows RTAB-Map to fuse multiple cameras in one graph.

    rgbd_sync_realsense = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync_realsense',
        namespace='realsense_front',
        output='screen',
        parameters=[{
            'approx_sync': True,
            'queue_size': 30,
        }],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
        ],
        condition=IfCondition(PythonExpression(["'", use_realsense, "' == 'true' and '", use_multicamera, "' == 'true'"])),
    )

    rgbd_sync_zed_left = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync_zed_left',
        namespace='zed_left',
        output='screen',
        parameters=[{
            'approx_sync': True,
            'queue_size': 30,
        }],
        remappings=[
            ('rgb/image', '/zed_left/zed_node/rgb/color/rect/image'),
            ('depth/image', '/zed_left/zed_node/depth/depth_registered'),
            ('rgb/camera_info', '/zed_left/zed_node/rgb/color/rect/camera_info'),
        ],
        condition=IfCondition(PythonExpression(["'", use_zed_left, "' == 'true' and '", use_multicamera, "' == 'true'"])),
    )

    rgbd_sync_zed_right = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync_zed_right',
        namespace='zed_right',
        output='screen',
        parameters=[{
            'approx_sync': True,
            'queue_size': 30,
        }],
        remappings=[
            ('rgb/image', '/zed_right/zed_node/rgb/color/rect/image'),
            ('depth/image', '/zed_right/zed_node/depth/depth_registered'),
            ('rgb/camera_info', '/zed_right/zed_node/rgb/color/rect/camera_info'),
        ],
        condition=IfCondition(PythonExpression(["'", use_zed_right, "' == 'true' and '", use_multicamera, "' == 'true'"])),
    )

    rgbd_sync_zed_back = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync_zed_back',
        namespace='zed_back',
        output='screen',
        parameters=[{
            'approx_sync': True,
            'queue_size': 30,
        }],
        remappings=[
            ('rgb/image', '/zed_back/zed_node/rgb/color/rect/image'),
            ('depth/image', '/zed_back/zed_node/depth/depth_registered'),
            ('rgb/camera_info', '/zed_back/zed_node/rgb/color/rect/camera_info'),
        ],
        condition=IfCondition(PythonExpression(["'", use_zed_back, "' == 'true' and '", use_multicamera, "' == 'true'"])),
    )

    # ── 8. RTAB-Map SLAM ──────────────────────────────────────────────
    # Two execution modes:
    #   1) single-camera mode (legacy): one RGB-D source selected by use_realsense
    #   2) multi-camera mode: RealSense front + enabled ZED streams via rgbd_sync

    rtabmap_common_params = [
        PathJoinSubstitution([avl_slam_share, 'config', 'rtabmap.yaml']),
        {'database_path': database_path},
        {'Mem/IncrementalMemory': 'true'},
        {
            'Vis/MaxFeatures': PythonExpression([
                "'", rtabmap_max_features, "' if '", gpu_profile, "' == 'true' else '600'"
            ]),
            'Icp/Iterations': PythonExpression([
                "'", rtabmap_icp_iterations, "' if '", gpu_profile, "' == 'true' else '30'"
            ]),
            'Optimizer/Iterations': PythonExpression([
                "'", rtabmap_optimizer_iterations, "' if '", gpu_profile, "' == 'true' else '100'"
            ]),
        },
    ]

    # RTAB-Map with RealSense D455 as primary RGB-D
    rtabmap_realsense = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=rtabmap_common_params,
        remappings=[
            ('scan_cloud',      '/velodyne_points'),
            ('rgb/image',       '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('depth/image',     '/camera/camera/aligned_depth_to_color/image_raw'),
            ('imu',             '/filter/imu/data'),   # FIX #1
        ],
        condition=IfCondition(PythonExpression(["'", use_realsense, "' == 'true' and '", use_multicamera, "' != 'true'"])),
    )

    # RTAB-Map with ZED X Right as primary RGB-D  (FIX #4: was ZED left)
    rtabmap_zed = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=rtabmap_common_params,
        remappings=[
            ('scan_cloud',      '/velodyne_points'),
            ('rgb/image',       '/zed_right/zed_node/rgb/color/rect/image'),
            ('rgb/camera_info', '/zed_right/zed_node/rgb/color/rect/camera_info'),
            ('depth/image',     '/zed_right/zed_node/depth/depth_registered'), # FIX #4
            ('imu',             '/filter/imu/data'),   # FIX #1
        ],
        condition=IfCondition(PythonExpression(["'", use_realsense, "' != 'true' and '", use_multicamera, "' != 'true' and '", use_zed_right, "' == 'true'"])),
    )

    rtabmap_multicam_4 = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            PathJoinSubstitution([avl_slam_share, 'config', 'rtabmap.yaml']),
            {
                'Mem/IncrementalMemory': 'true',
                'subscribe_rgb': False,
                'subscribe_depth': False,
                'subscribe_rgbd': True,
                'rgbd_cameras': 4,
                'approx_sync': True,
                'database_path': database_path,
            },
        ],
        remappings=[
            ('scan_cloud', '/velodyne_points'),
            ('imu', '/filter/imu/data'),
            ('rgbd_image0', '/realsense_front/rgbd_image'),
            ('rgbd_image1', '/zed_left/rgbd_image'),
            ('rgbd_image2', '/zed_right/rgbd_image'),
            ('rgbd_image3', '/zed_back/rgbd_image'),
        ],
        condition=IfCondition(PythonExpression([
            "'", use_multicamera, "' == 'true' and '", use_realsense,
            "' == 'true' and '", use_zed_left,
            "' == 'true' and '", use_zed_right,
            "' == 'true' and '", use_zed_back, "' == 'true'",
        ])),
    )

    rtabmap_multicam_3 = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            PathJoinSubstitution([avl_slam_share, 'config', 'rtabmap.yaml']),
            {
                'Mem/IncrementalMemory': 'true',
                'subscribe_rgb': False,
                'subscribe_depth': False,
                'subscribe_rgbd': True,
                'rgbd_cameras': 3,
                'approx_sync': True,
                'database_path': database_path,
            },
        ],
        remappings=[
            ('scan_cloud', '/velodyne_points'),
            ('imu', '/filter/imu/data'),
            ('rgbd_image0', '/realsense_front/rgbd_image'),
            ('rgbd_image1', '/zed_left/rgbd_image'),
            ('rgbd_image2', '/zed_back/rgbd_image'),
        ],
        condition=IfCondition(PythonExpression([
            "'", use_multicamera, "' == 'true' and '", use_realsense,
            "' == 'true' and '", use_zed_left,
            "' == 'true' and '", use_zed_right,
            "' != 'true' and '", use_zed_back, "' == 'true'",
        ])),
    )

    # ── 9. RTAB-Map Visualization ─────────────────────────────────────
    # FIX #5: both viz nodes now have condition guards matching the SLAM nodes.
    # Previously rtabmap_viz_realsense had no condition and always launched.

    rtabmap_viz_realsense = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[PathJoinSubstitution([avl_slam_share, 'config', 'rtabmap.yaml'])],
        remappings=[
            ('scan_cloud',      '/velodyne_points'),
            ('rgb/image',       '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('depth/image',     '/camera/camera/aligned_depth_to_color/image_raw'),
            ('imu',             '/filter/imu/data'),   # FIX #1
        ],
        condition=IfCondition(PythonExpression(["'", use_realsense, "' == 'true' and '", use_multicamera, "' != 'true'"])),
    )

    rtabmap_viz_zed = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[PathJoinSubstitution([avl_slam_share, 'config', 'rtabmap.yaml'])],
        remappings=[
            ('scan_cloud',      '/velodyne_points'),
            ('rgb/image',       '/zed_right/zed_node/rgb/color/rect/image'),
            ('rgb/camera_info', '/zed_right/zed_node/rgb/color/rect/camera_info'),
            ('depth/image',     '/zed_right/zed_node/depth/depth_registered'), # FIX #4
            ('imu',             '/filter/imu/data'),   # FIX #1
        ],
        condition=IfCondition(PythonExpression(["'", use_realsense, "' != 'true' and '", use_multicamera, "' != 'true' and '", use_zed_right, "' == 'true'"])),
    )

    rtabmap_viz_multicam_4 = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[
            PathJoinSubstitution([avl_slam_share, 'config', 'rtabmap.yaml']),
            {
                'subscribe_rgb': False,
                'subscribe_depth': False,
                'subscribe_rgbd': True,
                'rgbd_cameras': 4,
                'approx_sync': True,
            },
        ],
        remappings=[
            ('scan_cloud', '/velodyne_points'),
            ('imu', '/filter/imu/data'),
            ('rgbd_image0', '/realsense_front/rgbd_image'),
            ('rgbd_image1', '/zed_left/rgbd_image'),
            ('rgbd_image2', '/zed_right/rgbd_image'),
            ('rgbd_image3', '/zed_back/rgbd_image'),
        ],
        condition=IfCondition(PythonExpression([
            "'", use_multicamera, "' == 'true' and '", use_realsense,
            "' == 'true' and '", use_zed_left,
            "' == 'true' and '", use_zed_right,
            "' == 'true' and '", use_zed_back, "' == 'true'",
        ])),
    )

    rtabmap_viz_multicam_3 = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[
            PathJoinSubstitution([avl_slam_share, 'config', 'rtabmap.yaml']),
            {
                'subscribe_rgb': False,
                'subscribe_depth': False,
                'subscribe_rgbd': True,
                'rgbd_cameras': 3,
                'approx_sync': True,
            },
        ],
        remappings=[
            ('scan_cloud', '/velodyne_points'),
            ('imu', '/filter/imu/data'),
            ('rgbd_image0', '/realsense_front/rgbd_image'),
            ('rgbd_image1', '/zed_left/rgbd_image'),
            ('rgbd_image2', '/zed_back/rgbd_image'),
        ],
        condition=IfCondition(PythonExpression([
            "'", use_multicamera, "' == 'true' and '", use_realsense,
            "' == 'true' and '", use_zed_left,
            "' == 'true' and '", use_zed_right,
            "' != 'true' and '", use_zed_back, "' == 'true'",
        ])),
    )

    # ── 10. Static TFs ────────────────────────────────────────────────
    # IMPORTANT: Update x/y/z offsets to match your actual physical
    #            sensor positions measured from base_link.
    # Format: x y z roll pitch yaw parent child

    tf_base_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_velodyne',
        arguments=['0.75', '0.0', '0.3',
                   '0', '0', '0',
                   'base_link', 'velodyne'],
    )

    # ZED X Left — yaw +90° so camera faces left (+Y direction)
    tf_base_to_zed_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_zed_left',
        arguments=['-0.6', '0.35', '0.6',
                   '0', '0', '1.5708',
                   'base_link', 'zed_left_camera_center'],
    )

    # ZED X Right — yaw -90° so camera faces right (-Y direction)
    tf_base_to_zed_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_zed_right',
        arguments=['0.6', '0.35', '0.6',
                   '0', '0', '-1.5708',
                   'base_link', 'zed_right_camera_center'],
    )

    # ZED X Back — yaw 180° so camera faces backward (-X direction)
    tf_base_to_zed_back = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_zed_back',
        arguments=['-0.75', '0.0', '0.6',
                   '0', '0', '3.14159',
                   'base_link', 'zed_back_camera_center'],
    )

    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=['0.7', '0.0', '0.0',
                   '0', '0', '0',
                   'base_link', 'imu_link'],
    )

    # RealSense D455 — forward-facing, mounted center-front
    # Adjust x/y/z to match your actual D455 mount position
    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera',
        arguments=['0.35', '0.0', '0.75',
                   '0', '0', '0',
                   'base_link', 'camera_link'],
        condition=IfCondition(use_realsense),
    )

    # ── Build launch description ──────────────────────────────────────
    nodes = [
        # Args
        use_rviz_arg,
        localization_arg,
        velodyne_ip_arg,
        velodyne_port_arg,
        use_realsense_arg,
        realsense_serial_arg,
        database_path_arg,
        use_multicamera_arg,
        gpu_profile_arg,
        realsense_glsl_arg,
        rtabmap_max_features_arg,
        rtabmap_icp_iterations_arg,
        rtabmap_optimizer_iterations_arg,
        use_xsens_arg,
        use_zed_left_arg,
        use_zed_right_arg,
        use_zed_back_arg,
        # Velodyne + ICP odometry (always)
        velodyne_driver,
        velodyne_convert,
        icp_odometry,
        # RealSense D455 (conditional)
        realsense,
        # ZED X cameras (always — left, right, back)
        zed_left,
        zed_right,
        zed_back,
        # IMU (always)
        xsens_imu,
        # RGB-D sync nodes for multicamera mode
        rgbd_sync_realsense,
        rgbd_sync_zed_left,
        rgbd_sync_zed_right,
        rgbd_sync_zed_back,
        # Static TFs
        tf_base_to_velodyne,
        tf_base_to_zed_left,
        tf_base_to_zed_right,
        tf_base_to_zed_back,
        tf_base_to_imu,
        tf_base_to_camera,
        # RTAB-Map SLAM (conditional on use_realsense)
        rtabmap_realsense,
        rtabmap_zed,
        rtabmap_multicam_4,
        rtabmap_multicam_3,
    ]

    # Viz — launch matching variant only when use_rviz=true (FIX #5)
    nodes.append(GroupAction(
        condition=IfCondition(use_rviz),
        actions=[
            GroupAction(
                actions=[rtabmap_viz_realsense],
                condition=IfCondition(PythonExpression(["'", use_realsense, "' == 'true' and '", use_multicamera, "' != 'true'"])),
            ),
            GroupAction(
                actions=[rtabmap_viz_zed],
                condition=IfCondition(PythonExpression(["'", use_realsense, "' != 'true' and '", use_multicamera, "' != 'true' and '", use_zed_right, "' == 'true'"])),
            ),
            GroupAction(
                actions=[rtabmap_viz_multicam_4],
                condition=IfCondition(PythonExpression([
                    "'", use_multicamera, "' == 'true' and '", use_realsense,
                    "' == 'true' and '", use_zed_left,
                    "' == 'true' and '", use_zed_right,
                    "' == 'true' and '", use_zed_back, "' == 'true'",
                ])),
            ),
            GroupAction(
                actions=[rtabmap_viz_multicam_3],
                condition=IfCondition(PythonExpression([
                    "'", use_multicamera, "' == 'true' and '", use_realsense,
                    "' == 'true' and '", use_zed_left,
                    "' == 'true' and '", use_zed_right,
                    "' != 'true' and '", use_zed_back, "' == 'true'",
                ])),
            ),
        ],
    ))

    return LaunchDescription(nodes)