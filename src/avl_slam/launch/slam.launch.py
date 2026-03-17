"""
AVL SLAM Launch File
Sensors: Velodyne VLP-16 | ZED X Left | ZED X Right | ZED X Back | Intel RealSense D455 | Xsens IMU
SLAM:    RTAB-Map (LiDAR + RGB-D + IMU fusion)

RGB-D source selection:
  use_realsense:=true   → RealSense D455  (topics: /camera/camera/...)
  use_realsense:=false  → ZED X Left      (topics: /zed_left/zed_node/...)

All ZED X cameras always launch (left, right & back). The use_realsense flag
only controls which camera feeds into RTAB-Map as the primary RGB-D input.

RealSense D455 prerequisites (see docs/realsense_d455_setup.tex):
  - librealsense 2.57.6 built from source (RSUSB backend)
  - D455 firmware downgraded to 5.13.0.50
  - apt ros-humble-librealsense2 REMOVED
  - realsense-ros 4.56.4 built from source in this workspace
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
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
                            description='true=RealSense D455 as primary RGB-D, false=ZED X Left')
    realsense_serial_arg = DeclareLaunchArgument('realsense_serial', default_value='',
                            description='D455 serial number (leave empty for first available)')

    use_rviz        = LaunchConfiguration('use_rviz')
    velodyne_ip     = LaunchConfiguration('velodyne_ip')
    velodyne_port   = LaunchConfiguration('velodyne_port')
    use_realsense   = LaunchConfiguration('use_realsense')
    realsense_serial = LaunchConfiguration('realsense_serial')

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
    realsense_share = FindPackageShare('realsense2_camera')

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([realsense_share, 'launch', 'rs_launch.py'])
        ]),
        launch_arguments={
            'camera_name':          'camera',
            'serial_no':            realsense_serial,
            'enable_color':         'true',
            'enable_depth':         'true',
            'enable_infra1':        'false',
            'enable_infra2':        'false',
            'enable_gyro':          'false',         # D455 IMU broken with RSUSB backend
            'enable_accel':         'false',         # use Xsens external IMU instead
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile':   '640x480x30',
            'align_depth.enable':   'true',
            'pointcloud.enable':    'false',
            # base_frame_id defaults to 'link', which becomes 'camera_link'
            # after the camera_name prefix is applied (camera_name + "_" + base_frame_id)
            # Do NOT set base_frame_id to 'camera_link' — that produces 'camera_camera_link'!
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
        }.items()
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
        }.items()
    )

    # ── 5. ZED X Back Camera (rear side of vehicle) ───────────────────
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
        }.items()
    )

    # ── 6. Xsens IMU Driver ───────────────────────────────────────────
    xsens_imu = Node(
        package='ros2_xsens_mti_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        parameters=[PathJoinSubstitution([avl_slam_share, 'config', 'xsens.yaml'])],
        output='screen',
    )

    # ── 7. IMU ────────────────────────────────────────────────────────
    # Xsens MTi-680G has onboard sensor fusion — publishes fused
    # orientation directly on /imu/data. No external filter needed.

    # ── 8. RTAB-Map SLAM ──────────────────────────────────────────────
    # Two variants: one for RealSense topics, one for ZED X Left topics.
    # Only one launches based on use_realsense flag.

    rtabmap_common_params = [
        PathJoinSubstitution([avl_slam_share, 'config', 'rtabmap.yaml']),
        {'Mem/IncrementalMemory': 'true'},
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
            ('imu',             '/imu/data'),
        ],
        condition=IfCondition(use_realsense),
    )

    # RTAB-Map with ZED X Left as primary RGB-D
    rtabmap_zed = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=rtabmap_common_params,
        remappings=[
            ('scan_cloud',      '/velodyne_points'),
            ('rgb/image',       '/zed_left/zed_node/rgb/image_rect_color'),
            ('rgb/camera_info', '/zed_left/zed_node/rgb/camera_info'),
            ('depth/image',     '/zed_left/zed_node/depth/depth_registered'),
            ('imu',             '/imu/data'),
        ],
        condition=UnlessCondition(use_realsense),
    )

    # ── 9. RTAB-Map Visualization ─────────────────────────────────────
    # Two variants matching the SLAM node remappings above.

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
            ('imu',             '/imu/data'),
        ],

    )

    rtabmap_viz_zed = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[PathJoinSubstitution([avl_slam_share, 'config', 'rtabmap.yaml'])],
        remappings=[
            ('scan_cloud',      '/velodyne_points'),
            ('rgb/image',       '/zed_left/zed_node/rgb/image_rect_color'),
            ('rgb/camera_info', '/zed_left/zed_node/rgb/camera_info'),
            ('depth/image',     '/zed_left/zed_node/depth/depth_registered'),
            ('imu',             '/imu/data'),
        ],

    )

    # ── 10. Static TFs ────────────────────────────────────────────────
    # IMPORTANT: Update x/y/z offsets to match your actual physical
    #            sensor positions measured from base_link.

    tf_base_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_velodyne',
        arguments=['0.75', '0.0', '0.3',
                   '0', '0', '0',
                   'base_link', 'velodyne'],
    )

    # ZED X Left — yaw +90° so camera faces left
    tf_base_to_zed_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_zed_left',
        arguments=['-0.6', '0.35', '0.6',
                   '0', '0', '1.5708',
                   'base_link', 'zed_left_camera_center'],
    )

    # ZED X Right — yaw -90° so camera faces right
    tf_base_to_zed_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_zed_right',
        arguments=['0.6', '0.35', '0.6',
                   '0', '0', '-1.5708',
                   'base_link', 'zed_right_camera_center'],
    )

    # ZED X Back — yaw 180° so camera faces backward
    tf_base_to_zed_back = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_zed_back',
        arguments=['-0.75', '0.0', '-0.6',
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
    # Both ZED cameras always launch. RTAB-Map remappings switch based
    # on use_realsense. The viz node also switches to match.

    nodes = [
        # Args
        use_rviz_arg,
        localization_arg,
        velodyne_ip_arg,
        velodyne_port_arg,
        use_realsense_arg,
        realsense_serial_arg,
        # Velodyne + ICP odometry (always)
        velodyne_driver,
        velodyne_convert,
        icp_odometry,
        # RealSense D455 (conditional)
        realsense,
        # ZED X cameras (always — left, right, rear)
        zed_left,
        zed_right,
        zed_back,
        # IMU (always)
        xsens_imu,
        # Static TFs
        tf_base_to_velodyne,
        tf_base_to_zed_left,
        tf_base_to_zed_right,
        tf_base_to_zed_back,
        tf_base_to_imu,
        tf_base_to_camera,
        # RTAB-Map SLAM (one or the other based on use_realsense)
        rtabmap_realsense,
        rtabmap_zed,
    ]

    # Viz — launch matching variant only when use_rviz=true
    nodes.append(GroupAction(
        condition=IfCondition(use_rviz),
        actions=[
            GroupAction(
                actions=[rtabmap_viz_realsense],
                condition=IfCondition(use_realsense),
            ),
            GroupAction(
                actions=[rtabmap_viz_zed],
                condition=UnlessCondition(use_realsense),
            ),
        ],
    ))

    return LaunchDescription(nodes)