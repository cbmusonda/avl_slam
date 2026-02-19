"""
AVL SLAM Launch File
Sensors: Velodyne VLP-16 | ZED X Left | ZED X Right | Xsens IMU
SLAM:    RTAB-Map (LiDAR + RGB-D + IMU fusion)

NOTE: Intel RealSense D455 is commented out pending ROS connection fix.
      Two ZED X cameras are mounted on the left and right sides of the vehicle.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
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

    use_rviz      = LaunchConfiguration('use_rviz')
    velodyne_ip   = LaunchConfiguration('velodyne_ip')
    velodyne_port = LaunchConfiguration('velodyne_port')

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
            'timestamp_first_packet': True,
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
        executable='velodyne_transform_node',   # ← make sure this says transform_node
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
        {'Icp/MaxTranslation': '1.0'},   # ← override specifically for odom node
        {'Odom/Strategy': '0'},
    ],
        remappings=[
            ('scan_cloud', '/velodyne_points'),
            ('odom',       '/odom'),
        ],
    )

    # ── 2. Intel RealSense D455 — COMMENTED OUT (pending ROS connection fix) ──
    # TODO: Re-enable once RealSense ROS connection is resolved.
    #
    # realsense_share      = FindPackageShare('realsense2_camera')
    # realsense_serial_arg = DeclareLaunchArgument('realsense_serial', default_value='')
    # realsense_serial     = LaunchConfiguration('realsense_serial')
    #
    # realsense = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([realsense_share, 'launch', 'rs_launch.py'])
    #     ]),
    #     launch_arguments={
    #         'serial_no':            realsense_serial,
    #         'enable_color':         'true',
    #         'enable_depth':         'true',
    #         'enable_infra1':        'false',
    #         'enable_infra2':        'false',
    #         'enable_gyro':          'false',
    #         'enable_accel':         'false',
    #         'unite_imu_method':     'linear_interpolation',
    #         'depth_module.profile': '640x480x30',
    #         'rgb_camera.profile':   '640x480x30',
    #         'align_depth.enable':   'true',
    #         'pointcloud.enable':    'false',
    #         'base_frame_id':        'camera_link',
    #     }.items()
    # )
    #
    # rtabmap RealSense remappings (re-enable alongside node above):
    # ('rgb/image',       '/camera/color/image_raw'),
    # ('rgb/camera_info', '/camera/color/camera_info'),
    # ('depth/image',     '/camera/aligned_depth_to_color/image_raw'),

    # ── 3. ZED X Left Camera (left side of vehicle) ───────────────────
    # Uses the official zed_camera.launch.py from the ZED ROS 2 wrapper.
    # ros_params_override_path layers our custom zed_left.yaml on top of
    # the wrapper's default config so our settings take precedence.
    zed_left = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([zed_launch_dir, 'zed_camera.launch.py'])
        ]),
        launch_arguments={
            'camera_model':             'zedx',
            'camera_name':              'zed_left',
            'node_name':                'zed_node',
            'serial_number':            '43779087',        # ← SET THIS to your left ZED X serial number
            'publish_tf':               'false',    # RTAB-Map handles TF
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
            'serial_number':            '47753729',        # ← SET THIS to your right ZED X serial number
            'publish_tf':               'false',    # RTAB-Map handles TF
            'publish_map_tf':           'false',
            'publish_imu_tf':           'false',
            'publish_urdf':             'false',
            'ros_params_override_path': PathJoinSubstitution(
                [avl_slam_share, 'config', 'zed_right.yaml']
            ),
        }.items()
    )

    # ── 5. Xsens IMU Driver ───────────────────────────────────────────
    xsens_imu = Node(
        package='ros2_xsens_mti_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        parameters=[PathJoinSubstitution([avl_slam_share, 'config', 'xsens.yaml'])],
        output='screen',
    )

    # ── 6. IMU Filter (Madgwick) ──────────────────────────────────────
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[PathJoinSubstitution([avl_slam_share, 'config', 'imu_filter.yaml'])],
        remappings=[
            ('/imu/data_raw', '/imu/data'),
            ('/imu/data',     '/imu/filtered'),
        ],
        output='screen',
    )

    # ── 7. RTAB-Map SLAM ──────────────────────────────────────────────
    # Primary RGB-D input: ZED X Left camera (/zed_left/zed_node/...)
    # Right camera runs and publishes but is not yet fused into RTAB-Map.
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            PathJoinSubstitution([avl_slam_share, 'config', 'rtabmap.yaml']),
            {'Mem/IncrementalMemory': 'true'},
        ],
        remappings=[
            # LiDAR
            ('scan_cloud',      '/velodyne_points'),
            # ZED X Left — primary RGB-D input
            ('rgb/image',       '/zed_left/zed_node/rgb/image_rect_color'),
            ('rgb/camera_info', '/zed_left/zed_node/rgb/camera_info'),
            ('depth/image',     '/zed_left/zed_node/depth/depth_registered'),
            # IMU
            ('imu',             '/imu/filtered'),
        ],
        arguments=['--delete_db_on_start'] if False else [],
    )

    # ── 8. RTAB-Map Visualization ─────────────────────────────────────
    rtabmap_viz = Node(
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
            ('imu',             '/imu/filtered'),
        ],
        condition=IfCondition(use_rviz),
    )

    # ── 9. Static TFs ─────────────────────────────────────────────────
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

    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=['0.7', '0.0', '0.0',
                   '0', '0', '0',
                   'base_link', 'imu_link'],
    )

    # ── Old RealSense TF — COMMENTED OUT ──────────────────────────────
    # tf_base_to_camera = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='tf_base_to_camera',
    #     arguments=['0.35', '0.0', '0.75',
    #                '0', '0', '0',
    #                'base_link', 'camera_link'],
    # )

    return LaunchDescription([
        # Args
        use_rviz_arg,
        localization_arg,
        velodyne_ip_arg,
        velodyne_port_arg,
        # Nodes
        velodyne_driver,
        velodyne_convert,
        icp_odometry,
        zed_left,
        zed_right,
        xsens_imu,
        imu_filter,
        tf_base_to_velodyne,
        tf_base_to_zed_left,
        tf_base_to_zed_right,
        tf_base_to_imu,
        rtabmap,
        rtabmap_viz,
    ])