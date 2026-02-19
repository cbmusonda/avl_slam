"""
AVL SLAM Launch File
Sensors: Velodyne VLP-16 | ZED X | Xsens IMU
SLAM:    RTAB-Map (LiDAR + RGB-D + IMU fusion)
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
    use_rviz_arg         = DeclareLaunchArgument('use_rviz',         default_value='true')
    localization_arg     = DeclareLaunchArgument('localization',     default_value='false',
                              description='true=localization only, false=mapping+localization')
    velodyne_ip_arg      = DeclareLaunchArgument('velodyne_ip',      default_value='192.168.13.11')
    velodyne_port_arg    = DeclareLaunchArgument('velodyne_port',    default_value='2368')

    use_rviz         = LaunchConfiguration('use_rviz')
    localization     = LaunchConfiguration('localization')
    velodyne_ip      = LaunchConfiguration('velodyne_ip')
    velodyne_port    = LaunchConfiguration('velodyne_port')

    # ── Paths ─────────────────────────────────────────────────────────
    avl_slam_share = FindPackageShare('avl_slam')

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
        parameters=[PathJoinSubstitution([avl_slam_share, 'config', 'vlp16.yaml'])],
        output='screen',
    )

    # ── 2. Intel RealSense D455 (DISABLED — using ZED X instead) ─────
    # realsense_share   = FindPackageShare('realsense2_camera')
    # realsense_serial_arg = DeclareLaunchArgument('realsense_serial', default_value='')
    # realsense_serial = LaunchConfiguration('realsense_serial')
    # realsense = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([realsense_share, 'launch', 'rs_launch.py'])
    #     ]),
    #     launch_arguments={
    #         'serial_no':           realsense_serial,
    #         'enable_color':        'true',
    #         'enable_depth':        'true',
    #         'enable_infra1':       'false',
    #         'enable_infra2':       'false',
    #         'enable_gyro':         'false',
    #         'enable_accel':        'false',
    #         'unite_imu_method':    'linear_interpolation',
    #         'depth_module.profile':'640x480x30',
    #         'rgb_camera.profile':  '640x480x30',
    #         'align_depth.enable':  'true',
    #         'pointcloud.enable':   'false',
    #         'base_frame_id':       'camera_link',
    #     }.items()
    # )

    # ── 2. ZED X Camera ───────────────────────────────────────────────
    zed = Node(
        package='zed_wrapper',
        executable='zed_wrapper',
        name='zed',
        output='screen',
        parameters=[{
            'camera_model': 'zedx',
            'publish_tf': False,
            'publish_map_tf': False,
            'base_frame': 'base_link',
            'camera_frame': 'zed_camera_center',
            'general.grab_resolution': 'HD720',
            'general.grab_frame_rate': 30,
            'depth.depth_mode': 'PERFORMANCE',
            'depth.min_depth': 0.3,
            'depth.max_depth': 20.0,
        }],
        remappings=[],
    )

    # ── 3. Xsens IMU Driver ───────────────────────────────────────────
    xsens_imu = Node(
        package='ros2_xsens_mti_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        parameters=[PathJoinSubstitution([avl_slam_share, 'config', 'xsens.yaml'])],
        output='screen',
    )

    # ── 4. IMU Filter (Madgwick) ──────────────────────────────────────
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

    # ── 5. RTAB-Map SLAM ──────────────────────────────────────────────
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
            # ZED X RGB-D
            ('rgb/image',       '/zed/zed_node/rgb/image_rect_color'),
            ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
            ('depth/image',     '/zed/zed_node/depth/depth_registered'),
            # IMU
            ('imu',             '/imu/filtered'),
        ],
        arguments=['--delete_db_on_start'] if False else [],
    )

    # ── 6. RTAB-Map Visualization ─────────────────────────────────────
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[PathJoinSubstitution([avl_slam_share, 'config', 'rtabmap.yaml'])],
        remappings=[
            ('scan_cloud',      '/velodyne_points'),
            ('rgb/image',       '/zed/zed_node/rgb/image_rect_color'),
            ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
            ('depth/image',     '/zed/zed_node/depth/depth_registered'),
            ('imu',             '/imu/filtered'),
        ],
        condition=IfCondition(use_rviz),
    )

    # ── 7. Static TFs ─────────────────────────────────────────────────
    tf_base_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_velodyne',
        arguments=['0.75', '0.0', '0.3',
                   '0', '0', '0',
                   'base_link', 'velodyne'],
    )

    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera',
        arguments=['0.35', '0.0', '0.75',
                   '0', '0', '0',
                   'base_link', 'zed_camera_center'],
    )

    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=['0.7', '0.0', '0.0',
                   '0', '0', '0',
                   'base_link', 'imu_link'],
    )

    return LaunchDescription([
        # Args
        use_rviz_arg,
        localization_arg,
        velodyne_ip_arg,
        velodyne_port_arg,
        # Nodes
        velodyne_driver,
        velodyne_convert,
        zed,
        xsens_imu,
        imu_filter,
        tf_base_to_velodyne,
        tf_base_to_camera,
        tf_base_to_imu,
        rtabmap,
        rtabmap_viz,
    ])