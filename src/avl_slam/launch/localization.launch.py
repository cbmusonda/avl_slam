"""
AVL Localization-Only Launch
Use after a map has been built with slam.launch.py.
Loads the saved rtabmap.db and runs in localization mode.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    db_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value=os.path.expanduser('~/.ros/rtabmap.db'),
        description='Path to saved RTAB-Map database'
    )

    avl_slam_share = FindPackageShare('avl_slam')

    rtabmap_localize = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            PathJoinSubstitution([avl_slam_share, 'config', 'rtabmap.yaml']),
            {
                'Mem/IncrementalMemory': 'false',   # localization only
                'Mem/InitWMWithAllNodes': 'true',   # load full map into working memory
                'database_path': LaunchConfiguration('database_path'),
            }
        ],
        remappings=[
            ('scan_cloud',      '/velodyne_points'),
            ('rgb/image',       '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image',     '/camera/aligned_depth_to_color/image_raw'),
            ('imu',             '/imu/filtered'),
        ],
    )

    return LaunchDescription([
        db_path_arg,
        rtabmap_localize,
    ])
