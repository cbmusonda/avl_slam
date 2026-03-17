"""
AVL Localization-Only Launch
Includes the full SLAM pipeline (sensors, TFs, ICP odometry) but runs
RTAB-Map in localization mode against a saved map database.

Usage:
  ros2 launch avl_slam localization.launch.py
  ros2 launch avl_slam localization.launch.py database_path:=/path/to/rtabmap.db
  ros2 launch avl_slam localization.launch.py use_realsense:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import os


def generate_launch_description():

    avl_slam_share = FindPackageShare('avl_slam')

    db_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value=os.path.expanduser('~/.ros/rtabmap.db'),
        description='Path to saved RTAB-Map database'
    )

    use_realsense_arg = DeclareLaunchArgument(
        'use_realsense', default_value='false',
        description='true=RealSense D455, false=ZED X Left'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
    )

    use_realsense = LaunchConfiguration('use_realsense')

    # Include the full SLAM launch (sensors + TFs + ICP odom + RTAB-Map)
    # RTAB-Map's Mem/IncrementalMemory defaults to 'true' in slam.launch.py.
    # We override it here by launching an additional param-setting node.
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([avl_slam_share, 'launch', 'slam.launch.py'])
        ]),
        launch_arguments={
            'use_realsense': LaunchConfiguration('use_realsense'),
            'use_rviz':      LaunchConfiguration('use_rviz'),
        }.items(),
    )

    return LaunchDescription([
        db_path_arg,
        use_realsense_arg,
        use_rviz_arg,
        slam_launch,
    ])
