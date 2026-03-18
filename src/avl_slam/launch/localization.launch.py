"""
AVL Localization-Only Launch
Runs RTAB-Map in localization mode against a saved map database.
All sensors, TFs, and ICP odometry launch identically to slam.launch.py.
Mem/IncrementalMemory is overridden to false so no new nodes are added to the map.

Usage:
  ros2 launch avl_slam localization.launch.py
  ros2 launch avl_slam localization.launch.py database_path:=/path/to/rtabmap.db
  ros2 launch avl_slam localization.launch.py use_realsense:=true
  ros2 launch avl_slam localization.launch.py use_rviz:=false

FIX LOG (vs previous version):
  - database_path and localization:=true are now forwarded to slam.launch.py so
    RTAB-Map actually loads the saved database and enters localization mode.
    Previously only use_realsense and use_rviz were forwarded — the localization
    flag was declared but never passed down, so RTAB-Map always started in
    mapping mode and ignored the database_path argument.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
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
        description='true=RealSense D455, false=ZED X Right'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
    )

    # Include the full SLAM launch but with:
    #   localization:=true  → tells slam.launch.py to set Mem/IncrementalMemory=false
    #   database_path       → forwarded so RTAB-Map loads the correct .db file
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([avl_slam_share, 'launch', 'slam.launch.py'])
        ]),
        launch_arguments={
            'use_realsense':  LaunchConfiguration('use_realsense'),
            'use_rviz':       LaunchConfiguration('use_rviz'),
            'localization':   'true',                          # FIX: was never forwarded
            'database_path':  LaunchConfiguration('database_path'),  # FIX: was never forwarded
        }.items(),
    )

    return LaunchDescription([
        db_path_arg,
        use_realsense_arg,
        use_rviz_arg,
        slam_launch,
    ])