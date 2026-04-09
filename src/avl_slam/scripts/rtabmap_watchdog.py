#!/usr/bin/env python3
"""
RTAB-Map Watchdog Node

Monitors /realsense_front/rgbd_image and dynamically switches RTAB-Map
between two modes:
  - 3-camera: RealSense + ZED Left + ZED Back  (when RealSense is alive)
  - 2-camera: ZED Left + ZED Back only          (when RealSense drops)

Debounce:
  - RealSense must be silent for 'dead_timeout'  seconds before switching to ZED-only
  - RealSense must be alive  for 'alive_timeout' seconds before switching back to 3-cam
"""

import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rtabmap_msgs.msg import RGBDImage


class RTABMapWatchdog(Node):

    def __init__(self):
        super().__init__('rtabmap_watchdog')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('dead_timeout',   5.0)
        self.declare_parameter('alive_timeout',  10.0)
        self.declare_parameter('db_save_wait',   5.0)
        self.declare_parameter('database_path',  os.path.expanduser('~/.ros/rtabmap.db'))
        self.declare_parameter('rtabmap_yaml',
            '/home/dinosaur/avl_slam_ws/install/avl_slam/share/avl_slam/config/rtabmap.yaml')
        self.declare_parameter('icp_max_translation', '3.0')

        self.dead_timeout       = self.get_parameter('dead_timeout').value
        self.alive_timeout      = self.get_parameter('alive_timeout').value
        self.db_save_wait       = self.get_parameter('db_save_wait').value
        self.database_path      = self.get_parameter('database_path').value
        self.rtabmap_yaml       = self.get_parameter('rtabmap_yaml').value
        self.icp_max_translation = self.get_parameter('icp_max_translation').value

        # ── State ────────────────────────────────────────────────────────
        self.last_rs_time    = None   # wall-clock time of last RS message
        self.rs_alive        = False
        self.current_mode    = None   # '3cam' | 'zed2'
        self.rtabmap_proc    = None
        self.alive_since     = None   # when RS came back (for alive debounce)

        # ── ROS ──────────────────────────────────────────────────────────
        self.create_subscription(
            RGBDImage,
            '/realsense_front/rgbd_image',
            self._rs_callback,
            10,
        )
        self.create_timer(1.0, self._check_health)

        self.get_logger().info('Watchdog starting — launching 3-camera mode')
        self._launch_rtabmap('3cam')

    # ── RealSense topic callback ─────────────────────────────────────────

    def _rs_callback(self, _msg):
        self.last_rs_time = time.monotonic()
        if not self.rs_alive:
            self.rs_alive = True
            self.alive_since = time.monotonic()
            self.get_logger().info('RealSense is online')

    # ── Health check timer ───────────────────────────────────────────────

    def _check_health(self):
        now = time.monotonic()

        # Update alive/dead status
        if self.last_rs_time is not None:
            age = now - self.last_rs_time
            if age > self.dead_timeout and self.rs_alive:
                self.rs_alive = False
                self.alive_since = None
                self.get_logger().warn(
                    f'RealSense silent for {age:.1f}s — marked dead'
                )

        # 3-cam → zed2: RealSense has been dead long enough
        if self.current_mode == '3cam' and not self.rs_alive:
            dead_for = (now - self.last_rs_time) if self.last_rs_time else float('inf')
            if dead_for > self.dead_timeout:
                self.get_logger().warn(
                    f'RealSense dead {dead_for:.1f}s — switching to ZED-only mode'
                )
                self._switch_mode('zed2')

        # zed2 → 3-cam: RealSense has been alive long enough (debounce flapping)
        elif self.current_mode == 'zed2' and self.rs_alive and self.alive_since:
            alive_for = now - self.alive_since
            if alive_for > self.alive_timeout:
                self.get_logger().info(
                    f'RealSense stable for {alive_for:.1f}s — switching back to 3-camera mode'
                )
                self._switch_mode('3cam')

    # ── Mode switch ──────────────────────────────────────────────────────

    def _switch_mode(self, new_mode):
        self._stop_rtabmap()
        self.get_logger().info(f'Waiting {self.db_save_wait:.0f}s for database save...')
        time.sleep(self.db_save_wait)
        self._launch_rtabmap(new_mode)

    # ── Launch RTAB-Map subprocess ───────────────────────────────────────

    def _launch_rtabmap(self, mode):
        if mode == '3cam':
            rgbd_cameras = '3'
            topic_remaps = [
                '-r', 'rgbd_image0:=/realsense_front/rgbd_image',
                '-r', 'rgbd_image1:=/zed_left/rgbd_image',
                '-r', 'rgbd_image2:=/zed_back/rgbd_image',
            ]
        else:  # zed2
            rgbd_cameras = '2'
            topic_remaps = [
                '-r', 'rgbd_image0:=/zed_left/rgbd_image',
                '-r', 'rgbd_image1:=/zed_back/rgbd_image',
            ]

        cmd = [
            'ros2', 'run', 'rtabmap_slam', 'rtabmap',
            '--ros-args',
            '--params-file', self.rtabmap_yaml,
            '-p', f'database_path:={self.database_path}',
            '-p', 'Mem/IncrementalMemory:=true',
            '-p', 'subscribe_rgb:=false',
            '-p', 'subscribe_depth:=false',
            '-p', 'subscribe_rgbd:=true',
            '-p', f'rgbd_cameras:={rgbd_cameras}',
            '-p', 'approx_sync:=true',
            '-p', 'approx_sync_max_interval:=0.5',
            '-p', 'topic_queue_size:=30',
            '-p', 'sync_queue_size:=30',
            '-r', 'scan_cloud:=/velodyne_points',
            '-r', 'imu:=/imu/data',
        ] + topic_remaps

        self.rtabmap_proc = subprocess.Popen(cmd)
        self.current_mode = mode
        self.get_logger().info(
            f'RTAB-Map started in {mode} mode (PID {self.rtabmap_proc.pid})'
        )

    # ── Stop RTAB-Map subprocess ─────────────────────────────────────────

    def _stop_rtabmap(self):
        if self.rtabmap_proc is None:
            return
        if self.rtabmap_proc.poll() is not None:
            self.rtabmap_proc = None
            return

        self.get_logger().info(
            f'Stopping RTAB-Map (PID {self.rtabmap_proc.pid}) cleanly...'
        )
        self.rtabmap_proc.send_signal(signal.SIGINT)
        try:
            self.rtabmap_proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.get_logger().warn('Clean stop timed out — force killing')
            self.rtabmap_proc.kill()
            self.rtabmap_proc.wait()
        self.rtabmap_proc = None

    # ── Cleanup on shutdown ──────────────────────────────────────────────

    def destroy_node(self):
        self._stop_rtabmap()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RTABMapWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
