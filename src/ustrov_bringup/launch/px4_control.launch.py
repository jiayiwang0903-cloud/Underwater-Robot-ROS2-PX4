#!/usr/bin/env python3
"""PX4 实机模式最小 bringup。

启动链路:
  imu_bridge + dvl_bridge + depth_bridge  ->  EKF  ->  px4_main (px4 backend controller)

前置条件:
  - Micro XRCE-DDS Agent 已运行
  - PX4 飞控已启动
  - DVL A50 传感器已连接

用法:
  ros2 launch ustrov_bringup px4_control.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory('ustrov_bringup')
    ekf_config = os.path.join(bringup_share, 'config', 'ekf_real.yaml')

    imu_bridge = Node(
        package='ustrov_sensor_bridge',
        executable='imu_bridge',
        name='imu_bridge',
        output='screen',
    )

    dvl_bridge = Node(
        package='ustrov_sensor_bridge',
        executable='dvl_bridge',
        name='dvl_bridge',
        output='screen',
    )

    depth_bridge = Node(
        package='ustrov_sensor_bridge',
        executable='depth_bridge',
        name='depth_bridge',
        output='screen',
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )

    controller = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ustrov_bringup',
                executable='px4_main',
                name='ustrov_controller',
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        imu_bridge,
        dvl_bridge,
        depth_bridge,
        ekf,
        controller,
    ])
