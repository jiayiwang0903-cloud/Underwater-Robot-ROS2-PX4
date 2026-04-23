#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
实机手柄控制 launch

这个 launch 的定位是：
1. 先复用 hw_bringup.launch.py，把基础链路接好
2. 再启动 joy_node
3. 再启动 manual_control

注意：
- 这里不再重复启动 monitor_px4
- 因为 monitor 只在 hw_bringup 里单独使用
- manual_control 本身负责 Offboard 心跳、手柄控制、手动解锁
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
     
    # 把基础 bringup 需要的参数透传下来
     

    start_dvl_driver_arg = DeclareLaunchArgument(
        'start_dvl_driver',
        default_value='false',
        description='是否启动 DVL 驱动'
    )

    enable_dvl_bridge_arg = DeclareLaunchArgument(
        'enable_dvl_bridge',
        default_value='false',
        description='是否启动 DVL bridge'
    )

    dvl_ip_arg = DeclareLaunchArgument(
        'dvl_ip',
        default_value='192.168.194.95',
        description='DVL IP 地址'
    )

    ekf_config_arg = DeclareLaunchArgument(
        'ekf_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('ustrov_bringup'),
            'config',
            'ekf_real.yaml'
        ]),
        description='EKF 配置文件路径'
    )

     
    # 手柄驱动 joy_node
     
    # /joy 话题由它提供
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

     
    # 手柄控制节点
     
    # manual_control 负责：
    # 1. 发送 Offboard 心跳
    # 2. 接收 /joy
    # 3. Start 键解锁，Back 键上锁
    # 4. 直接给 PX4 actuator 发指令
    manual_control = Node(
        package='ustrov_px4',
        executable='manual_control',
        name='manual_control',
        output='screen',
    )

    return LaunchDescription([
        start_dvl_driver_arg,
        enable_dvl_bridge_arg,
        dvl_ip_arg,
        ekf_config_arg,

        joy_node,
        manual_control,
    ])