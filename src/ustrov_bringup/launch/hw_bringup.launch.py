#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
实机硬件基础 bringup（只接通链路，不启动任何控制模式）

这个 launch 的定位是：
1. 启动传感器桥接节点（IMU / Depth / 可选 DVL）
2. 启动 EKF，产出 /odometry/filtered
3. 启动监控面板 monitor_px4
4. 但不启动闭环控制器 px4_main
5. 也不启动手柄控制 manual_control

也就是说，这个 launch 只是把“硬件链路和状态估计链路”接好，
方便你后续再决定：
- 是进入手柄控制
- 还是进入目标跟踪控制
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
     
    # 可配置启动参数
    # 是否启动 DVL 驱动节点（dvl_a50）
    # 如果不接 DVL，保持 false
    start_dvl_driver_arg = DeclareLaunchArgument(
        'start_dvl_driver',
        default_value='false',
        description='是否启动 DVL A50 驱动节点'
    )

    enable_monitor_arg = DeclareLaunchArgument(
        'enable_monitor',
        default_value='true',
        description='是否启动 USTROV Sensor Monitor'
    )

    # 是否启动 DVL bridge
    # 如果不用 DVL，保持 false
    enable_dvl_bridge_arg = DeclareLaunchArgument(
        'enable_dvl_bridge',
        default_value='false',
        description='是否启动 dvl_bridge 节点'
    )

    # DVL 的 IP 地址
    dvl_ip_arg = DeclareLaunchArgument(
        'dvl_ip',
        default_value='192.168.194.95',
        description='DVL A50 的 IP 地址'
    )

    # EKF 参数文件路径
    # 如果你不用 DVL，请确保这里指向的 ekf_real.yaml 已经把 pose1 那块注释掉
    ekf_config_arg = DeclareLaunchArgument(
        'ekf_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('ustrov_bringup'),
            'config',
            'ekf_real.yaml'
        ]),
        description='EKF 配置文件路径'
    )

     
    # 固定启动的基础节点
    # IMU bridge：
    # 把 PX4 的 sensor_combined + vehicle_attitude
    # 转成 robot_localization 可融合的 /sensor/imu
    imu_bridge = Node(
        package='ustrov_sensor_bridge',
        executable='imu_bridge',
        name='imu_bridge',
        output='screen',
    )

    # Depth bridge：
    # 把 PX4 气压计 /fmu/out/sensor_baro
    # 转成 EKF 可融合的 /sensor/depth
    depth_bridge = Node(
        package='ustrov_sensor_bridge',
        executable='depth_bridge',
        name='depth_bridge',
        output='screen',
    )

    # EKF：
    # 读取 /sensor/imu、/sensor/depth、（可选）/sensor/dvl_pose
    # 产出统一状态话题 /odometry/filtered
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('ekf_config')],
    )

    # Monitor只在这个基础 bringup里启动
    # 用于查看 depth / dvl / odom 是否通
    monitor = Node(
        package='ustrov_px4',
        executable='monitor_px4',
        name='monitor_px4',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_monitor')),
    )

     
    # 可选启动：DVL 驱动
     

    # 如果 start_dvl_driver:=true，则启动 dvl_a50 官方驱动
    dvl_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('dvl_a50'),
                'launch',
                'dvl_a50.launch.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('start_dvl_driver')),
        launch_arguments={
            'ip_address': LaunchConfiguration('dvl_ip')
        }.items(),
    )

     
    # DVL bridge
     

    # 如果 enable_dvl_bridge:=true，则启动 dvl_bridge
    # 它会把 /dvl/data 和 /dvl/position 转成：
    #   /sensor/dvl
    #   /sensor/dvl_pose
    dvl_bridge = Node(
        package='ustrov_sensor_bridge',
        executable='dvl_bridge',
        name='dvl_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_dvl_bridge')),
    )

     
    # 组合 launch description
     
    return LaunchDescription([
        start_dvl_driver_arg,
        enable_dvl_bridge_arg,
        dvl_ip_arg,
        ekf_config_arg,
        enable_monitor_arg,

        imu_bridge,
        depth_bridge,
        ekf,
        monitor,

        dvl_driver,
        dvl_bridge,
    ])