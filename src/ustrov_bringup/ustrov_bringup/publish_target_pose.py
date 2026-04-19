#!/usr/bin/env python3
"""向 /ustrov/target_pose 发布目标位姿的测试工具。

用法:
  # 单次发布: x=1.0 y=0.0 depth=5.0 yaw_deg=0.0
  python3 publish_target_pose.py 1.0 0.0 5.0 0.0

  # 持续发布 (1 Hz):
  python3 publish_target_pose.py 1.0 0.0 5.0 0.0 --rate 1.0

坐标约定: NED (z > 0 = 更深), yaw 单位为度。
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def yaw_to_quaternion(yaw_rad: float):
    """将 yaw 角（弧度）转换为四元数 (w, x, y, z)，roll=pitch=0。"""
    w = math.cos(yaw_rad / 2.0)
    z = math.sin(yaw_rad / 2.0)
    return w, 0.0, 0.0, z


def main():
    parser = argparse.ArgumentParser(description='Publish target pose to /ustrov/target_pose')
    parser.add_argument('x', type=float, help='Target X (NED north, meters)')
    parser.add_argument('y', type=float, help='Target Y (NED east, meters)')
    parser.add_argument('z', type=float, help='Target depth (NED down, z>0=deeper, meters)')
    parser.add_argument('yaw', type=float, help='Target yaw (degrees)')
    parser.add_argument('--rate', type=float, default=0.0,
                        help='Publish rate in Hz. 0 = single publish and exit.')
    args = parser.parse_args()

    rclpy.init()
    node = Node('target_pose_publisher')
    pub = node.create_publisher(PoseStamped, '/ustrov/target_pose', 10)

    msg = PoseStamped()
    msg.header.frame_id = 'ned'
    msg.pose.position.x = args.x
    msg.pose.position.y = args.y
    msg.pose.position.z = args.z

    yaw_rad = math.radians(args.yaw)
    w, qx, qy, qz = yaw_to_quaternion(yaw_rad)
    msg.pose.orientation.w = w
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz

    if args.rate <= 0:
        # 单次发布
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        node.get_logger().info(
            f'Published target: x={args.x} y={args.y} depth={args.z} yaw={args.yaw}deg')
        # 等待一小段时间确保消息发出
        import time
        time.sleep(0.5)
    else:
        # 持续发布
        period = 1.0 / args.rate
        node.get_logger().info(
            f'Publishing target at {args.rate} Hz: x={args.x} y={args.y} depth={args.z} yaw={args.yaw}deg')

        def timer_cb():
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)

        node.create_timer(period, timer_cb)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
