#!/usr/bin/env python3
"""DVL A50 -> EKF bridge / DVL A50 到 EKF 桥接节点。

English:
This node converts DVL outputs into EKF-consumable ROS messages:
- `dvl_msgs/msg/DVL`   -> `geometry_msgs/msg/TwistWithCovarianceStamped`
- `dvl_msgs/msg/DVLDR` -> `geometry_msgs/msg/PoseWithCovarianceStamped`

中文：
该节点将 DVL 输出转换为 EKF 可直接融合的 ROS 标准消息：
- `dvl_msgs/msg/DVL`   -> `geometry_msgs/msg/TwistWithCovarianceStamped`
- `dvl_msgs/msg/DVLDR` -> `geometry_msgs/msg/PoseWithCovarianceStamped`

Input / 输入:
- /dvl/data (dvl_msgs/msg/DVL)
- /dvl/position (dvl_msgs/msg/DVLDR)

Output / 输出:
- /sensor/dvl (geometry_msgs/msg/TwistWithCovarianceStamped)
- /sensor/dvl_pose (geometry_msgs/msg/PoseWithCovarianceStamped)

Design note / 设计说明:
- Only valid bottom-lock velocity is forwarded (`velocity_valid == True`).
- 仅在 DVL 速度有效（`velocity_valid == True`）时转发给 EKF。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math

from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from dvl_msgs.msg import DVL, DVLDR


class DvlBridge(Node):
    def __init__(self):
        super().__init__('dvl_bridge')

        dvl_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # English: EKF expects twist-with-covariance on this topic.
        # 中文：EKF 在该话题上期望收到“带协方差的速度”。
        self.pub_dvl = self.create_publisher(TwistWithCovarianceStamped, '/sensor/dvl', 10)
        self.pub_dvl_pose = self.create_publisher(PoseWithCovarianceStamped, '/sensor/dvl_pose', 10)

        # English: Water Linked reports roll/pitch/yaw in degrees.
        # 中文：Water Linked 的 roll/pitch/yaw 为角度单位。
        self._angles_in_degrees = True

        self.create_subscription(DVL, '/dvl/data', self._dvl_cb, dvl_qos)
        self.create_subscription(DVLDR, '/dvl/position', self._dvl_dr_cb, dvl_qos)

        self._published_vel_count = 0
        self._published_pose_count = 0
        self._dropped_invalid_vel_count = 0
        self.create_timer(1.0, self._stats_timer)
        self.get_logger().info(
            'DVL bridge started / 启动成功: '
            '/dvl/data->/sensor/dvl and /dvl/position->/sensor/dvl_pose'
        )

    def _dvl_cb(self, msg: DVL):
        # English: Forward only valid bottom-lock samples.
        # 中文：仅转发“已锁底且有效”的速度样本。
        if not msg.velocity_valid:
            self._dropped_invalid_vel_count += 1
            return

        out = TwistWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        # English: Keep frame_id consistent with EKF twist input convention.
        # 中文：frame_id 与 EKF 速度输入约定保持一致。
        out.header.frame_id = 'base_link'

        out.twist.twist.linear.x = msg.velocity.x
        out.twist.twist.linear.y = msg.velocity.y
        out.twist.twist.linear.z = msg.velocity.z

        # English: Map DVL 3x3 velocity covariance into ROS 6x6 twist covariance.
        # 中文：将 DVL 的 3x3 速度协方差映射到 ROS 的 6x6 twist 协方差矩阵。

        # ROS linear velocity block indices / ROS 线速度块索引:
        # [0,1,2,
        #  6,7,8,
        # 12,13,14]
        if len(msg.covariance) >= 9:
            out.twist.covariance[0] = float(msg.covariance[0])
            out.twist.covariance[1] = float(msg.covariance[1])
            out.twist.covariance[2] = float(msg.covariance[2])
            out.twist.covariance[6] = float(msg.covariance[3])
            out.twist.covariance[7] = float(msg.covariance[4])
            out.twist.covariance[8] = float(msg.covariance[5])
            out.twist.covariance[12] = float(msg.covariance[6])
            out.twist.covariance[13] = float(msg.covariance[7])
            out.twist.covariance[14] = float(msg.covariance[8])
        else:
            # English: Fallback when full covariance is unavailable.
            # 中文：当原始协方差缺失时，使用 FOM^2 作为对角线方差回退值。
            var = max(float(msg.fom) * float(msg.fom), 1e-6)
            out.twist.covariance[0] = var
            out.twist.covariance[7] = var
            out.twist.covariance[14] = var

        self.pub_dvl.publish(out)
        self._published_vel_count += 1

    def _dvl_dr_cb(self, msg: DVLDR):
        out = PoseWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'odom'

        out.pose.pose.position.x = float(msg.position.x)
        out.pose.pose.position.y = float(msg.position.y)
        out.pose.pose.position.z = float(msg.position.z)

        roll = float(msg.roll)
        pitch = float(msg.pitch)
        yaw = float(msg.yaw)
        if self._angles_in_degrees:
            roll = math.radians(roll)
            pitch = math.radians(pitch)
            yaw = math.radians(yaw)

        qx, qy, qz, qw = self._quaternion_from_euler(roll, pitch, yaw)
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw

        # Position covariance from DVL-reported std (same variance on xyz).
        pos_var = max(float(msg.pos_std) * float(msg.pos_std), 1e-4)
        out.pose.covariance[0] = pos_var
        out.pose.covariance[7] = pos_var
        out.pose.covariance[14] = pos_var

        # Orientation covariance fallback (about 5 deg std).
        rpy_var = math.radians(5.0) ** 2
        out.pose.covariance[21] = rpy_var
        out.pose.covariance[28] = rpy_var
        out.pose.covariance[35] = rpy_var

        self.pub_dvl_pose.publish(out)
        self._published_pose_count += 1

    @staticmethod
    def _quaternion_from_euler(roll: float, pitch: float, yaw: float):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def _stats_timer(self):
        self.get_logger().info(
            f'/sensor/dvl vel_published(valid)={self._published_vel_count}, '
            f'vel_dropped_invalid={self._dropped_invalid_vel_count}, '
            f'/sensor/dvl_pose pose_published={self._published_pose_count}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DvlBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
