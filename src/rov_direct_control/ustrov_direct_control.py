#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import math

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleAttitude

# Gazebo Transport — 直接发送，不再依赖 ros_gz_bridge！
from gz.transport13 import Node as GzNode
from gz.msgs10.double_pb2 import Double as GzDouble

class PIDController:
    def __init__(self, kp, ki, kd, limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        # 积分限幅
        self.integral = np.clip(self.integral, -self.limit / self.ki if self.ki != 0 else self.limit, 
                                 self.limit / self.ki if self.ki != 0 else self.limit)
        
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        # 总输出限幅
        output = np.clip(output, -self.limit, self.limit)
        return output


class USTROVDirectController(Node):
    def __init__(self):
        super().__init__('ustrov_direct_controller')

        # Z 为 Down
        self.target_depth = 15.0    
        self.target_yaw = 0.0      
        self.dt = 0.02             

        #PID
        self.pid_z = PIDController(kp=50.0, ki=5.0, kd=20.0, limit=200.0)
        self.pid_yaw = PIDController(kp=5.0, ki=0.5, kd=2.0, limit=50.0)

        #B_alloc  FRD 坐标系
        # Gazebo FLU → FRD 转换: Y_FRD = -Y_Gaz, Z_FRD = -Z_Gaz
        # 正cmd_thrust产生沿关节轴正方向的力
        a = 0.7071  # sin(45) = cos(45)
        
        # SDF 物理位置 → FRD 坐标:
        # R0: pos(0.2, 0.15) axis(+a,+a)  前右角，推力指向前右
        # R1: pos(-0.2,-0.15) axis(-a,-a)  后左角，推力指向后左
        # R2: pos(0.2,-0.15) axis(+a,-a)  前左角，推力指向前左
        # R3: pos(-0.2, 0.15) axis(-a,+a)  后右角，推力指向后右
        # R4: pos(0.15,-0.1) axis(0,0,-1)  R5: pos(-0.15,0.1)
        # R6: pos(0.15, 0.1) axis(0,0,-1)  R7: pos(-0.15,-0.1)
        
        # 偏航力矩系数: Mz = x*Fy - y*Fx
        c = a * 0.05  # = 0.7071 * (0.2 - 0.15) ≈ 0.0354
        
        self.B_alloc = np.array([
            [ a,  -a,   a,  -a,     0,     0,     0,     0  ],  # Fx (前进)
            [ a,  -a,  -a,   a,     0,     0,     0,     0  ],  # Fy (FRD右)
            [ 0,   0,   0,   0,    -1,    -1,    -1,    -1  ],  # Fz (FRD下=NED潜)
            [ 0,   0,   0,   0,   0.1,  -0.1,  -0.1,   0.1 ],  # Mx (Roll)
            [ 0,   0,   0,   0,  0.15, -0.15,  0.15, -0.15 ],  # My (Pitch)
            [ c,   c,  -c,  -c,     0,     0,     0,     0  ],  # Mz (Yaw)
        ], dtype=np.float64)
        
        self.B_pinv = np.linalg.pinv(self.B_alloc)

        # === QoS 配置 (与 PX4 底层通信必须用 BEST_EFFORT) ===
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        #ROS 2 订阅器
        self.sub_pos = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.pos_cb, qos_profile)
        self.sub_att = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.att_cb, qos_profile)

        #ROS 2 发布器
        self.pub_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.pub_vehicle_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        #Gazebo Transport 直连推进器
        self.gz_node = GzNode()
        self.gz_pubs = []
        for i in range(8):
            topic = f'/model/ustrov_0/joint/rotor_{i}_joint/cmd_thrust'
            pub = self.gz_node.advertise(topic, GzDouble)
            self.gz_pubs.append(pub)
            self.get_logger().info(f'Gazebo 直连: {topic}')

        #状态变量
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.offboard_setpoint_counter = 0
        self.pos_received = False  # 收到第一条位置数据前不发推力

        #启动控制循环
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("USTROV 原生底层控制器已启动！准备下潜！")

    def pos_cb(self, msg):
        if not self.pos_received:
            self.pos_received = True
            self.pid_z.integral = 0.0
            self.pid_z.prev_error = 0.0
            self.get_logger().info(f'首次收到位置: z={msg.z:.2f}m，控制启动！')
        self.current_z = msg.z  # NED坐标系下的Z (正数为水下)

    def att_cb(self, msg):
        # 从四元数解算 Yaw (偏航角)
        q = msg.q
        self.current_yaw = math.atan2(2.0 * (q[0]*q[3] + q[1]*q[2]), 1.0 - 2.0 * (q[2]**2 + q[3]**2))

    def control_loop(self):
        self.offboard_setpoint_counter += 1
        self.publish_offboard_control_mode()

        # 延时解锁
        if self.offboard_setpoint_counter >= 50:
            if self.offboard_setpoint_counter % 10 == 0:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) 
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)

        # 等待第一条位置数据到达后再输出推力
        if not self.pos_received:
            return

        error_z = self.target_depth - self.current_z
        tau_z = self.pid_z.update(error_z, self.dt)

        # === 2. 航向控制 (FRD 逻辑) ===
        error_yaw = self.target_yaw - self.current_yaw
        error_yaw = (error_yaw + math.pi) % (2 * math.pi) - math.pi
        tau_yaw = self.pid_yaw.update(error_yaw, self.dt)

        # === 3. 组装期望力矩 (FRD Wrench) ===
        # tau_desired = [F_x, F_y, F_z, M_x, M_y, M_z]
        tau_desired = np.array([0.0, 0.0, tau_z, 0.0, 0.0, tau_yaw])

        # === 4. 解算并限制输出 ===
        thrusts = self.B_pinv @ tau_desired
        self.publish_actuator_motors(thrusts)

        if self.offboard_setpoint_counter % 50 == 0:
            self.get_logger().info(
                f"NED Depth: {self.current_z:.2f}m (Target: {self.target_depth}m) | "
                f"Yaw: {math.degrees(self.current_yaw):.1f}° | 推力(+Z为下): {tau_z:.1f}N"
            )

    def publish_offboard_control_mode(self):
        """告诉飞控：绕过混控器，我们要直接控制底层推力！"""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.direct_actuator = True  # <--- 核心指令：开启上帝模式！
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_offboard_mode.publish(msg)

    def publish_actuator_motors(self, thrusts):
        """直接通过 Gazebo Transport 发送力(牛顿)到推进器。"""
        for i in range(8):
            msg = GzDouble()
            msg.data = float(np.clip(thrusts[i], -500.0, 500.0))
            self.gz_pubs[i].publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """发送飞控指令 (解锁/切模式)"""
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_vehicle_cmd.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = USTROVDirectController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()