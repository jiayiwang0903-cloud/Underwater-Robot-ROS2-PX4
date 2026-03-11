#!/usr/bin/env python3
"""
电机空转测试脚本 (Hardware-in-the-Loop)
!!! 运行前切记：拆除真实机器人上的所有螺旋桨 !!!

允许你输入 0-7 的电机编号和推力比例 (-1.0 到 1.0) 进行单电机测试。
必须在有 DDS 连接到 Pixhawk 的情况下运行。
"""

import sys
import threading
import numpy as np

import rclpy
from rclpy.node import Node

from px4_interface import PX4Interface
from px4_actuator import PX4ActuatorInterface


class MotorTestNode(Node):
    def __init__(self):
        super().__init__('motor_test_node')
        
        self.px4_cmd = PX4Interface(self)
        self.px4_act = PX4ActuatorInterface(self)
        
        # 为了测试，我们将推力标定改为归一化直传。
        # 覆写 px4_act.max_thrust_newtons，使其在调用 send 时，传入的即是 -1.0 到 1.0 的控制量
        self.px4_act.max_thrust_newtons = 1.0  
        
        self.active_thrusts = np.zeros(8, dtype=float)
        
        # 50Hz 维持 PX4 Offboard 心跳和 Actuator 消息
        self.create_timer(0.02, self._timer_callback)
        
        self.get_logger().info("电机测试节点已启动，等待控制台输入指令...")

    def _timer_callback(self):
        # 1. 告诉飞控我们绕过内置混控器
        self.px4_cmd.send_offboard_mode()
        # 2. 发送实际推力指令
        self.px4_act.send(self.active_thrusts)


def console_input_thread(node: MotorTestNode):
    print("====================================")
    print("  !!! 危险：请确认浆叶已拆除 !!!    ")
    print("====================================")
    print("命令格式: [电机编号 0-7] [推力 -1.0 ~ 1.0]")
    print("例如: 0 0.2 (表示0号电机开启20%正向推力)")
    print("输入 q 或 exit 退出并归零全系电机")
    print("输入 arm 解锁，输入 disarm 上锁")
    print("====================================")
    
    while rclpy.ok():
        try:
            cmd = input("指令 > ").strip().lower()
        except EOFError:
            break
            
        if not cmd:
            continue
            
        if cmd in ['q', 'quit', 'exit']:
            node.active_thrusts.fill(0.0)
            print("正在退出...")
            # 停止节点
            rclpy.shutdown()
            break
            
        if cmd == 'arm':
            node.px4_cmd.arm()
            print(">>> 试图发送解锁指令！请观察安全开关和呼吸灯状态。")
            continue
            
        if cmd == 'disarm':
            node.px4_cmd._send_command(
                200, # VEHICLE_CMD_COMPONENT_ARM_DISARM 实际定义的值会有不同，但在 px4_interface 中做了封装，我们直接借用内部函数发0
                0.0, 21196.0) 
            node.active_thrusts.fill(0.0)
            print(">>> 发送上锁指令并全通道归零。")
            continue

        parts = cmd.split()
        if len(parts) != 2:
            print("格式错误。请输入: [编号] [推力值]")
            continue
            
        try:
            m_id = int(parts[0])
            val = float(parts[1])
            
            if m_id < 0 or m_id > 7:
                print("电机编号必须在 0 到 7 之间")
                continue
                
            val = max(min(val, 1.0), -1.0)
            
            node.active_thrusts[m_id] = val
            print(f">>> [更新] 电机 {m_id} -> {val}")
            
        except ValueError:
            print("参数必须为数字")


def main(args=None):
    rclpy.init(args=args)
    node = MotorTestNode()
    
    # 启动控制台输入线程
    input_thread = threading.Thread(target=console_input_thread, args=(node,))
    input_thread.daemon = True
    input_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.active_thrusts.fill(0.0)
        node.px4_act.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()