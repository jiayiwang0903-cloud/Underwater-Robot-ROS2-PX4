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

from px4_msgs.msg import VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy
from allocator import allocate

class MotorTestNode(Node):
    def __init__(self):
        super().__init__('motor_test_node')
        
        self.px4_cmd = PX4Interface(self)
        self.px4_act = PX4ActuatorInterface(self)
        
        # 定义与 PX4 匹配的 QoS (Best Effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅飞行器状态以实时监控解锁状态
        self.sub_status = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self._status_cb, qos_profile
        )
        self.is_armed = False
        self.nav_state = 0

        # ROS 2 标准手柄消息订阅
        self.sub_joy = self.create_subscription(Joy, '/joy', self._joy_cb, qos_profile)

        # 为了测试，我们将推力标定改为归一化直传。
        # 覆写 px4_act.max_thrust_newtons，使其在调用 send 时，传入的即是 -1.0 到 1.0 的控制量
        self.px4_act.max_thrust_newtons = 1.0  
        
        # 使用 NaN 初始化，告诉飞控"我不控制这些电机"，以便让它输出默认的 Disarmed PWM 值
        self.active_thrusts = np.full(8, np.nan, dtype=float)
        
        # 50Hz 维持 PX4 Offboard 心跳和 Actuator 消息
        self.create_timer(0.02, self._timer_callback)
        
        self.get_logger().info("电机测试节点已启动，等待控制台输入指令...")

    def _status_cb(self, msg):
        self.is_armed = (msg.arming_state == 2) # ARMING_STATE_ARMED
        self.nav_state = msg.nav_state

    def _joy_cb(self, msg):
        # 简单消除摇杆死区
        def deadband(val, threshold=0.1):
            return 0.0 if abs(val) < threshold else val

        if len(msg.axes) < 6:
            return

        # 摇杆映射 (以标准Linux中Xbox手柄轴为准)
        # axes[0]: 左摇杆横向 (+左, -右) -> 期望Fy (+右所以需取反)
        # axes[1]: 左摇杆纵向 (+上, -下) -> 期望Fx (+前所以不需取反)
        # axes[2]: LT扳机 (+1松开, -1按到底) -> 向下(+Z)
        # axes[3]: 右摇杆横向 (+左, -右) -> 期望Mz (+顺时针所以需取反)
        # axes[5]: RT扳机 (+1松开, -1按到底) -> 向上(-Z)
        
        ls_y =  deadband(msg.axes[1])
        ls_x = -deadband(msg.axes[0])
        rs_x = -deadband(msg.axes[3])
        
        # 部分驱动扳机默认值为0，直到首次按下才变1。用0.1的判定避免开机漂移
        lt_val = (1.0 - msg.axes[2]) / 2.0 if msg.axes[2] != 0.0 else 0.0
        rt_val = (1.0 - msg.axes[5]) / 2.0 if msg.axes[5] != 0.0 else 0.0
        lt_val = deadband(lt_val, 0.05)
        rt_val = deadband(rt_val, 0.05)
        
        # tau 包含 [Fx, Fy, Fz, Mx, My, Mz]
        tau = np.zeros(6, dtype=np.float64)
        tau[0] = ls_y             # 前后
        tau[1] = ls_x             # 左右
        tau[2] = lt_val - rt_val  # Z轴 (FRD中，+Z为向下)
        tau[5] = rs_x             # Yaw绕Z旋转
        
        # 只有在有输入时，才覆盖当前推力值，避免影响手敲指令
        if not np.allclose(tau, 0.0, atol=0.01):
            # 将推力截取限制在 [-1.0, 1.0]
            thrusts = allocate(tau, clip=1.0)
            self.active_thrusts[:] = thrusts
            self._joy_controlled = True
        else:
            # 当手柄摇杆归中时，如果是从手柄控制转来的，将推力归为0
            # 这样就不会覆盖用户通过键盘独立设置的推力
            if getattr(self, '_joy_controlled', False):
                self.active_thrusts.fill(np.nan)
                self._joy_controlled = False

        # 手柄按键快捷操作：
        # msg.buttons[7] (Start键) -> 解锁
        # msg.buttons[6] (Back键) -> 上锁
        if len(msg.buttons) >= 8:
            if msg.buttons[7] == 1 and not self.is_armed:
                self.px4_cmd.arm()
                self.get_logger().info('手柄: 已发送解锁指令')
            elif msg.buttons[6] == 1 and self.is_armed:
                self.px4_cmd._send_command(400, 0.0, 21196.0)
                self.active_thrusts.fill(np.nan)
                self.get_logger().info('手柄: 已发送上锁指令')

    def _timer_callback(self):
        # 1. 告诉飞控我们绕过内置混控器
        self.px4_cmd.send_offboard_mode()
        # 2. 发送实际推力指令
        self.px4_act.send(self.active_thrusts)


def print_status_dashboard(node: MotorTestNode):
    """打印简洁美观的电机状态进度条"""
    arm_str = "已解锁 (ARMED)" if node.is_armed else "未解锁 (DISARMED)"
    print(f"\n[{arm_str}] 导航模式: {node.nav_state}")
    print("--- 实时推力指令 ---")
    for i in range(8):
        val = node.active_thrusts[i]
        if np.isnan(val):
            print(f"[{i}] 电机:  [ NaN (不控制/休眠) ]")
        else:
            # 渲染简单的进度条
            bar_len = 10
            if val < 0:
                bars = int(abs(val) * bar_len)
                bar_str = " " * (bar_len - bars) + "<" * bars + "|" + " " * bar_len
            else:
                bars = int(val * bar_len)
                bar_str = " " * bar_len + "|" + ">" * bars + " " * (bar_len - bars)
            
            print(f"[{i}] 电机:  [{bar_str}] {val*100:>5.1f}%")
    print("--------------------\n")

def console_input_thread(node: MotorTestNode):
    print("====================================")
    print("  !!! 危险：请确认浆叶已拆除 !!!    ")
    print("====================================")
    print("命令格式: [电机编号 0-7] [推力 -1.0 ~ 1.0]")
    print("        如 '0 0.5' 表示0号电机50%正向推力")
    print("        如 '2 nan' 表示2号电机休眠(切断)")
    print("        如 'z 0.3' 表示同时控制 4,5,6,7 垂直电机(上浮/下潜)")
    print("               ")
    print("输入 arm 解锁，输入 disarm 上锁")
    print("支持手柄(需运行 ros2 run joy joy_node): Start 键解锁, Back 键上锁")
    print("手柄控制：左摇杆前后左右平移，右摇杆转向，LT下潜，RT上浮")
    print("输入 status 查看所有电机图形化状态")
    print("输入 stop 所有通道急停为 nan")
    print("输入 q 退出系统")
    print("====================================")
    
    while rclpy.ok():
        try:
            cmd = input("指令 (按回车或输入 status 查看状态) > ").strip().lower()
        except EOFError:
            break
            
        if not cmd or cmd == 'status':
            print_status_dashboard(node)
            continue
            
        if cmd in ['q', 'quit', 'exit']:
            node.active_thrusts.fill(np.nan)
            # 优雅退退出信号，正在出前，主动发送上锁指令并停顿一小会确保发出
            node.px4_cmd._send_command(400, 0.0, 21196.0)
            import time
            time.sleep(0.2)
            print("正在退出并上锁...")
            rclpy.shutdown()
            break
            
        if cmd == 'stop':
            node.active_thrusts.fill(np.nan)
            print(">>> 急停：所有通道恢复 NaN")
            print_status_dashboard(node)
            continue

        if cmd == 'arm':
            node.px4_cmd.arm()
            print(">>> 试图发送解锁指令！")
            continue
            
        if cmd == 'disarm':
            node.px4_cmd._send_command(400, 0.0, 21196.0) 
            node.active_thrusts.fill(np.nan)
            print(">>> 发送上锁指令并全通道归零。")
            continue

        parts = cmd.split()
        if len(parts) != 2:
            print("格式错误。请输入: [编号] [推力值] 或者 'z [推力值]'")
            continue
            
        # 处理垂直电机批处理命令 'z'
        if parts[0] == 'z':
            vertical_ids = [4, 5, 6, 7]
            val_str = parts[1]
            if val_str == 'nan':
                for i in vertical_ids:
                    node.active_thrusts[i] = np.nan
            else:
                try:
                    val = float(val_str)
                    val = max(min(val, 1.0), -1.0)
                    for i in vertical_ids:
                        node.active_thrusts[i] = val
                except ValueError:
                    print("推力值必须是数字或 'nan'")
                    continue
            print(">>> 垂直电机组(4~7)更新完毕")
            print_status_dashboard(node)
            continue
            
        try:
            m_id = int(parts[0])
            val_str = parts[1]
            
            if m_id < 0 or m_id > 7:
                print("电机编号必须在 0 到 7 之间")
                continue
            
            if val_str == 'nan':
                node.active_thrusts[m_id] = np.nan
            else:
                val = float(val_str)
                val = max(min(val, 1.0), -1.0)
                node.active_thrusts[m_id] = val
            
            print_status_dashboard(node)
            
        except ValueError:
            print("参数设值必须为数字或 'nan'")


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
        # 检测到 Ctrl+C 等异常退出时，也要确保发送上锁指令
        node.active_thrusts.fill(np.nan)
        node.px4_cmd._send_command(400, 0.0, 21196.0)
        import time
        time.sleep(0.2)
        node.px4_act.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()