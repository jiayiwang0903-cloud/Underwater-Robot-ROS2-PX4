"""PID 控制器 — 纯算法，无 ROS/Gazebo 依赖。"""

import numpy as np


class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, limit: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_derivative = 0.0

    def update(self, error: float, dt: float) -> float:
        self.integral += error * dt
        # 积分限幅（防止积分项单独超限）
        i_limit = self.limit / self.ki if self.ki != 0 else self.limit
        self.integral = np.clip(self.integral, -i_limit, i_limit)

        raw_derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        # 增加一阶低通滤波(EMA)以平滑微分项的噪声放大
        alpha = 0.2
        self.filtered_derivative = alpha * raw_derivative + (1.0 - alpha) * self.filtered_derivative

        output = self.kp * error + self.ki * self.integral + self.kd * self.filtered_derivative
        return float(np.clip(output, -self.limit, self.limit))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_derivative = 0.0
