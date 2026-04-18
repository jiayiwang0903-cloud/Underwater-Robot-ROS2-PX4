"""推力分配层（allocation layer）。

将控制层输出的 6-DOF 广义力/力矩 [Fx, Fy, Fz, Mx, My, Mz] (FRD)
通过 Moore-Penrose 伪逆解算为 8 个推进器的独立推力（牛顿）。

支持两种初始化方式：
  1. ThrustAllocator 类：从 YAML 配置文件加载推进器几何
  2. allocate() 函数：使用内置默认矩阵（向后兼容）
"""

import numpy as np


class ThrustAllocator:
    """可配置的推力分配器。

    在初始化阶段完成：YAML 读取 -> 分配矩阵 B 构造 -> 伪逆 B+ 计算。
    运行阶段的 allocate() 为纯计算路径。
    """

    def __init__(self, config_path: str):
        """从 YAML 配置加载推进器几何并计算伪逆。

        Args:
            config_path: thruster_geometry.yaml 的路径
        """
        import yaml
        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)

        self.num_thrusters = int(cfg['num_thrusters'])
        self.max_thrust_n = float(cfg['max_thrust_n'])
        self.clip_thrust_n = float(cfg['clip_thrust_n'])
        self.channel_signs = np.array(cfg['channel_signs'], dtype=np.float64)

        self.B = np.array(cfg['allocation_matrix'], dtype=np.float64)
        self.B_pinv = np.linalg.pinv(self.B)

    def allocate(self, tau: np.ndarray) -> np.ndarray:
        """将 6-DOF 力矩向量解算为推力并限幅。"""
        thrusts = self.B_pinv @ tau
        return np.clip(thrusts, -self.clip_thrust_n, self.clip_thrust_n)


# ── 向后兼容的默认分配函数 ──────────────────────────

_A = 0.7071
_C = _A * 0.05

_B_ALLOC_DEFAULT = np.array([
    [ _A, -_A,  _A, -_A,     0,     0,     0,     0  ],  # Fx
    [ _A, -_A, -_A,  _A,     0,     0,     0,     0  ],  # Fy
    [  0,   0,   0,   0,   1,    -1,    1,    1  ],  # Fz
    [  0,   0,   0,   0,   0.1,  -0.1,  -0.1,   0.1 ],  # Mx
    [  0,   0,   0,   0,  0.15, -0.15,  0.15, -0.15 ],  # My
    [ _C,  _C,  -_C, -_C,    0,     0,     0,     0  ],  # Mz
], dtype=np.float64)

_B_PINV_DEFAULT = np.linalg.pinv(_B_ALLOC_DEFAULT)


def allocate(tau: np.ndarray, clip: float = 100.0) -> np.ndarray:
    """使用默认分配矩阵将 6-DOF 力矩向量解算为 8 路推力。"""
    thrusts = _B_PINV_DEFAULT @ tau
    return np.clip(thrusts, -clip, clip)
