"""推力分配矩阵
将 6-DOF 期望力/力矩 [Fx, Fy, Fz, Mx, My, Mz] (FRD)
解算为 8 个推进器的独立力值 (牛顿)。
"""

import numpy as np

# SDF 物理布局 → FRD 坐标（Y_FRD = -Y_Gaz, Z_FRD = -Z_Gaz）
# R0: pos(0.2, 0.15,0)  axis(+a,+a,0)   前右
# R1: pos(-0.2,-0.15,0) axis(-a,-a,0)   后左
# R2: pos(0.2,-0.15,0)  axis(+a,-a,0)   前左
# R3: pos(-0.2, 0.15,0) axis(-a,+a,0)   后右
# R4: pos(0.15,-0.1,z)  axis(0,0,-1)    R5: pos(-0.15,0.1,z)
# R6: pos(0.15, 0.1,z)  axis(0,0,-1)    R7: pos(-0.15,-0.1,z)

_A = 0.7071                  # sin(45°) = cos(45°)
_C = _A * 0.05               # 偏航力矩系数 ≈ 0.0354

B_ALLOC = np.array([
    [ _A, -_A,  _A, -_A,     0,     0,     0,     0  ],  # Fx
    [ _A, -_A, -_A,  _A,     0,     0,     0,     0  ],  # Fy
    [  0,   0,   0,   0,    -1,    -1,    -1,    -1  ],  # Fz (FRD 下)
    [  0,   0,   0,   0,   0.1,  -0.1,  -0.1,   0.1 ],  # Mx
    [  0,   0,   0,   0,  0.15, -0.15,  0.15, -0.15 ],  # My
    [ _C,  _C,  -_C, -_C,    0,     0,     0,     0  ],  # Mz
], dtype=np.float64)

B_PINV = np.linalg.pinv(B_ALLOC)


def allocate(tau: np.ndarray, clip: float = 100.0) -> np.ndarray:
    """将 6-DOF 力矩向量解算为 8 路推力，并限幅。"""
    thrusts = B_PINV @ tau
    return np.clip(thrusts, -clip, clip)
