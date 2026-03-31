# rov_direct_control

USTROV direct-control stack based on ROS 2 + PX4.

USTROV 基于 ROS 2 + PX4 的直接控制代码栈。

## 0. Rename Note / 重命名说明

- Simulation folder is now **`sim/`**.
- Previous name **`sim_env/`** is deprecated and should not be used.

- 仿真目录已统一为 **`sim/`**。
- 之前的 **`sim_env/`** 已废弃，不再作为有效路径使用。

## 1. Architecture Overview / 架构总览

### 1.1 Real Vehicle Path / 实际控制链路

1. EKF publishes fused state on `/odometry/filtered`.
2. `controller_node.py` reads state and computes 4-DOF control (`X/Y/Z/Yaw`).
3. `allocator.py` maps wrench `[Fx, Fy, Fz, Mx, My, Mz]` to 8 thruster forces.
4. `px4_actuator.py` publishes `ActuatorMotors` to `/fmu/in/actuator_motors`.
5. PX4 outputs PWM to ESC/thrusters.

1. EKF 在 `/odometry/filtered` 输出融合状态。
2. `controller_node.py` 读取状态并计算 4 自由度控制量（`X/Y/Z/Yaw`）。
3. `allocator.py` 将 `[Fx, Fy, Fz, Mx, My, Mz]` 分配为 8 路推进器推力。
4. `px4_actuator.py` 发布 `ActuatorMotors` 到 `/fmu/in/actuator_motors`。
5. PX4 输出 PWM 至 ESC/推进器。

### 1.2 PX4 Mode Path / PX4 模式控制链路

1. `px4_interface.py` keeps sending `/fmu/in/offboard_control_mode` heartbeat.
2. Controller sends `VehicleCommand` for OFFBOARD + ARM/DISARM.

1. `px4_interface.py` 持续发送 `/fmu/in/offboard_control_mode` 心跳。
2. 控制器发送 `VehicleCommand` 执行 OFFBOARD + ARM/DISARM。

### 1.3 Simulation Path / 仿真链路

All simulation components are under `sim/`.

仿真相关组件全部位于 `sim/`。

1. `sim/sim_sensors.py` converts Gazebo truth into noisy IMU/DVL/Depth topics.
2. `robot_localization` (with `config/ekf.yaml`) fuses sensor topics into `/odometry/filtered`.
3. `sim/error_analyzer.py` compares EKF estimate with Gazebo ground truth.
4. Optional visual source: `sim/visual_ekf_node.py` publishes `/sensor/aruco_pose`.

1. `sim/sim_sensors.py` 将 Gazebo 真值转换为带噪声的 IMU/DVL/Depth 话题。
2. `robot_localization`（参数文件 `config/ekf.yaml`）融合后输出 `/odometry/filtered`。
3. `sim/error_analyzer.py` 对比 EKF 估计和 Gazebo 真值误差。
4. 可选视觉源：`sim/visual_ekf_node.py` 发布 `/sensor/aruco_pose`。

## 2. Code Map / 文件职责

### Core (real + common) / 核心模块（实机+通用）

- `main.py`
  Entry point, spins `USTROVDirectController`.
  入口文件，启动 `USTROVDirectController`。

- `controller_node.py`
  Main control loop, PID, coordinate transform, allocation, actuator publish.
  主控制循环，执行 PID、坐标变换、推力分配与执行发布。

- `pid.py`
  Generic PID (integral clamp + filtered derivative).
  通用 PID（积分限幅 + 微分低通滤波）。

- `allocator.py`
  6-DOF wrench to 8-thruster allocation using pseudo-inverse.
  使用伪逆矩阵将 6 自由度力矩分配到 8 推进器。

- `state_estimator.py`
  Estimator abstraction and `EKFEstimator` subscriber for `/odometry/filtered`.
  状态估计抽象层与 `EKFEstimator`（订阅 `/odometry/filtered`）。

- `px4_interface.py`
  Offboard heartbeat and vehicle arm/disarm commands.
  Offboard 心跳与解锁/上锁指令接口。

- `px4_actuator.py`
  Converts thrust array to PX4 `ActuatorMotors` message and publishes to FMU input.
  将推力数组转换为 PX4 `ActuatorMotors` 消息并发布到 FMU 输入。

- `test_motors.py`
  Manual motor test utility for direct channel validation.
  电机手动测试工具，用于直接验证通道链路。

### Config / 配置

- `config/ekf.yaml`
  `robot_localization` EKF sensor fusion configuration.
  `robot_localization` 的 EKF 传感器融合配置。

### Simulation (`sim/`) / 仿真模块（`sim/`）

- `sim/start_perception_brain.sh`
  One-command startup for simulation perception + EKF pipeline.
  一键启动仿真感知 + EKF 融合链路。

- `sim/sim_sensors.py`
  Generates noisy IMU/DVL/Depth from Gazebo dynamic pose.
  从 Gazebo 动态位姿生成带噪 IMU/DVL/Depth。

- `sim/error_analyzer.py`
  Live EKF-vs-ground-truth error visualization.
  实时显示 EKF 与真值误差。

- `sim/visual_ekf_node.py`
  ArUco-based visual relative localization.
  基于 ArUco 的视觉相对定位。

- `sim/usv_simulator.py`
  USV target spawn/motion and GPS-like odometry publisher.
  水面 USV 目标生成/运动与 GPS 类里程计发布。

- `sim/gz_thruster.py`
  Gazebo transport-based direct thruster command helper.
  基于 Gazebo transport 的推力直连接口。

## 3. Real Controller Details / 实机控制细节

### `USTROVDirectController` (`controller_node.py`)

- Loop rate: `50 Hz` (`dt = 0.02`).
- 主循环频率：`50 Hz`（`dt = 0.02`）。

Per tick / 每个周期：
1. Send Offboard heartbeat / 发送 Offboard 心跳。
2. Try OFFBOARD + ARM after warmup ticks / 预热后尝试切换 OFFBOARD 并解锁。
3. Read `RovState` from estimator / 从估计器读取 `RovState`。
4. Compute body-frame errors and PID outputs / 计算机体系误差与 PID 输出。
5. Clamp control outputs (`tau_x/y/z/yaw`) / 对控制量进行限幅。
6. Allocate thrust and publish actuator command / 分配推力并发布执行指令。

### `PX4ActuatorInterface` (`px4_actuator.py`)

- Input: 8-element thrust array in Newtons.
- 输入：8 路推进器推力（牛顿）。

Processing / 处理过程：
1. Normalize by `max_thrust_newtons` (default `200.0`).
2. Clip to `[-1.0, 1.0]`.
3. Remap to `[0.0, 1.0]`.
4. Fill unused channels with `NaN`.
5. Publish `/fmu/in/actuator_motors`.

1. 按 `max_thrust_newtons`（默认 `200.0`）归一化。
2. 限幅到 `[-1.0, 1.0]`。
3. 线性映射到 `[0.0, 1.0]`。
4. 未用通道填充 `NaN`。
5. 发布 `/fmu/in/actuator_motors`。

## 4. Topics / 关键话题

### Published by control stack / 控制栈发布

- `/fmu/in/offboard_control_mode` (`px4_msgs/OffboardControlMode`)
- `/fmu/in/vehicle_command` (`px4_msgs/VehicleCommand`)
- `/fmu/in/actuator_motors` (`px4_msgs/ActuatorMotors`)

### Consumed by control stack / 控制栈订阅

- `/odometry/filtered` (`nav_msgs/Odometry`)

### Simulation sensor topics / 仿真传感器话题

- `/sensor/imu`
- `/sensor/dvl`
- `/sensor/depth`
- `/sensor/aruco_pose` (optional; currently commented in EKF config)
- `/sensor/aruco_pose`（可选；当前在 EKF 配置中注释）

## 5. Run Modes / 运行模式

### 5.1 Main controller / 主控制器

Requirements / 前置条件：
- Micro XRCE agent running.
- PX4 SITL or hardware PX4 running.
- EKF pipeline alive (`/odometry/filtered` available).

- Micro XRCE Agent 已运行。
- PX4 SITL 或实机 PX4 已运行。
- EKF 融合链路可用（`/odometry/filtered` 有数据）。

```bash
ros2 run rov_direct_control main.py
```

### 5.2 Manual motor test / 手动电机测试

```bash
ros2 run rov_direct_control test_motors.py
```

Interactive commands / 交互命令：
- `arm`, `disarm`
- `status`
- `stop`
- `<id> <value>` (`id: 0..7`, `value: -1.0..1.0` or `nan`)
- `z <value>` (batch set channels `0..3`)
- `q`

### 5.3 Simulation perception+EKF / 仿真感知与 EKF 启动

```bash
bash src/rov_direct_control/sim/start_perception_brain.sh
```

## 6. Quick Troubleshooting / 快速排查

### A. No publisher on `/fmu/in/actuator_motors`
### A. `/fmu/in/actuator_motors` 发布者为 0

```bash
ros2 node list
ros2 topic info /fmu/in/actuator_motors
```

Check if `main.py`/`test_motors.py` is alive and not crashing.

检查 `main.py`/`test_motors.py` 是否正常运行且未崩溃。

### B. Micro XRCE bind error (`errno 98`)
### B. Micro XRCE 端口冲突（`errno 98`）

```bash
ss -lunp | grep 8888
lsof -iUDP:8888
```

Stop duplicate agent process; keep only one on UDP `8888`.

停止重复 Agent 进程，确保 UDP `8888` 仅有一个实例。

### C. EKF not updating
### C. EKF 不更新

```bash
ros2 topic hz /odometry/filtered
ros2 topic echo /sensor/imu --once
ros2 topic echo /sensor/dvl --once
ros2 topic echo /sensor/depth --once
```

Ensure timestamps are moving and frames/config match.

确认时间戳持续递增，且坐标系与 EKF 配置一致。

### D. Armed but no thrust response
### D. 已解锁但无推力响应

```bash
ros2 topic echo /fmu/in/offboard_control_mode --once
ros2 topic echo /fmu/in/vehicle_command --once
ros2 topic echo /fmu/in/actuator_motors --once
```

## 7. Design Intent / 设计意图

- `state_estimator.py` is an abstraction boundary; sensor backend can change without touching control logic.
- `allocator.py` is stateless and easy to recalibrate.
- `sim/` is fully dedicated to simulation and validation tooling.

- `state_estimator.py` 是抽象边界；传感器后端可替换且不影响控制逻辑。
- `allocator.py` 无状态，便于独立标定。
- `sim/` 完整承载仿真与验证工具链。
