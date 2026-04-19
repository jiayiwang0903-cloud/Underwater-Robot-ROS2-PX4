# rov_direct_control (DEPRECATED)

> **This directory is deprecated.** Code has been restructured into separate ROS 2 packages under `src/`. See below for the new package structure.

> **该目录已废弃。** 代码已重构为独立的 ROS 2 包，位于 `src/` 下。参见下方新包结构。

## New Package Structure / 新包结构

| Package | Role |
|---------|------|
| `ustrov_control` | Control core: PID, allocator, state estimator, actuator backend ABC / 控制核心 |
| `ustrov_sensor_bridge` | Sensor bridges: IMU, DVL, depth to EKF / 传感器桥接 |
| `ustrov_px4` | PX4 interface: actuator backend, mode interface, manual control / PX4 接口 |
| `ustrov_sim` | Simulation: Gazebo backend, sim sensors, error analysis / 仿真 |
| `ustrov_bringup` | Integration: launch files, configs, entry points / 集成启动 |

## Quick Start

```bash
# Build all packages
cd ~/catkin_ws && colcon build

# Gazebo simulation
ros2 launch ustrov_bringup sim_control.launch.py

# PX4 real hardware
ros2 launch ustrov_bringup px4_control.launch.py

# Publish target at runtime
ros2 run ustrov_bringup publish_target_pose -- 1.0 0.0 5.0 0.0
```

---

**Below is the original documentation, kept for reference.**

---

USTROV external low-level control stack. It reads fused state, computes control outputs, allocates thruster forces, and sends them to a pluggable actuator backend (Gazebo or PX4).

USTROV 外部低层控制栈。读取融合状态、计算控制输出、分配推力、通过可替换后端（Gazebo 或 PX4）下发执行。

## Scope / 职责边界

This stack is responsible for:
1. Reading estimated state from a unified interface (`/odometry/filtered`).
2. Computing low-level 4-DOF control outputs (X/Y/Z/Yaw).
3. Allocating generalized wrench to 8 thruster forces via pseudo-inverse.
4. Sending thruster commands through a pluggable actuator backend.

本栈的职责：
1. 从统一状态接口（`/odometry/filtered`）读取估计状态。
2. 计算 4 自由度低层控制输出（X/Y/Z/Yaw）。
3. 通过伪逆矩阵将广义力/力矩分配为 8 路推进器推力。
4. 通过可替换的执行后端下发推力指令。

This stack does **NOT** replace:
- PX4 flight controller core (mode state machine, failsafe, built-in estimators, driver framework).
- PX4 is still responsible for: offboard mode admission, arm/disarm authorization, actuator PWM/ESC output (in PX4 backend mode).
- Gazebo is only the simulation execution environment and sensor source.

本栈**不**替代：
- PX4 飞控内核（模式状态机、failsafe、内建估计器、驱动框架）。
- PX4 仍负责：offboard 接入、arm/disarm 授权、PWM/ESC 输出（PX4 后端模式下）。
- Gazebo 仅为仿真执行环境与传感器来源。

## Architecture / 架构分层

```
 State Source Layer        Control Layer         Allocation Layer      Actuation Backend Layer
 ┌──────────────┐    ┌───────────────────┐    ┌──────────────┐    ┌─────────────────────┐
 │ EKF          │    │ controller_node   │    │ allocator    │    │ GazeboBackend       │
 │ /odometry/   │───>│ 4-DOF PID         │───>│ B+ * tau     │───>│   or                │
 │  filtered    │    │ (X, Y, Z, Yaw)    │    │ -> 8 thrusts │    │ PX4ActuatorBackend  │
 └──────────────┘    └───────────────────┘    └──────────────┘    └─────────────────────┘
                            │ (PX4 only)
                     ┌──────┴──────┐
                     │ PX4Interface│
                     │ (mode/arm)  │
                     └─────────────┘
```

Four layers:
1. **State source layer** — EKF provides fused state via `/odometry/filtered`.
2. **Control layer** — `controller_node.py` computes 4-DOF PID control outputs.
3. **Allocation layer** — `allocator.py` maps 6-DOF wrench to 8 thruster forces.
4. **Actuation backend layer** — pluggable backend sends thruster commands to Gazebo or PX4.

Additionally, `px4_interface.py` is a **mode interface** (not an actuator backend): it handles offboard heartbeat, mode switching, and arm/disarm — only active in PX4 backend mode.

四层架构：
1. **状态源层** — EKF 通过 `/odometry/filtered` 提供融合状态。
2. **控制层** — `controller_node.py` 计算 4 自由度 PID 控制输出。
3. **分配层** — `allocator.py` 将 6 自由度力矩映射为 8 路推进器推力。
4. **执行后端层** — 可替换后端将推力指令发送到 Gazebo 或 PX4。

此外，`px4_interface.py` 是**模式接口**（非执行后端）：负责 offboard 心跳、模式切换、arm/disarm——仅在 PX4 后端模式下激活。

## Code Map / 文件职责

### Core / 核心模块

| File | Role |
|------|------|
| `main.py` | Entry point, spins `USTROVDirectController` / 入口 |
| `controller_node.py` | 4-DOF control loop (50 Hz), orchestrates PID + allocation + backend / 4 自由度控制循环 |
| `pid.py` | Generic PID with integral clamp + filtered derivative / 通用 PID |
| `allocator.py` | 6-DOF wrench to 8-thruster allocation (pseudo-inverse) / 推力分配 |
| `state_estimator.py` | State abstraction; `EKFEstimator` subscribes `/odometry/filtered` / 状态估计抽象 |

### Actuation Backend / 执行后端

| File | Role |
|------|------|
| `actuator_backend.py` | Abstract base class for actuator backends / 执行后端抽象基类 |
| `backend_factory.py` | Factory: creates `(actuator_backend, mode_interface)` from parameter / 工厂函数 |
| `px4_actuator.py` | **PX4 actuator backend**: normalizes thrust and publishes `ActuatorMotors` / PX4 执行后端 |
| `px4_interface.py` | **PX4 mode interface**: offboard heartbeat + arm/disarm (not an actuator backend) / PX4 模式接口 |
| `sim/gz_thruster.py` | **Gazebo actuator backend**: sends thrust via Gazebo transport / Gazebo 执行后端 |

### Target Interface / 目标接口

The controller accepts target pose via ROS topic:

控制器通过 ROS topic 接收目标位姿：

- **Topic**: `/ustrov/target_pose`
- **Type**: `geometry_msgs/PoseStamped`
- **Fields**:
  - `pose.position.x` -> target X (NED north, meters)
  - `pose.position.y` -> target Y (NED east, meters)
  - `pose.position.z` -> target depth (**positive = deeper**, NED down convention)
  - `pose.orientation` -> target yaw (quaternion; only yaw is used, roll/pitch ignored)
- **frame_id**: `"ned"` (NED convention: z > 0 means underwater)

**Priority**: topic target overrides environment variable defaults. Environment variables (`USTROV_TARGET_X/Y/DEPTH/YAW`) serve only as startup defaults.

**优先级**：topic 目标覆盖环境变量默认值。环境变量仅作为启动默认目标。

### Sensor Bridges / 传感器桥接

Bridges convert raw sensor data into EKF-consumable standard ROS messages. Simulation uses `sim/sim_sensors.py` to publish the same topics directly.

Bridge 将原始传感器数据转换为 EKF 可融合的标准 ROS 消息。仿真模式下由 `sim/sim_sensors.py` 直接发布相同话题。

| Bridge | Input | Output (EKF) | Used by |
|--------|-------|--------------|---------|
| `imu_bridge.py` | `/fmu/out/sensor_combined` + `/fmu/out/vehicle_attitude` | `/sensor/imu` (`Imu`) | `ekf_real.yaml` |
| `dvl_bridge.py` | `/dvl/data` + `/dvl/position` | `/sensor/dvl` (`TwistWithCovarianceStamped`) + `/sensor/dvl_pose` (`PoseWithCovarianceStamped`) | `ekf_real.yaml` |
| `depth_bridge.py` | `/fmu/out/sensor_baro` | `/sensor/depth` (`PoseWithCovarianceStamped`) | both `ekf.yaml` / `ekf_real.yaml` |
| `sim/sim_sensors.py` | Gazebo dynamic pose | `/sensor/imu` + `/sensor/dvl` + `/sensor/depth` | `ekf.yaml` |

### Hardware Support / 硬件支持

| File | Role |
|------|------|
| `manual_control.py` | Joystick teleoperation + open-loop thrust testing / 手柄遥控 |
| `monitor_px4.py` | Real-time sensor monitoring dashboard / 传感器监控面板 |

### Config / 配置

| File | Role |
|------|------|
| `config/ekf.yaml` | EKF config for simulation / 仿真 EKF 配置 |
| `config/ekf_real.yaml` | EKF config for real hardware / 实机 EKF 配置 |

### Simulation (`sim/`) / 仿真模块

| File | Role |
|------|------|
| `sim/sim_sensors.py` | Gazebo truth -> noisy IMU/DVL/Depth / 仿真传感器噪声注入 |
| `sim/error_analyzer.py` | EKF vs ground truth visualization / EKF 误差可视化 |
| `sim/visual_ekf_node.py` | ArUco-based visual localization / ArUco 视觉定位 |
| `sim/usv_simulator.py` | USV spawn + GPS-like odometry / 水面 USV 仿真 |
| `sim/start_perception_brain.sh` | One-command simulation perception startup / 一键启动仿真感知 |

## Topics / 关键话题

### Published by control stack / 控制栈发布

| Topic | Type | Backend |
|-------|------|---------|
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | PX4 only |
| `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | PX4 only |
| `/fmu/in/actuator_motors` | `px4_msgs/ActuatorMotors` | PX4 only |
| `/model/.../cmd_thrust` (x8) | Gazebo `Double` | Gazebo only |

### Consumed by control stack / 控制栈订阅

| Topic | Type |
|-------|------|
| `/odometry/filtered` | `nav_msgs/Odometry` |
| `/ustrov/target_pose` | `geometry_msgs/PoseStamped` |

## Run Modes / 运行模式

### Unified bringup (recommended) / 统一启动（推荐）

```bash
# Gazebo simulation: sensors + EKF + controller (one command)
ros2 launch src/rov_direct_control/launch/sim_control.launch.py

# PX4 real hardware: bridges + EKF + controller (one command)
ros2 launch src/rov_direct_control/launch/px4_control.launch.py
```

### Manual startup / 手动分步启动

#### Gazebo backend

```bash
# Step 1: Start simulation perception + EKF
bash src/rov_direct_control/sim/start_perception_brain.sh

# Step 2: Start controller
python3 src/rov_direct_control/main.py --ros-args -p actuator_backend:=gazebo
```

#### PX4 backend

Requirements: Micro XRCE Agent running, PX4 running, DVL connected.

```bash
# Step 1: Start bridges
python3 src/rov_direct_control/imu_bridge.py &
python3 src/rov_direct_control/dvl_bridge.py &
python3 src/rov_direct_control/depth_bridge.py &

# Step 2: Start EKF
ros2 run robot_localization ekf_node --ros-args --params-file src/rov_direct_control/config/ekf_real.yaml &

# Step 3: Start controller
python3 src/rov_direct_control/main.py --ros-args -p actuator_backend:=px4
```

### Publish target at runtime / 运行中发布目标

```bash
# Single target: x=1.0 y=0.0 depth=5.0 yaw=0.0
python3 src/rov_direct_control/tools/publish_target_pose.py 1.0 0.0 5.0 0.0
```

### Manual motor test / 手动电机测试

```bash
ros2 run rov_direct_control manual_control.py
```

## Quick Troubleshooting / 快速排查

### No publisher on `/fmu/in/actuator_motors`

```bash
ros2 node list
ros2 topic info /fmu/in/actuator_motors
```

Check if `main.py` is alive and not crashing.

### Micro XRCE bind error (`errno 98`)

```bash
ss -lunp | grep 8888
```

Stop duplicate agent process.

### EKF not updating

```bash
ros2 topic hz /odometry/filtered
ros2 topic echo /sensor/imu --once
ros2 topic echo /sensor/dvl --once
ros2 topic echo /sensor/depth --once
```

### Armed but no thrust response

```bash
ros2 topic echo /fmu/in/offboard_control_mode --once
ros2 topic echo /fmu/in/actuator_motors --once
```
