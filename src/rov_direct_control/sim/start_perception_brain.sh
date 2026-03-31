#!/bin/bash

set -eo pipefail

# 优雅的颜色定义
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${CYAN}======================================================${NC}"
echo -e "${GREEN}      🚀 最强大脑·多源异构视觉 EKF 融合底座启动      ${NC}"
echo -e "${CYAN}======================================================${NC}"

# 按脚本位置自动推导工作空间根目录，避免硬编码路径失效
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

cleanup_stale_processes() {
	echo -e "${YELLOW}[预清理] 终止残留仿真/EKF 进程，保证干净启动...${NC}"

	# 仅清理本项目的仿真脚本及 EKF 进程，避免多次启动叠加。
	pkill -f "$SCRIPT_DIR/sim_sensors.py" >/dev/null 2>&1 || true
	pkill -f "$SCRIPT_DIR/usv_simulator.py" >/dev/null 2>&1 || true
	pkill -f "$SCRIPT_DIR/visual_ekf_node.py" >/dev/null 2>&1 || true
	pkill -f "$SCRIPT_DIR/error_analyzer.py" >/dev/null 2>&1 || true
	pkill -f "/robot_localization/ekf_node" >/dev/null 2>&1 || true
}

graceful_shutdown() {
	echo -e "\n\033[91m⚠️ 接收到退出信号，正在安全关闭所有神经系统...\033[0m"

	for pid_var in PID_SENSORS PID_USV PID_VISUAL PID_EKF; do
		pid="${!pid_var:-}"
		if [[ -n "$pid" ]] && kill -0 "$pid" >/dev/null 2>&1; then
			kill "$pid" >/dev/null 2>&1 || true
		fi
	done

	wait >/dev/null 2>&1 || true
}

cleanup_stale_processes

# Source工作空间
source /opt/ros/humble/setup.bash
source "$WS_ROOT/install/setup.bash"

echo -e "${YELLOW}[1/4] 启动带有真实噪声的物理深水传感器群 (sim_sensors)...${NC}"
python3 "$SCRIPT_DIR/sim_sensors.py" &
PID_SENSORS=$!

echo -e "${YELLOW}[2/4] 投入水面带有绝对 RTK-GPS 的伴飞无人船 (usv_simulator)...${NC}"
python3 "$SCRIPT_DIR/usv_simulator.py" &
PID_USV=$!

echo -e "${YELLOW}[3/4] 睁开 OpenCV 天眼追踪水面相对位姿 (visual_ekf)...${NC}"
python3 "$SCRIPT_DIR/visual_ekf_node.py" &
PID_VISUAL=$!

echo -e "${YELLOW}[4/4] 唤醒 EKF 滤波中枢，开始数据镇压融合 (robot_localization)...${NC}"
ros2 run robot_localization ekf_node --ros-args --params-file "$WS_ROOT/src/rov_direct_control/config/ekf.yaml" &
PID_EKF=$!

# 核心安全机制：捕捉退出信号，一键优雅关闭所有后台节点
trap graceful_shutdown SIGINT SIGTERM EXIT

sleep 2
echo -e "${GREEN}✅ 神经中枢部署完毕！正在进入上帝视角分析仪控制台...${NC}"

# 快速自检，确认关键话题和节点已经在线
echo -e "${CYAN}--- ROS 2 启动自检 ---${NC}"
ros2 node list || true
ros2 topic list | grep -E '^/sensor/(imu|dvl|depth)$|^/odometry/filtered$' || true

sleep 2

# 前身堵塞：在这唯一的终端前台运行面板，展示漂亮的刷新 UI
python3 "$SCRIPT_DIR/error_analyzer.py"
