#!/bin/bash
# USTROV 完整启动脚本：PX4 + Gazebo + DDS Agent
# 控制节点已内置 gz.transport 直连 Gazebo，不再需要 ros_gz_bridge！
# 用法: bash start_all.sh

set -e

echo "=== [1/3] 清理残留进程 ==="
pkill -9 -f "bin/px4" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
sleep 2

echo "=== [2/3] 启动 PX4 SITL + Gazebo ==="
cd ~/PX4-Autopilot
make px4_sitl gz_ustrov &
PX4_PID=$!
sleep 20
echo "PX4 + Gazebo 已启动 (PID: $PX4_PID)"

echo "=== [3/3] 启动 Micro-XRCE-DDS Agent ==="
MicroXRCEAgent udp4 -p 8888 &
DDS_PID=$!
sleep 3
echo "DDS Agent 已启动 (PID: $DDS_PID)"

echo ""
echo "============================================"
echo "  基础设施就绪！（不再需要 ros_gz_bridge）"
echo "  现在在新终端中运行控制节点:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ~/ws_ustrov/install/setup.bash"
echo "  python3 ~/ws_ustrov/src/rov_direct_control/main.py"
echo "============================================"
echo ""
echo "按 Ctrl+C 终止所有进程"

trap "kill $PX4_PID $DDS_PID 2>/dev/null; exit" INT TERM
wait
