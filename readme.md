USTROV / PanorAUV 直接控制系统 (ROS 2 + PX4)
本项目是基于 ROS 2 (Humble) 与 PX4 飞控的 8 推进器水下机器人（ROV）直接控制（Offboard）代码库。核心功能包括：底层推力分配（Control Allocation）、PID 姿态与深度控制、多传感器数据融合（EKF）以及软硬件通讯桥接。

📦 1. 环境准备与编译 (首次安装必看)
本系统依赖 ROS 2 和 PX4 的 DDS 通信机制，在运行代码前，必须确保你的工作空间（Workspace）配置正确。

1.1 系统要求
操作系统: Ubuntu 22.04 LTS

ROS 2 版本: Humble

飞控固件: PX4 Autopilot (建议 v1.14 及以上)

1.2 创建工作空间与拉取依赖代码

```bash
# 1. 创建并进入 ROS 2 工作空间的 src 目录
mkdir -p ~/ws_ustrov/src
cd ~/ws_ustrov/src

# 2. 拉取 PX4 官方的 ROS 2 通信与消息定义库
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git

# 3. 拉取 DVL 传感器驱动与消息包（可选）
$ git clone https://github.com/paagutie/dvl_msgs.git
$ git clone --recurse-submodules https://github.com/paagutie/dvl-a50.git

# 4. 拉取本 ROV 代码仓库
git clone https://github.com/Airbian/PanorAUV.git

cd ..
colcon build

# 将环境写入 bashrc（只需执行一次），确保每次打开终端都能找到程序
echo "source ~/ws_ustrov/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

💻 2. 仿真模式 (Simulation)
⚠️ 核心检查: 启动前，请打开主程序（main.py 或 controller_node.py）和状态估计（state_estimator.py），确保将推力执行后端设置为仿真：self.thruster_backend = 'gazebo'。

步骤 1：启动 PX4 仿真环境 (在 PX4-Autopilot 源码目录下执行)

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_ustrov
```

步骤 2：启动 DDS 通信桥接 (连接 PX4 与 ROS 2)

```bash
cd ~/Micro-XRCE-DDS-Agent
MicroXRCEAgent udp4 -p 8888
```

步骤 3：启动感知与控制系统 (打开新终端)

```bash
# 1. 启动传感器与视觉大脑脚本
~/ws_ustrov/src/rov_direct_control/sim/start_perception_brain.sh 

# 2. 启动主控程序
python3 ~/ws_ustrov/src/rov_direct_control/main.py
```

🌊 3. 实机模式 (Real Hardware)
⚠️ 核心检查: 下水前，必须将代码后端调整至实机：self.thruster_backend = 'px4'。
硬件注意: 由于实机水下线缆较长（>2米），为防止 TTL 寄生电容导致信号严重丢包，DDS 串口波特率已降至 115200（飞控端参数也必须同步修改）。

步骤 1：启动底层串口通信

```bash
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 115200
```

步骤 2：启动传感器与数据桥接 (推荐使用 tmux 或打开多个终端运行)

```bash
# 1. 启动 DVL 驱动节点
ros2 launch dvl_a50 dvl_a50.launch.py ip_address:='192.168.194.95'

# 2. 启动数据桥接 (将 DVL 和气压计转为标准位姿)
python3 ~/ws_ustrov/src/rov_direct_control/dvl_bridge.py
python3 ~/ws_ustrov/src/rov_direct_control/depth_bridge.py

# 3. 启动 EKF 传感器融合节点
ros2 run robot_localization ekf_node --ros-args --params-file ~/ws_ustrov/src/rov_direct_control/config/ekf_real.yaml
```

步骤 3：启动测试与主控

```bash
# (下水前强烈推荐) 手柄开环推力测试，验证各桨叶转向
# 需要先运行: ros2 run joy joy_node
python3 ~/ws_ustrov/src/rov_direct_control/manual_control.py

# (可选) PX4 状态监控板
python3 ~/ws_ustrov/src/rov_direct_control/monitor_px4.py

# 启动闭环主控程序
python3 ~/ws_ustrov/src/rov_direct_control/main.py
```

🛠️ 4. 故障排除 (Troubleshooting)
在日常开发与联调中，如果遇到奇怪的报错，请优先查阅以下解决方案：

4.1 串口连接被拒绝 / ttyUSB0 不存在
现象：运行 DDS Agent 时提示 Permission denied 或串口被占用。
原因：Ubuntu 默认安装的盲文显示器驱动 (brltty) 会霸占 CH340/CP2102 等 USB 串口设备。
解决：彻底卸载该驱动：

```bash
sudo systemctl stop brltty-udev.service
sudo systemctl mask brltty-udev.service
sudo systemctl stop brltty.service
sudo systemctl disable brltty.service
sudo apt-get remove --purge brltty -y
```

4.2 端口被占用 / 节点启动失败
现象：启动 DDS 提示 UDP 8888 端口已被使用，或 Gazebo 崩溃卡死。
解决：一键强制清理相关残留进程：

```bash
pkill -f MicroXRCEAgent
killall -9 px4 gz ruby
```

4.3 仿真模型修改未生效
解决：清除 PX4 的缓存配置并重新编译：

```bash
cd ~/PX4-Autopilot
rm -rf build/px4_sitl_default/
rm -f build/px4_sitl_default/bin/parameters*.bson
make px4_sitl
```

4.4 其他常用维护指令
```bash
# 切换至 Ubuntu 备用内核 (解决内核升级导致的网卡/显卡驱动掉线)
sudo grub-reboot 2
sudo reboot

# 启动地面站 QGroundControl
./QGroundControl-x86_64.AppImage
```
