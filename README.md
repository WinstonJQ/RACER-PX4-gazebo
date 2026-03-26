# RACER-PX4-Gazebo 多无人机协同探索

基于 RACER 算法的 PX4-Gazebo 四架无人机协同探索仿真系统。

## 主要特性

- **px4ctrl 姿态控制器**: 替换原有简单速度控制器，实现更接近真实飞行的姿态控制
- **多机协同探索**: 4 架 iris 无人机在室内场景中自主集群探索
- **一键启动**: 集成启动脚本，自动完成所有节点初始化和解锁流程
- **稳定 odometry**: 60Hz ground truth 发布，避免 MAVROS 高负载下丢包问题

## 系统架构

- **RACER**: 快速协同探索算法 (Rapid Collaborative Exploration)
- **PX4**: 无人机飞控仿真 (SITL)
- **Gazebo**: 物理仿真环境
- **ROS**: 机器人操作系统 (Melodic/Noetic)
- **px4ctrl**: 姿态控制器 (来自 Fast-Exploration)

## 环境依赖

### 1. 系统要求
- Ubuntu 18.04 / 20.04
- ROS Melodic / Noetic
- Gazebo 9 / 11

### 2. 必备软件包

```bash
# PX4 固件 (需要提前安装)
# 参考: https://docs.px4.io/main/en/simulation/ros_interface.html

# ROS 依赖
sudo apt-get install -y \
    ros-$ROS_DISTRO-mavros \
    ros-$ROS_DISTRO-mavros-extras \
    ros-$ROS_DISTRO-gazebo-ros \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-tf \
    ros-$ROS_DISTRO-cv-bridge

# 其他依赖
sudo apt-get install -y \
    libeigen3-dev \
    libpcl-dev \
    libnlopt-dev
```

## 快速开始

### 1. 克隆仓库

```bash
cd ~/
git clone https://github.com/WinstonJQ/RACER-PX4-gazebo.git
cd RACER-PX4-gazebo
```

### 2. 编译

```bash
# 编译
catkin_make

# 设置环境变量
source devel/setup.bash
# 或者添加到 ~/.bashrc
echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
```

### 3. 一键启动四机协同探索

```bash
# 确保已经在 RACER-PX4-gazebo 目录
cd ~/RACER-PX4-gazebo
source devel/setup.bash

# 运行启动脚本
./racer_start.sh
```

**启动流程：**
1. Gazebo + MAVROS (`indoor31.launch`)
2. 位姿桥接 (`get_local_pose_new.py iris 4`)
3. Ground truth odom (`gazebo_odom_pub.py iris 4`)
4. px4ctrl (`multi_px4ctrl.launch`)
5. 交互式解锁起飞 (`unlock_multi.py`)
6. RACER 探索 (`swarm_exploration.launch`)

**手动分步启动（如需调试）：**

```bash
# 终端 1: 启动 PX4 仿真
roslaunch racer_bringup indoor31.launch

# 终端 2: 启动位姿桥接
python3 ./src/px4_gazebo_racer/scripts/get_local_pose_new.py iris 4

# 终端 3: 启动 ground truth odometry
python3 ./src/px4_gazebo_racer/scripts/gazebo_odom_pub.py iris 4

# 终端 4: 启动 px4ctrl
roslaunch px4ctrl multi_px4ctrl.launch

# 终端 5: 交互式解锁（依次输入 y 解锁 4 架飞机）
python3 ./src/px4_gazebo_racer/scripts/unlock_multi.py

# 终端 6: 启动探索算法
roslaunch exploration_manager swarm_exploration.launch
```

## 项目结构

```
RACER-PX4-gazebo/
├── racer_start.sh               # 一键启动脚本
├── src/px4_gazebo_racer/
│   ├── px4ctrl/                 # px4ctrl 姿态控制器
│   │   ├── launch/multi_px4ctrl.launch
│   │   └── config/ctrl_param_fpv.yaml
│   ├── racer_bringup/           # Gazebo + MAVROS 启动
│   │   ├── launch/indoor31.launch
│   │   └── worlds/indoor32.world
│   ├── RACER/                   # RACER 集群探索算法
│   │   └── swarm_exploration/
│   ├── control/                 # 旧速度控制器（已弃用）
│   └── scripts/                 # 辅助脚本
│       ├── gazebo_odom_pub.py   # Ground truth 发布
│       ├── unlock_multi.py      # 交互式解锁
│       └── get_local_pose_new.py
└── README.md
```

## 关键配置

### px4ctrl 参数

配置文件: `src/px4_gazebo_racer/px4ctrl/config/ctrl_param_fpv.yaml`

| 参数 | 值 | 说明 |
|------|-----|------|
| `no_RC` | true | SITL 无遥控器模式 |
| `enable_auto_arm` | false | 手动解锁（通过 unlock_multi.py） |
| `takeoff_height` | 1.0m | 起飞高度 |
| `msg_timeout/odom` | 1.0s | odometry 超时阈值 |

Topic 映射:
- `~odom` → `/iris_X/ground_truth/odom`
- `~cmd` → `/planning/pos_cmd_{X+1}`
- `~takeoff_land` → `/iris_X/takeoff_land`

### 探索参数

配置文件: `src/px4_gazebo_racer/RACER/swarm_exploration/exploration_manager/launch/single_drone_planner.xml`

| 参数 | 值 | 说明 |
|------|-----|------|
| `box_min/max_x` | -8.5 / 8.5 | X 方向边界 (17m) |
| `box_min/max_y` | -9.5 / 9.5 | Y 方向边界 (19m) |
| `box_min/max_z` | 0 / 1.5 | Z 方向边界 (1.5m) |
| `max_vel` | 0.5 m/s | 最大速度 |
| `max_acc` | 0.5 m/s² | 最大加速度 |
| `max_yawdot` | 90°/s | 最大转向速度 |

## 仿真场景说明

### indoor32.world

- **尺寸**: 17m x 19m 的室内环境（砖块围墙）
- **高度**: 砖墙高 3m，探索限制在 1.5m 高度
- **无人机初始位置**:
  - iris_0: (0, 0, 0)
  - iris_1: (0, -2, 0)
  - iris_2: (2, 0, 0)
  - iris_3: (2, -2, 0)

## 主要修改记录

### v1.0 - px4ctrl 控制器迁移
- ✅ 用 px4ctrl 姿态控制器替换 4 个独立速度控制器
- ✅ 新增 `gazebo_odom_pub.py` 提供 60Hz 稳定 ground truth odometry
- ✅ 新增 `unlock_multi.py` 交互式解锁脚本
- ✅ 补充 `quadrotor_msgs` 缺失字段（jerk, TakeoffLand, Px4ctrlDebug）

### v1.1 - 参数优化
- ✅ 探索范围调整为实际地图尺寸 17×19×1.5m
- ✅ 降低飞行速度 1.0 → 0.5 m/s，改善稳定性
- ✅ 降低转向速度 120° → 90°/s
- ✅ 删除 world 文件中车辆/树木模型

## 常见问题

### 1. Gazebo 启动失败

```bash
# 检查 gazebo 模型路径
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find racer_bringup)/models

# 重新编译
catkin_make clean
catkin_make
```

### 2. px4ctrl 出现 CMD_CTRL → MANUAL_CTRL

检查 `gazebo_odom_pub.py` 是否已启动，确保 `/iris_X/ground_truth/odom` 正常发布（60Hz）。

### 3. 无法解锁

检查：
- MAVROS 连接状态（`rostopic echo /iris_0/mavros/state`）
- px4ctrl 节点是否正常运行（`rosnode list | grep px4ctrl`）
- odom 数据是否正常（`rostopic hz /iris_0/ground_truth/odom`）

## 参考资料

- [RACER Paper](https://arxiv.org/abs/2011.06994) - RACER 算法论文
- [Fast-Drone-250](https://github.com/ZJU-FAST-Lab/Fast-Drone-250) - px4ctrl 来源
- [PX4 Documentation](https://docs.px4.io/) - PX4 官方文档
- [XTDrone](https://github.com/robin-shaun/XTDrone) - 多机仿真框架参考

## 作者

- **WinstonJQ** - 1451850927@qq.com

## 许可证

MIT License
