# RACER-PX4-Gazebo 多无人机协同探索

基于 RACER 算法的 PX4-Gazebo 四架无人机协同探索仿真系统。

## 系统架构

- **RACER**: 快速协同探索算法 (Rapid Collaborative Exploration)
- **PX4**: 无人机飞控仿真 (SITL)
- **Gazebo**: 物理仿真环境
- **ROS**: 机器人操作系统 (Melodic/Noetic)

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
# 初始化 catkin 工作空间
catkin_init_workspace src/

# 编译
catkin_make

# 或者使用 catkin_tools (推荐)
catkin build

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

**脚本说明：**
- 脚本会依次启动以下节点：
  1. **PX4 仿真环境** - 启动 Gazebo 并加载四架无人机 (iris_0 ~ iris_3)
  2. **位姿获取节点** - 获取无人机的局部位置信息
  3. **四个控制器** - 每架无人机的独立控制节点
  4. **探索管理器** - RACER 协同探索算法

**手动分步启动（如需调试）：**

```bash
# 终端 1: 启动 PX4 仿真
roslaunch racer_bringup indoor31.launch

# 等待 Gazebo 完全启动后，终端 2: 启动位姿获取
python3 ./src/px4_gazebo_racer/scripts/get_local_pose_new.py iris 4

# 终端 3-6: 分别启动四个控制器
rosrun control control_iris0
rosrun control control_iris1
rosrun control control_iris2
rosrun control control_iris3

# 终端 7: 启动探索算法
roslaunch exploration_manager swarm_exploration.launch
```

## 项目结构

```
RACER-PX4-gazebo/
├── assets/                      # 资源文件备份
│   ├── launch/                  # Launch 文件备份
│   └── worlds/                  # Gazebo 场景文件备份
├── src/
│   └── px4_gazebo_racer/
│       ├── racer_bringup/       # 启动包
│       │   ├── launch/
│       │   │   └── indoor31.launch    # 四机仿真启动文件
│       │   └── worlds/
│       │       └── indoor32.world     # Gazebo 场景
│       ├── control/             # 无人机控制
│       ├── RACER/               # RACER 算法核心
│       │   ├── swarm_exploration/
│       │   └── uav_simulator/
│       └── scripts/             # 辅助脚本
├── racer_start.sh               # 一键启动脚本
└── README.md                    # 本文件
```

## 仿真场景说明

### indoor32.world

- **尺寸**: 约 20m x 20m 的室内环境
- **障碍物**: 四周环绕砖墙障碍物
- **无人机初始位置**:
  - iris_0: (0, 0, 0)
  - iris_1: (0, -2, 0)
  - iris_2: (2, 0, 0)
  - iris_3: (2, -2, 0)

## 参数配置

### 修改无人机数量

编辑 `src/px4_gazebo_racer/scripts/get_local_pose_new.py`:
```python
# 修改无人机数量
num_drones = 4  # 改为需要的数量
```

### 修改探索参数

编辑 `src/px4_gazebo_racer/RACER/swarm_exploration/exploration_manager/config/` 下的 YAML 文件。

## 常见问题

### 1. Gazebo 启动失败

```bash
# 检查 gazebo 模型路径
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find racer_bringup)/models

# 重新编译
catkin_make clean
catkin_make
```

### 2. 无人机无法解锁

检查终端输出，确保：
- PX4 仿真完全启动
- 控制器节点正常运行
- 位姿数据正常发布

### 3. 无法连接到 MAVROS

检查 `indoor31.launch` 中的端口配置是否与控制器匹配。

## 参考资料

- [RACER Paper](https://arxiv.org/abs/xxx) - RACER 算法论文
- [PX4 Documentation](https://docs.px4.io/) - PX4 官方文档
- [XTDrone](https://github.com/robin-shaun/XTDrone) - 多机仿真框架参考

## 作者

- **WinstonJQ** - 1451850927@qq.com

## 许可证

MIT License
