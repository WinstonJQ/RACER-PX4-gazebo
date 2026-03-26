#!/bin/bash

# 创建新终端执行命令函数（每个命令独立标签页/窗口）
run_in_new_tab() {
    gnome-terminal --tab --title="$1" -- bash -ic "echo '执行命令: $2'; $2; exec bash"
}

# 由于PX4启动较慢，优先启动并延长等待时间
run_in_new_tab "PX4仿真" "roslaunch racer_bringup indoor31.launch"
echo "等待PX4启动..."
sleep 10  # 重要！PX4启动需要足够时间，可根据实际情况调整

# 按顺序执行后续命令
run_in_new_tab "位姿获取" "python3 ./src/px4_gazebo_racer/scripts/get_local_pose_new.py iris 4"
sleep 2

run_in_new_tab "GT Odom" "python3 ./src/px4_gazebo_racer/scripts/gazebo_odom_pub.py iris 4"
sleep 1

run_in_new_tab "px4ctrl" "roslaunch px4ctrl multi_px4ctrl.launch"
sleep 3

run_in_new_tab "解锁起飞" "python3 ./src/px4_gazebo_racer/scripts/unlock_multi.py"
sleep 2

run_in_new_tab "探索" "roslaunch exploration_manager swarm_exploration.launch"

echo "所有命令已启动！建议手动排列窗口位置以便观察。"
