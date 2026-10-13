#!/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR" || exit

if [ ! -d "install" ]; then
    echo "❌ 未找到 install 目录，请先执行：colcon build --symlink-install"
    exit 1
fi

source /opt/ros/humble/setup.bash
source install/setup.bash

# DDS 域隔离：与局域网其他 ROS 设备（默认域 0）隔离，防止 /clock 等话题串扰
# （见 doc/ros_domain_clock_conflict.md）；跨机通信对端须使用相同域 ID
export ROS_DOMAIN_ID=42

case "$1" in
    dognav) 
        echo "启动航路点导航节点..."
        ros2 launch waypoint_nav waypoint_nav.launch.py use_sim_time:=true 
        ;;
    gpsloc) 
        echo "启动 GPS 定位融合节点..."
        ros2 launch gps_imu_fusion gps_imu_fusion.launch.py use_sim_time:=true 
        ;;
    taskserver)
        echo "启动指控端任务服务节点..."
        ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1
        ;;
    full)
        echo "启动完整导航系统（GPS + 导航 + 任务服务）..."
        # 定位(gps_imu_fusion)与导航(waypoint_nav)已解耦，需分别启动；更完整的一键启动见 ./start_system.sh
        ros2 launch gps_imu_fusion gps_imu_fusion.launch.py use_sim_time:=true &
        sleep 2
        ros2 launch waypoint_nav waypoint_nav.launch.py use_sim_time:=true &
        sleep 2
        ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1
        ;;
    *) 
        echo "用法：$0 [dognav|gpsloc|taskserver|full]"
        echo "  dognav      - 航路点导航节点"
        echo "  gpsloc      - 仅启动 GPS 定位融合节点"
        echo "  taskserver  - 指控端任务服务节点 (execute_task_server)"
        echo "  full        - 完整导航系统 (GPS+ 导航 + 任务服务)"
        exit 1 
        ;;
esac
