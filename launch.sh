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
    dognav) ros2 launch waypoint_nav waypoint_nav.launch.py use_sim_time:=true ;;
    gpsloc) ros2 launch gps_imu_fusion gps_imu_fusion.launch.py use_sim_time:=true ;;
    *) echo "用法：$0 [dognav|gpsloc]"
       echo "  dognav - 航路点导航启动（定位 gps_imu_fusion 已解耦，需单独启动）"
       echo "  gpsloc - 仅启动 GPS 定位融合节点"
       exit 1 ;;
esac
