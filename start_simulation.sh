#!/bin/bash
# Quick start script for robotdog_nav simulation and navigation

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR" || exit

echo "=========================================="
echo "RobotDog Nav - Quick Start"
echo "=========================================="
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash

# Check if install directory exists
if [ ! -d "install/robot_scene" ]; then
    echo "⚠️  robot_scene package not found. Building..."
    echo ""
    echo "Step 1: Building gazebo_quadbot packages..."
    colcon build --packages-select \
        champ_msgs champ_description champ \
        go2_config go2_description \
        go2w_config go2w_description go2w_control \
        mid360_simulation \
        --symlink-install 2>&1 | tail -5
    
    echo ""
    echo "Step 2: Building robot_scene..."
    colcon build --packages-select robot_scene --symlink-install 2>&1 | tail -5
    
    if [ ! -d "install/robot_scene" ]; then
        echo ""
        echo "❌ Build failed. Please check the error messages above."
        echo ""
        echo "Alternative: Try building all packages at once:"
        echo "  colcon build --symlink-install"
        exit 1
    fi
    echo ""
    echo "✅ Build completed!"
fi

echo ""
echo "Sourcing workspace..."
source install/setup.bash

echo ""
echo "Starting Gazebo simulation..."
echo ""

# Start Gazebo with Go2 robot
ros2 launch robot_scene go2_lidar_gps.launch.py
