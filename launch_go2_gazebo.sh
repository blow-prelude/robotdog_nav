#!/bin/bash
# Complete launch script for Go2 robot in Gazebo with standing control

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR" || exit

echo "=========================================="
echo "Go2 Robot - Gazebo Simulation"
echo "=========================================="
echo ""

source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

# Check if robot_scene is available
if ! ros2 pkg prefix robot_scene &>/dev/null; then
    echo "❌ robot_scene package not found!"
    echo "   Building... (this may take a while)"
    echo ""
    cd "$SCRIPT_DIR"
    colcon build --packages-select robot_scene go2_description mid360_simulation --symlink-install 2>&1 | tail -5
    source install/setup.bash
fi

echo "Starting Gazebo with Go2 robot..."
echo ""

# Launch Gazebo with Go2
ros2 launch robot_scene go2_lidar_gps.launch.py &
GAZEBO_PID=$!

# Wait for Gazebo to start
sleep 5

echo ""
echo "✅ Gazebo started"
echo ""
echo "To control the robot:"
echo "  - Keyboard: ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo "  - Stand up: ros2 launch go2_gazebo stand_controller.launch.py (if available)"
echo ""
echo "Press Ctrl+C to stop Gazebo"
wait $GAZEBO_PID
