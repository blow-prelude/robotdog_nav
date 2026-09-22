#!/bin/bash
# Stop the moving robot dog

source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

echo "Stopping robot dog..."

# Publish zero velocity
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

echo "Robot dog stopped."
echo ""
echo "To check what's publishing cmd_vel:"
echo "  ros2 topic info /cmd_vel --verbose"
