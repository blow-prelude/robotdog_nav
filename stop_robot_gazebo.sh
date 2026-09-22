#!/bin/bash
# Stop the moving robot dog - Direct Gazebo method

echo "Stopping robot dog in Gazebo..."

# Method 1: Use gz command to stop all links
gz service -s /world/scene/set_pose \
  --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req "name: 'go2' x: 0 y: 0 z: 0.275 roll: 0 pitch: 0 yaw: 0" 2>/dev/null

# Method 2: Reset the model
gz service -s /world/scene/reset \
  --reqtype gz.msgs.WorldReset \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req "" 2>/dev/null

echo "Commands sent. Check Gazebo window."
echo ""
echo "Alternative manual methods:"
echo "1. In Gazebo GUI: Right-click robot → 'Reset Model'"
echo "2. Or restart Gazebo with: ros2 launch robot_scene go2_lidar_gps.launch.py"
