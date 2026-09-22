#!/bin/bash
# Install Gazebo ROS packages

echo "=========================================="
echo "Gazebo ROS Packages Installation"
echo "=========================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run with sudo:"
    echo "  sudo $0"
    exit 1
fi

echo "Updating package lists..."
apt update

echo ""
echo "Installing Gazebo ROS packages..."
apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev

echo ""
echo "Verifying installation..."
if [ -f "/opt/ros/humble/lib/libgazebo_ros_init.so" ]; then
    echo "✅ libgazebo_ros_init.so installed successfully"
else
    echo "❌ libgazebo_ros_init.so not found"
    echo ""
    echo "Trying to find alternative location..."
    find /usr -name "libgazebo_ros_init.so" 2>/dev/null
fi

echo ""
echo "Installation complete!"
echo ""
echo "Next steps:"
echo "1. Clean Gazebo cache: rm -rf ~/.gazebo/sdf"
echo "2. Test launch: ros2 launch robot_scene go2_lidar_gps.launch.py"
