#!/bin/bash
# Gazebo 启动诊断和修复脚本

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR" || exit

echo "=========================================="
echo "Gazebo 启动诊断"
echo "=========================================="
echo ""

source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

# 1. 检查 Gazebo 版本
echo "1. 检查 Gazebo 版本:"
gzserver --version 2>&1 | head -3
echo ""

# 2. 检查 world 文件是否存在
WORLD_FILE="$SCRIPT_DIR/install/robot_scene/share/robot_scene/worlds/agriculture.world"
if [ -f "$WORLD_FILE" ]; then
    echo "✅ World 文件存在：$WORLD_FILE"
else
    echo "❌ World 文件不存在：$WORLD_FILE"
    echo "   尝试查找其他 world 文件..."
    find "$SCRIPT_DIR/install" -name "*.world" 2>/dev/null | head -5
fi
echo ""

# 3. 检查 Gazebo 插件
echo "3. 检查 Gazebo ROS 插件:"
if [ -f "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libgazebo_ros_init.so" ]; then
    echo "✅ libgazebo_ros_init.so 存在"
else
    echo "❌ libgazebo_ros_init.so 不存在"
    echo "   尝试安装：sudo apt install ros-humble-gazebo-ros-pkgs"
fi
echo ""

# 4. 清理 Gazebo 缓存
echo "4. 清理 Gazebo 缓存..."
rm -rf ~/.gazebo/sdf 2>/dev/null
rm -rf ~/.gazebo/gz 2>/dev/null
echo "   缓存已清理"
echo ""

# 5. 提供启动建议
echo "=========================================="
echo "建议的启动方式"
echo "=========================================="
echo ""
echo "方法 1：使用 Gazebo Classic 兼容模式"
echo "  gzserver /media/lenovo/1/project/gazebo/0713/robotdog_nav/install/robot_scene/share/robot_scene/worlds/agriculture.world &"
echo ""
echo "方法 2：使用简化启动（不加载复杂世界）"
echo "  ros2 launch robot_scene go2_description.launch.py"
echo ""
echo "方法 3：手动启动 Gazebo 并加载机器人"
echo "  gzserver &"
echo "  gzclient &"
echo "  ros2 launch robot_scene spawn_go2.launch.py"
echo ""
