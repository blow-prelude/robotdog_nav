# Gazebo 启动问题解决方案

## 问题现象

```
[ERROR] [gzserver-3]: process has died [pid XXXX, exit code 255]
[ERROR] [spawn_entity.py-5]: Service /spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?
```

## 原因分析

1. **Gazebo 版本不匹配** - 系统安装 Gazebo 11 (Classic)，但配置可能使用 Ignition/Garden 语法
2. **World 文件路径问题** - world 文件不存在或路径错误
3. **Gazebo ROS 插件缺失** - `libgazebo_ros_init.so` 未安装或路径错误
4. **配置文件问题** - `gazebo.yaml` 配置不正确

## 解决方案

### 方案 1：安装缺失的 Gazebo ROS 包（推荐）

```bash
# 使用安装脚本
sudo ./install_gazebo_ros.sh

# 或手动安装
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev
```

验证安装：
```bash
ls /opt/ros/humble/lib/libgazebo_ros_init.so
```

### 方案 2：运行诊断脚本

```bash
./fix_gazebo.sh
```

### 方案 2：安装缺失的 Gazebo ROS 包

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev
```

### 方案 3：使用简化启动（推荐）

如果 `go2_lidar_gps.launch.py` 启动失败，尝试简化版本：

```bash
# 只启动机器人描述，不加载复杂世界
ros2 launch robot_scene go2_description.launch.py
```

### 方案 4：手动启动 Gazebo

```bash
# 终端 1：启动 Gazebo server
gzserver /media/lenovo/1/project/gazebo/0713/robotdog_nav/install/robot_scene/share/robot_scene/worlds/agriculture.world &

# 终端 2：启动 Gazebo client
gzclient &

# 等待 5 秒
sleep 5

# 终端 3：spawn 机器人
source install/setup.bash
ros2 launch robot_scene spawn_go2.launch.py
```

### 方案 5：检查并修复 world 文件

```bash
# 检查 world 文件
ls -la install/robot_scene/share/robot_scene/worlds/

# 如果 agriculture.world 损坏，尝试其他 world
# 或重新构建
colcon build --packages-select robot_scene --symlink-install
```

### 方案 6：清理 Gazebo 缓存

```bash
# 清理缓存
rm -rf ~/.gazebo/sdf
rm -rf ~/.gazebo/gz
rm -rf ~/.gazebo/models

# 重启 Gazebo
killall gzserver gzclient
```

## 替代方案：使用其他启动文件

如果 `go2_lidar_gps.launch.py` 持续失败，尝试：

```bash
# Go2 基础版本
ros2 launch robot_scene go2_base.launch.py

# Go2w（轮式版本）
ros2 launch robot_scene go2w_lidar_gps.launch.py

# Diffbot
ros2 launch robot_scene diffbot_lidar_gps.launch.py
```

## 检查清单

- [ ] Gazebo 11 已安装：`gzserver --version`
- [ ] ROS2 Gazebo 包已安装：`dpkg -l | grep gazebo-ros`
- [ ] World 文件存在：`ls install/robot_scene/share/robot_scene/worlds/`
- [ ] URDF 文件正确：`ros2 run robot_state_publisher go2_description`
- [ ] 环境变量已设置：`source install/setup.bash`

## 常见错误及解决

### 错误：`libgazebo_ros_init.so` 找不到

**解决**:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

### 错误：World 文件不存在

**解决**:
```bash
# 重新构建 robot_scene
colcon build --packages-select robot_scene --symlink-install
```

### 错误：`spawn_entity` 服务不可用

**解决**:
1. 确保 gzserver 已启动
2. 等待 10 秒让服务可用
3. 检查 Gazebo ROS 插件是否加载

### 错误：Gazebo 崩溃 (exit code 255)

**解决**:
```bash
# 清理缓存
rm -rf ~/.gazebo/sdf

# 检查 world 文件语法
gz sdf -p install/robot_scene/share/robot_scene/worlds/agriculture.world > /dev/null
```

## 联系支持

如果以上方法都无效，请提供：
1. `gzserver --version` 输出
2. 完整的错误日志
3. `dpkg -l | grep gazebo` 输出
