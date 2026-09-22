# Gazebo ROS 插件安装指南

## 问题诊断

```bash
./fix_gazebo.sh
```

输出显示：
```
❌ libgazebo_ros_init.so 不存在
```

## 解决方案

### 方法 1：安装 Gazebo ROS 包（推荐）

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev
```

安装后验证：
```bash
ls /opt/ros/humble/lib/libgazebo_ros_init.so
```

### 方法 2：使用现有插件（临时方案）

如果无法安装新包，尝试使用系统已有的 Gazebo 插件：

```bash
# 检查系统已有的插件
ls /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/

# 如果有 libgazebo_ros_init.so，创建软链接
sudo ln -s /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libgazebo_ros_init.so \
          /opt/ros/humble/lib/libgazebo_ros_init.so
```

### 方法 3：修改 launch 文件不使用该插件

编辑 `go2_lidar_gps.launch.py`，移除对 `libgazebo_ros_init.so` 的依赖。

## 验证安装

```bash
# 检查插件是否存在
ls -la /opt/ros/humble/lib/libgazebo_ros_init.so

# 检查 Gazebo ROS 命令
ros2 run gazebo_ros gzserver --help
```

## 启动测试

安装完成后：

```bash
# 清理缓存
rm -rf ~/.gazebo/sdf

# 重新启动
ros2 launch robot_scene go2_lidar_gps.launch.py
```

## 常见错误

### 错误：无法定位软件包 ros-humble-gazebo-ros-pkgs

**解决**:
```bash
# 添加 ROS2 仓库
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 更新并安装
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs
```

### 错误：插件加载失败

**解决**:
```bash
# 检查 LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH

# 或添加到 .bashrc
echo "export LD_LIBRARY_PATH=/opt/ros/humble/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc
source ~/.bashrc
```
