# 机器狗持续运动问题排查指南

## 问题现象

运行 `ros2 launch robot_scene go2_lidar_gps.launch.py` 启动 Gazebo 后，机器狗一直在运动。

## 可能原因

### 1. champ_bringup 启动了控制器 ⚠️

`go2_lidar_gps.launch.py` 中包含了 `champ_bringup`，它会：
- 启动关节控制器
- 启动运动控制器
- 可能发布初始速度命令

**查看 launch 文件**:
```yaml
# go2_lidar_gps.launch.py
bringup_ld = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("champ_bringup"),
            "launch",
            "bringup.launch.py",
        )
    ),
    launch_arguments={
        "gazebo": "true",
        "close_loop_odom": "true",  # ← 闭环里程计
        ...
    }.items(),
)
```

### 2. 有节点在持续发布 `/cmd_vel`

可能是：
- `waypoint_nav` 节点
- `teleop_twist_keyboard`
- 其他导航控制器

### 3. 物理仿真问题

- 机器狗模型与地面接触不良
- 摩擦系数设置问题
- 关节 PID 参数不合适

---

## 解决方案

### 方案 1：使用修改版 launch 文件（推荐）⭐

启动禁用闭环里程计的版本：

```bash
# 使用静态版本，禁用 close_loop_odom
ros2 launch robot_scene go2_lidar_gps_static.launch.py
```

这个版本修改了以下参数：
- `close_loop_odom: false` - 禁用闭环里程计，防止机器狗自动"纠正"位置
- `publish_foot_contacts: false` - 减少控制器活动

### 方案 2：停止速度命令

```bash
# 终端 1：发布零速度
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

如果机器狗停止，说明有节点在发布速度命令。

### 方案 2：查找谁在发布 cmd_vel

```bash
# 查看发布者信息
ros2 topic info /cmd_vel --verbose
```

输出示例：
```
Publisher count: 1
Node name: nav_waypoint_node  # ← 这是发布者
```

找到后停止对应的节点。

### 方案 3：修改 launch 文件禁用 champ 控制器

创建修改版 launch 文件：

```bash
# 复制原始文件
cp src/gazebo_quadbot/robot_scene/launch/go2_lidar_gps.launch.py \
   src/gazebo_quadbot/robot_scene/launch/go2_lidar_gps_static.launch.py

# 编辑文件，注释掉 champ_bringup
# 或者修改 launch 参数
```

### 方案 4：在 Gazebo GUI 中重置

1. 在 Gazebo 窗口中右键点击机器狗
2. 选择 **"Reset Model"** 或 **"Reset Model Poses"**
3. 或者点击菜单栏 **Edit → Reset Model**

### 方案 5：使用 Gazebo 命令行重置

```bash
# 重置模型
gz service -s /world/scene/reset \
  --reqtype gz.msgs.WorldReset \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req ""
```

---

## 推荐的测试流程

### 仅启动仿真（不启动控制器）

```bash
# 修改 launch 参数，禁用某些组件
ros2 launch robot_scene go2_lidar_gps.launch.py lite:=true
```

### 检查控制器状态

```bash
# 查看关节控制器
ros2 topic list | grep joint

# 查看控制器状态
ros2 node list | grep controller
```

### 使用键盘控制测试

```bash
# 启动键盘控制（这样可以手动控制机器狗）
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 按空格键停止
```

---

## 根本原因分析

根据 `go2_lidar_gps.launch.py` 文件，问题出在：

```python
bringup_ld = IncludeLaunchDescription(
    ...
    launch_arguments={
        ...
        "close_loop_odom": "true",  # 闭环里程计启用
        ...
    }.items(),
)
```

`close_loop_odom: true` 会启用闭环里程计，这可能导致机器狗尝试"纠正"位置而产生运动。

### 建议修改

创建自定义 launch 文件，设置：
```python
"close_loop_odom": "false",  # 禁用闭环里程计
"publish_foot_contacts": "false",  # 禁用足端接触发布
```

---

## 快速解决脚本

```bash
#!/bin/bash
# stop_robot.sh

# 方法 1：发布零速度
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

echo "Robot stop command sent."

# 方法 2：检查发布者
echo ""
echo "Checking /cmd_vel publishers:"
ros2 topic info /cmd_vel --verbose
```

---

## 如果是仿真物理问题

如果上述方法都无效，可能是物理仿真问题：

1. **检查 URDF 模型** - 关节限位、摩擦系数
2. **检查 Gazebo 插件配置** - IMU、GPS、接触传感器
3. **检查世界文件** - 地面摩擦系数、重力设置

### 临时解决方案

在 Gazebo GUI 中：
1. **Edit → Model Properties**
2. 勾选 **"Self Collide"**
3. 调整 **Mass** 和 **Inertia** 参数
