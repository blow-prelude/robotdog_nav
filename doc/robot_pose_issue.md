# 机器狗姿态问题解决方案

## 问题现象

1. 机器狗在 Gazebo 中**直立**（不趴下）
2. 执行 `./launch.sh dognav` 后机器狗**静止不动**

## 原因分析

### 1. 机器狗直立原因

在 `go2_robot_VLP.xacro` 中：
```xml
<joint name="floating_base" type="fixed">
  <parent link="base_link"/>
  <child link="trunk"/>
</joint>
```

`floating_base` 关节类型是 `fixed`，将机器狗固定在空间中，不受重力影响。

**解决方案**: 这是正常的设计。Gazebo 会通过 `gazebo_ros2_control` 插件控制关节，让机器狗保持站立姿态。

### 2. 机器狗静止原因

`launch.sh dognav` 只启动**导航节点**，不发送关节控制命令。

导航节点发布 `/cmd_vel` → 需要控制器将速度命令转换为关节力矩 → 机器狗才能运动。

## 解决方案

### 方案 1：使用键盘控制（测试用）

```bash
# 终端 1：启动 Gazebo
ros2 launch robot_scene go2_lidar_gps.launch.py

# 终端 2：键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 按键说明：
# i - 前进
# , - 后退
# j - 左转
# l - 右转
# 空格 - 停止
```

### 方案 2：使用导航系统

```bash
# 终端 1：启动 Gazebo
ros2 launch robot_scene go2_lidar_gps.launch.py

# 终端 2：启动定位
ros2 launch gps_imu_fusion gps_imu_fusion.launch.py use_sim_time:=true

# 终端 3：启动导航
./launch.sh dognav

# 终端 4：发送航点
ros2 topic pub --once /waypoint_goals msg_set_msgs/msg/MultiGoal "{...}"
```

### 方案 3：检查控制器状态

```bash
# 检查控制器是否加载
ros2 control list_controllers

# 检查关节状态
ros2 topic echo /joint_states

# 检查 cmd_vel
ros2 topic echo /cmd_vel
ros2 topic info /cmd_vel --verbose
```

### 方案 4：手动发送站立命令（如果支持）

某些配置需要发送初始关节角度让机器狗站立：

```bash
# 发布关节角度命令（示例）
ros2 topic pub --once /joint_group_effort_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory "{
    joint_names: ['lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint', ...],
    points: [{
      positions: [0.0, 0.8, -1.6, ...],  # 站立角度
      time_from_start: {sec: 2, nanosec: 0}
    }]
  }"
```

## 完整测试流程

### 步骤 1：启动 Gazebo

```bash
ros2 launch robot_scene go2_lidar_gps.launch.py
```

### 步骤 2：检查控制器

```bash
# 等待 10 秒让控制器加载
sleep 10

# 检查控制器状态
ros2 control list_controllers
```

期望输出：
```
joint_states_controller[joint_state_broadcaster] active
joint_group_effort_controller[joint_trajectory_controller] active
```

### 步骤 3：测试运动

```bash
# 键盘控制测试
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 步骤 4：测试导航

```bash
# 启动导航
./launch.sh dognav

# 发送航点
python3 test_execute_task_py.py
```

## 常见问题

### Q: 控制器未加载

**症状**: `ros2 control list_controllers` 显示控制器 inactive 或不存在

**解决**:
```bash
# 激活控制器
ros2 control switch_controllers --start joint_group_effort_controller
```

### Q: 机器狗腿在抖动

**原因**: PID 参数不合适或物理仿真问题

**解决**:
1. 调整 `ros_control.yaml` 中的 PID 参数
2. 检查 Gazebo 物理引擎设置
3. 尝试增加 `kp` 和 `kd` 值

### Q: 机器狗趴在地上

**原因**: 关节力矩不足或控制器未正确加载

**解决**:
1. 检查控制器状态
2. 增加 PID 的 `p` 值
3. 确保 Gazebo 正确加载了 `gazebo_ros2_control` 插件

## 高级配置

### 修改 URDF 让机器狗初始趴下

编辑 `go2_robot_VLP.xacro`，修改关节初始角度：

```xml
<joint name="lf_hip_joint" type="revolute">
  ...
  <limit effort="..." velocity="..." 
       lower="-0.5" upper="0.5"/>
  <!-- 添加初始角度 -->
  <safety_controller k_soft="100" k_hard="100"/>
</joint>
```

### 使用不同的启动配置

```bash
# 使用静态版本（禁用闭环里程计）
ros2 launch robot_scene go2_lidar_gps_static.launch.py

# 使用 Go2w（轮式版本）
ros2 launch robot_scene go2w_lidar_gps.launch.py
```

## 相关文档

- [ROS2 Control 文档](https://control.ros.org/)
- [Gazebo ROS2 Control](https://gazebosim.org/docs/garden/ros_gazebo/)
- [Unitree Go2 文档](https://support.unitree.com/)
