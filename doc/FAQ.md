# 常见问题解答 (FAQ)

## Q1: 启动 Gazebo 后机器狗一直在运动怎么办？

**现象**: 运行 `ros2 launch robot_scene go2_lidar_gps.launch.py` 后，机器狗持续运动。

**原因**: 
- `champ_bringup` 启动了闭环里程计 (`close_loop_odom: true`)
- 机器狗尝试"纠正"位置导致持续运动
- 可能有节点在持续发布 `/cmd_vel` 速度命令

**解决方案**:

### 方法 1：使用静态版本 launch 文件（推荐）

```bash
ros2 launch robot_scene go2_lidar_gps_static.launch.py
```

这个版本禁用了闭环里程计，机器狗不会自动运动。

### 方法 2：手动停止

```bash
# 发布零速度命令
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 方法 3：在 Gazebo GUI 中重置

1. 右键点击机器狗
2. 选择 **"Reset Model"**

### 详细文档

查看 [doc/robot_moving_issue.md](doc/robot_moving_issue.md) 获取完整排查指南。

---

## Q2: `robot_scene` 包找不到怎么办？

**原因**: 未构建 gazebo_quadbot 相关包。

**解决方案**:

```bash
# 清理并重新构建
cd /media/lenovo/1/project/gazebo/0713/robotdog_nav
rm -rf build/ install/ log/
colcon build --symlink-install
```

或只构建必要的包：
```bash
colcon build --packages-select \
  champ_msgs champ_description champ \
  go2_config go2_description \
  go2w_config go2w_description go2w_control \
  mid360_simulation robot_scene \
  --symlink-install
```

---

## Q3: 如何快速测试 execute_task_server 功能？

**答案**: 不需要启动 Gazebo 仿真，只需：

```bash
# 1. 构建
colcon build --packages-select execute_task_server msg_set_msgs

# 2. 启动服务
ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1

# 3. 运行测试
python3 test_execute_task_py.py
```

参考 [QUICK_START.md](QUICK_START.md) 获取详细步骤。

---

## Q4: 构建失败，提示缺少依赖？

**解决方案**:

```bash
# 1. 清理构建缓存
rm -rf build/ install/ log/

# 2. 更新依赖
sudo apt update
sudo apt install -y nlohmann-json3-dev

# 3. 重新构建
colcon build --symlink-install
```

---

## Q5: waypoint_nav 不执行航点怎么办？

**检查清单**:

1. 检查是否有 `/world_odom` 数据：
   ```bash
   ros2 topic echo /world_odom
   ```

2. 检查 `/waypoint_goals` 话题是否有消息：
   ```bash
   ros2 topic echo /waypoint_goals
   ```

3. 检查节点日志：
   ```bash
   ros2 topic echo /rosout --filter 'node_name=="nav_waypoint_node"'
   ```

4. 确认 `is_gps_aid` 参数设置正确：
   - GPS 坐标：`is_gps_aid: true`
   - 局部坐标：`is_gps_aid: false`

---

## Q6: 如何与 multi_task_bridge 集成？

**步骤**:

1. 构建 multi_task_bridge：
   ```bash
   cd /media/lenovo/1/project/gazebo/multi_task_bridge
   colcon build --symlink-install
   ```

2. 启动所有节点：
   ```bash
   # Terminal 1: 仿真和导航
   ros2 launch robot_scene go2_lidar_gps_static.launch.py &
   ros2 launch gps_imu_fusion gps_imu_fusion.launch.py use_sim_time:=true &
   ros2 launch waypoint_nav waypoint_nav.launch.py use_sim_time:=true &
   
   # Terminal 2: 执行服务
   ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1
   
   # Terminal 3: 桥接节点
   ros2 launch multi_task_bridge multi_task_bridge.launch.py drone_name:=uav1
   ```

3. 发送 MultiTask 测试。

---

## Q7: GPS 坐标和局部坐标如何转换？

**答案**: `execute_task_server` 会自动处理：

- `waypoint_frame: "gps"` - 自动转换为 ENU 局部坐标
- `waypoint_frame: "local"` - 直接使用局部坐标

转换使用 GeographicLib 库，基于配置的 origin 坐标。

---

## 获取更多帮助

- 详细测试指南：[QUICK_START.md](QUICK_START.md)
- 机器狗运动问题：[doc/robot_moving_issue.md](doc/robot_moving_issue.md)
- 系统架构：[README.md](README.md#系统架构)
