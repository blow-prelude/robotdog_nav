# 快速测试指南

如果你只是想快速测试 `execute_task_server` 功能，而不需要 Gazebo 仿真，请按照以下步骤：

## 方案 A：仅测试 execute_task_server（无需仿真）

### 1. 构建必要的包

```bash
cd /media/lenovo/1/project/gazebo/0713/robotdog_nav
source /opt/ros/humble/setup.bash

# 只构建 execute_task_server 和依赖
colcon build --packages-select execute_task_server msg_set_msgs --symlink-install
```

### 2. 启动节点并测试

```bash
source install/setup.bash

# 终端 1：启动 execute_task_server
ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1

# 终端 2：运行测试脚本
python3 test_execute_task_py.py
```

### 3. 验证结果

如果看到以下输出，说明测试成功：

```
Test 1: Local waypoints (3 points)
Response: {"message":"Task accepted and executing","mission_id":"local_test_01","status":"success","waypoint_count":3}

Test 2: GPS waypoints (2 points)
Response: {"message":"Task accepted and executing","mission_id":"gps_test_01","status":"success","waypoint_count":2}
```

---

## 方案 B：测试完整导航链路（需要 waypoint_nav）

### 1. 构建导航相关包

```bash
cd /media/lenovo/1/project/gazebo/0713/robotdog_nav
source /opt/ros/humble/setup.bash

# 构建导航相关包（约 5-10 分钟）
colcon build --packages-select waypoint_nav gps_imu_fusion execute_task_server msg_set_msgs --symlink-install
```

### 2. 启动导航系统

```bash
source install/setup.bash

# 终端 1：启动 GPS/IMU 融合（提供定位）
ros2 run gps_imu_fusion gps_imu_fusion_node

# 终端 2：启动航路点导航
ros2 run waypoint_nav nav_waypoint_node --ros-args \
  -p use_sim_time:=true \
  -p use_controller_server:=false

# 终端 3：启动执行服务
ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1

# 终端 4：运行测试
python3 test_execute_task_py.py
```

---

## 方案 C：完整仿真测试（需要 Gazebo）

> ⚠️ **注意**: 首次构建需要 20-30 分钟，包含 Gazebo 仿真环境、机器人模型等。

### 1. 构建所有包

```bash
cd /media/lenovo/1/project/gazebo/0713/robotdog_nav
source /opt/ros/humble/setup.bash

# 清理旧构建（可选）
rm -rf build/ install/ log/

# 构建所有包（首次构建耗时较长）
colcon build --symlink-install
```

### 2. 启动完整系统

```bash
# 终端 1：启动 Gazebo 仿真
source install/setup.bash
ros2 launch robot_scene go2_lidar_gps.launch.py

# 终端 2：启动定位和导航
source install/setup.bash
ros2 launch gps_imu_fusion gps_imu_fusion.launch.py use_sim_time:=true &
ros2 launch waypoint_nav waypoint_nav.launch.py use_sim_time:=true

# 终端 3：启动执行服务
source install/setup.bash
ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1
```

### 3. 发送测试指令

```bash
source install/setup.bash
python3 test_execute_task_py.py
```

---

## 常见问题

### Q1: `robot_scene` 包找不到

**原因**: 未构建 gazebo_quadbot 相关包。

**解决**:
```bash
# 方案 1：构建所有包
colcon build --symlink-install

# 方案 2：只构建必要的包（推荐）
colcon build --packages-select \
  champ_msgs champ_description champ \
  go2_config go2_description \
  go2w_config go2w_description go2w_control \
  mid360_simulation robot_scene \
  --symlink-install
```

### Q2: 构建失败，提示缺少依赖

**解决**:
```bash
# 清理构建缓存
rm -rf build/ install/ log/

# 重新构建
colcon build --symlink-install
```

### Q3: 只想测试 execute_task_server 功能

**解决**: 使用方案 A，只构建 `execute_task_server` 和 `msg_set_msgs`：
```bash
colcon build --packages-select execute_task_server msg_set_msgs --symlink-install
python3 test_execute_task_py.py
```

---

## 测试脚本说明

`test_execute_task_py.py` 会执行以下测试：

1. **局部坐标航点** - 发送 3 个局部坐标点
2. **GPS 坐标航点** - 发送 2 个 GPS 坐标点
3. **Agent ID 验证** - 测试错误的 Agent ID 是否被拒绝
4. **能力类型验证** - 测试不支持的能力类型是否被拒绝

所有测试通过后，会显示：
```
==================================================
All tests completed!
==================================================
```

---

## 推荐方案

| 需求 | 推荐方案 | 构建时间 |
|------|---------|---------|
| 仅测试 execute_task_server | 方案 A | < 1 分钟 |
| 测试导航算法链路 | 方案 B | 5-10 分钟 |
| 完整仿真测试 | 方案 C | 20-30 分钟 |
