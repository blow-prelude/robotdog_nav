# ROS 2 Bag 录制仿真数据与回放测试操作手册

## 1. 用途与原理

本文描述如何完整运行一遍仿真，用 `ros2 bag` 录制传感器话题，之后**关闭仿真、回放录制数据代替 Gazebo** 来驱动导航栈，用于测试数据流可通性与算法可行性。

### 数据流对照

仿真时（方案 C）：

```
Gazebo → /gps/data + /livox/imu → gps_imu_fusion → /world_odom + TF world→base_link
Gazebo → /livox/lidar ────────────────────────────→ nav_waypoint_node / Nav2 代价地图
```

回放时结构完全相同，只是数据源由 Gazebo 换成 `ros2 bag play`：

```
ros2 bag play → /gps/data + /livox/imu → gps_imu_fusion → /world_odom + TF world→base_link
ros2 bag play → /livox/lidar ────────────────────→ nav_waypoint_node / Nav2 代价地图
```

### 时钟策略（关键）

录制数据的时间戳是**仿真时间**（Gazebo `/clock` 的时间系）。回放时若节点仍用系统壁钟，时间对不上会导致 Nav2 代价地图查 TF 失败、点云全部被丢弃。因此：

- **不录 `/clock`**，回放时用 `ros2 bag play --clock` 合成发布（频率 40 Hz）。合成时钟与回放消息的时间戳天然一致，且避免与录制的 `/clock` 形成双发布者冲突。
- 回放时所有消费节点以 `use_sim_time:=true` 启动，分别传给 `gps_imu_fusion.launch.py` 与 `waypoint_nav.launch.py`（后者继续向 controller_server、lifecycle_manager 注入；两者已解耦，需分别启动）。
- 若确需离线分析 `/clock`，可单独录制，但回放驱动导航栈时必须用 `--topics` 将其排除。

### 开环限制（必读）

回放是**开环**的：录制的传感器数据不会响应回放期间产生的 `/cmd_vel`，"机器人"不会按新指令改变轨迹。因此这种方式：

- ✅ 可验证：数据流可通（话题→融合→TF→规划→控制逐级出数）、节点不崩溃、控制输出与录制时原次输出对照合理、回放可重复（同一 bag 结果确定，适合回归）。
- ❌ 不可验证：闭环导航能否真正到达目标（仍需 Gazebo 或实机验证）。

## 2. 前置条件

- 已按 QUICK_START.md 构建工作空间；每个终端先执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

- 本机固定 `ROS_DOMAIN_ID=42`（见 doc/ros_domain_clock_conflict.md），录制与回放须同域。
- 完整仿真（方案 C）可正常运行。

## 3. 录制

### 3.1 录制话题清单

| 话题 | 类型 | 频率 | 回放时的作用 |
|---|---|---|---|
| `/gps/data` | sensor_msgs/NavSatFix | 30 Hz | fusion 输入，重算 `/world_odom` 与 TF |
| `/livox/imu` | sensor_msgs/Imu | 200 Hz | fusion 输入 |
| `/livox/lidar` | sensor_msgs/PointCloud2 | 10 Hz（约 2 万点/帧） | 代价地图 + 导航节点感知 |
| `/cmd_vel` | geometry_msgs/Twist | 10 Hz | 仅作对照：录制时原次控制输出 |
| `/nav_status` | std_msgs/String | 事件驱动 | 仅作对照：原次状态流转时序 |

> `/cmd_vel`、`/nav_status` 不参与回放驱动（回放时由导航节点现场发布），只用于离线对照。

### 3.2 操作步骤

1. **启动完整仿真**（3 个终端，同 QUICK_START.md 方案 C）：

```bash
# 终端 1：Gazebo 仿真
ros2 launch robot_scene go2_lidar_gps.launch.py

# 终端 2：定位 + 导航（两包已解耦，分别启动）
ros2 launch gps_imu_fusion gps_imu_fusion.launch.py use_sim_time:=true &
ros2 launch waypoint_nav waypoint_nav.launch.py use_sim_time:=true

# 终端 3：任务服务
ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1
```

2. **先起录制**（终端 4），确保 bag 覆盖整个任务窗口：

```bash
ros2 bag record \
  -o sim_bag_$(date +%Y%m%d_%H%M%S) \
  /gps/data /livox/imu /livox/lidar /cmd_vel /nav_status
```

> 雷达每帧约 2 万点，bag 体积约 300–500 MB/分钟，单次录制建议控制在 2–3 分钟内。

3. **下发任务**（终端 5）。使用**局部坐标**航点（下例 3 点，与 test_full_system.sh Test 1 相同；GPS 用例在 origin 为 0,0 时坐标会映射到极远处，勿用于录制）：

```bash
ros2 service call /agent/uav1/execute_task execute_task_server/srv/ExecuteTask "{
  request_json: '{\"mission_id\":\"bag_record\",\"agent_id\":\"uav1\",\"capability_id\":\"goto_waypoints\",\"params\":{\"waypoint_frame\":\"local\",\"waypoints\":[{\"x\":1.0,\"y\":0.0,\"z\":0.0},{\"x\":2.0,\"y\":1.0,\"z\":0.0},{\"x\":0.0,\"y\":2.0,\"z\":0.0}]}}'
}"
```

4. 终端 2 观察 `/nav_status` 走到 `COMPLETED`（或另开终端 `ros2 topic echo /nav_status`），随后 **Ctrl-C 停止录制**。

5. 检查录制结果：

```bash
ros2 bag info sim_bag_20XXXXXX_XXXXXX
```

确认各话题均有消息且 `/livox/lidar` 频率接近 10 Hz。

## 4. 回放

### 4.1 回放前检查

1. **确认仿真已完全关闭**，无残留 `/clock` 发布者（残留 Gazebo 会与合成时钟冲突）：

```bash
ros2 topic info /clock        # Publisher count 必须为 0
pkill -f gzserver; pkill -f gzclient   # 如有残留
```

### 4.2 操作步骤

1. **回放传感器数据**（终端 1，先起数据源，仿"仿真先跑"的惯例）：

```bash
ros2 bag play sim_bag_20XXXXXX_XXXXXX \
  --clock 40 \
  --topics /gps/data /livox/imu /livox/lidar
```

> `--clock 40`：以 40 Hz 合成发布 `/clock`；`--topics` 只回放 3 个传感器话题——`/cmd_vel`、`/nav_status` 必须排除，否则会与导航节点现场发布的同名话题冲突。

2. **启动定位 + 导航**（终端 2，`use_sim_time` 必须为 true；两包已解耦，分别启动）：

```bash
ros2 launch gps_imu_fusion gps_imu_fusion.launch.py use_sim_time:=true &
ros2 launch waypoint_nav waypoint_nav.launch.py use_sim_time:=true
```

3. **启动任务服务**（终端 3）：

```bash
ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1
```

4. **尽快下发与录制时完全相同的任务**（趁回放位姿尚在录制起点附近）。任务内容与录制时不同或下发过晚都会使结果失真，原因见 6.1。

5. 观察执行（终端 4）：

```bash
ros2 topic echo /nav_status          # 状态流转
ros2 topic echo /cmd_vel --field linear   # 控制输出（对照录制时的原次输出）
```

## 5. 验收点

| 检查项 | 命令/方式 | 通过标准 |
|---|---|---|
| 融合定位出数 | `ros2 topic hz /world_odom` | ≈10 Hz |
| TF 正常 | `ros2 run tf2_ros tf2_echo world base_link` | 数值随回放轨迹连续变化，无查找失败 |
| 代价地图 | RViz 或 controller_server 日志 | 点云进入代价地图，无 TF 超时/外推报错 |
| 状态流转 | `ros2 topic echo /nav_status` | EXECUTING_PATH → 各航点 GOAL_REACHED → COMPLETED |
| 控制输出 | `/cmd_vel` 对照 | 量级/方向与录制时 `/cmd_vel` 原次输出一致 |

全部满足即证明：录制数据可完整驱动导航栈，数据流可通，算法在真实仿真数据上运行正常。

## 6. 限制与常见问题

### 6.1 任务下发时机（回放 FAILED 的最常见原因）

回放位姿是"已走过的轨迹"。录制时机器人用约 T 秒走完任务；回放时若下发过晚，位姿已越过目标点，而直线规划与纯跟踪控制器不支持倒车（`allow_backward: false`），机器人会原地绕圈直到超时 FAILED。**务必在 bag 起播后立刻下发与录制时相同的任务。**

### 6.2 RViz 中 markers / path 不显示

原因：`nav_waypoint_node` 的 launch 启动项未注入 `use_sim_time`，其发布的 `/waypoint_markers`、`/path_markers`、`/actual_path` 带系统壁钟时间戳（约为仿真钟的 5×10⁶ 倍），RViz 切到仿真时间后认为这些消息"来自未来"而挂起。对策（二选一）：

- 回放时接受仅以 `/nav_status`、`/cmd_vel`、controller_server 日志判断结果（不影响功能验证）；
- 或在 `waypoint_nav/launch/waypoint_nav.launch.py` 的 `waypoint_nav_node` 参数中追加一行 `{'use_sim_time': use_sim_time}` 后重建该包。

### 6.3 QoS 不兼容导致收不到数据

Gazebo 插件的发布 QoS 会被 `ros2 bag record` 自动记录并在回放时复现，一般无需处理。若回放时订阅端收不到数据，用 QoS 覆盖文件强制 best_effort：

```bash
ros2 bag play <bag> --qos-profile-overrides-path qos_override.yaml
```

### 6.4 回放倍速

`--rate 0.5` 慢放、`--rate 2.0` 倍速。注意合成时钟与消息时间戳同步缩放，节点逻辑不受影响；但慢放/倍速会改变控制周期与消息到达周期的相对关系，对照分析时建议保持 1.0。

### 6.5 服务调用报 "The passed service type is invalid"

原因：该终端没有 source 工作空间。`execute_task_server/srv/ExecuteTask` 只注册在 `install/setup.bash` 中，仅 source `/opt/ros/humble` 时找不到该类型。先在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

验证环境就绪：`ros2 interface show execute_task_server/srv/ExecuteTask` 能打印出 `request_json` 定义即可。

### 6.6 同一 bag 的重复回放是确定性的

相同 bag + 相同任务 + 相同配置，每次回放的 `/world_odom`、`/cmd_vel` 输出一致，适合做算法改动的回归对照；这也意味着 bag 不能替代仿真做闭环调参。
