# ROS 2 Bag 录制仿真数据与回放测试操作手册

## 1. 用途与原理

本文描述如何完整运行一遍仿真，用 `ros2 bag` 录制话题，之后**关闭仿真、回放录制数据代替 Gazebo** 来驱动导航栈，用于测试数据流可通性与算法可行性。

推荐方式为**仅回放雷达 + 里程计**（`/livox/lidar` + `/world_odom`）：TF 由 `nav_replay_support/odom_to_tf_node` 从回放的里程计重建，无需录制/回放 GPS、IMU 原始数据，也不用启动 `gps_imu_fusion`。旧的"回放 GPS/IMU + 融合节点"流程保留在 §7 作为对照。

### 数据流对照

仿真时（方案 C）：

```
Gazebo → /gps/data + /livox/imu → gps_imu_fusion → /world_odom + TF world→base_link
Gazebo → /livox/lidar ────────────────────────────→ nav_waypoint_node / Nav2 代价地图
Gazebo → robot_state_publisher ──→ TF base_link→livox
```

回放时（推荐，仅雷达 + 里程计）：

```
ros2 bag play → /world_odom ──→ odom_to_tf_node ────→ TF world→base_link
ros2 bag play → /livox/lidar ───────────────────────→ nav_waypoint_node / Nav2 代价地图
replay_tf.launch.py ──────────→ 静态 TF base_link→livox
```

导航层只依赖契约：`/world_odom` + TF `world→base_link` + TF `base_link→livox`，不感知定位数据从何而来。回放时**不启动** `gps_imu_fusion`（避免重复的 `/world_odom` 与 `world→base_link` 发布者），导航 launch **保持默认 `simulation:=true`**（静态 `base_link→livox` 由 `replay_tf.launch.py` 发布，传 `simulation:=false` 会造成重复静态 TF）。

### 时钟策略（关键）

录制数据的时间戳是**仿真时间**（Gazebo `/clock` 的时间系）。回放时若节点仍用系统壁钟，时间对不上会导致 Nav2 代价地图查 TF 失败、点云全部被丢弃。因此：

- **不录 `/clock`**，回放时用 `ros2 bag play --clock` 合成发布（频率 40 Hz）。合成时钟与回放消息的时间戳天然一致，且避免与录制的 `/clock` 形成双发布者冲突。
- 回放时所有节点以 `use_sim_time:=true` 启动（`replay_tf.launch.py`、`waypoint_nav.launch.py`、`world_nav2.launch.py`；`nav_waypoint_node` 已由 launch 注入 `use_sim_time`）。
- 若确需离线分析 `/clock`，可单独录制，但回放驱动导航栈时必须用 `--topics` 将其排除。

### 开环限制（必读）

回放是**开环**的：录制的传感器与里程计数据不会响应回放期间产生的 `/cmd_vel`，"机器人"不会按新指令改变轨迹；且 `/world_odom` 的 twist 恒为 0。因此这种方式：

- ✅ 可验证：数据流可通（里程计→TF→代价地图→规划→控制逐级出数）、节点不崩溃、任务服务接受任务并驱动状态机、控制输出非零且方向合理、回放可重复（同一 bag 结果确定，适合回归）。
- ❌ 不可验证：闭环导航能否真正到达目标。若下发的航点与录制轨迹不匹配（位姿已越过目标、控制器不支持倒车），会原地转向直至超时 `FAILED`——这是开环失配的预期表现，不是链路故障。`COMPLETED` 只能出现在下发与录制轨迹匹配的任务时。

## 2. 前置条件

- 已按 QUICK_START.md 构建工作空间（含 `nav_replay_support`）；每个终端先执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

- **所有终端必须使用同一个 `ROS_DOMAIN_ID`**（含 `ros2 service call` 所在终端）。本机 `~/.bashrc` 当前默认导出 `41`；若只在部分终端 `export` 成其他值（如旧文档写的 42），就会出现"服务列表能看到、调用永远卡死"的现象——列表走 daemon 所在域，调用走当前终端的域。跨机/域隔离背景见 doc/ros_domain_clock_conflict.md。
- 完整仿真（方案 C）可正常运行。

## 3. 录制

### 3.1 录制话题清单

| 话题 | 类型 | 频率 | 回放时的作用 |
|---|---|---|---|
| `/world_odom` | nav_msgs/Odometry | 10 Hz | 里程计 + `odom_to_tf_node` 重建 `world→base_link` TF |
| `/livox/lidar` | sensor_msgs/PointCloud2 | 10 Hz（约 2 万点/帧） | 代价地图 + 导航节点感知 |
| `/cmd_vel` | geometry_msgs/Twist | 10 Hz | 仅作对照：录制时原次控制输出 |
| `/nav_status` | std_msgs/String | 事件驱动 | 仅作对照：原次状态流转时序 |

- `/cmd_vel`、`/nav_status` 不参与回放驱动（回放时由导航节点现场发布），只用于离线对照。
- **不录** `/gps/data`、`/livox/imu`：回放不再需要融合输入（录制时 `gps_imu_fusion` 正常运行以产生 `/world_odom`）。
- **不录** `/tf`、`/tf_static`：回放由 `odom_to_tf_node` 与静态发布器重建，录制会导致重复发布者。
- **不录** `/clock`：见 §1 时钟策略。

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
  /world_odom /livox/lidar /cmd_vel /nav_status
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

确认 `/world_odom`（≈10 Hz）与 `/livox/lidar`（≈10 Hz）均有消息。

> 注意：`/world_odom` 是融合后的派生数据，修改融合参数/坐标原点后需重新录制。
> 早期不含 `/world_odom` 的 bag（如 sim_bag_20261012_194610）不适用于本流程。

## 4. 回放（仅雷达 + 里程计，推荐）

### 4.1 回放前检查

**检查 1：确认仿真已完全关闭**，无残留 `/clock` 发布者、无残留 `gps_imu_fusion`/Gazebo/`robot_state_publisher`（残留会造成双 `/clock`、双 TF 发布者）：

```bash
ros2 topic info /clock        # Publisher count 必须为 0
pkill -f gzserver; pkill -f gzclient   # 如有残留
```

**检查 2：确认各终端 `ROS_DOMAIN_ID` 一致**：在每个终端执行 `echo $ROS_DOMAIN_ID`，值必须相同（本机 .bashrc 默认 41）。不一致时服务可见但调用卡死（见 §6.9）。

**检查 3：确认服务端真的在线再调用**：`ros2 node list | grep execute_task_node` 必须能看到节点。若节点已退出而 `ros2 service list` 仍显示该服务，是 daemon 的陈旧缓存——先 `ros2 daemon stop` 清理，再核对节点存活，否则调用会永远等不到响应。

### 4.2 操作步骤

1. **启动里程计 → TF**（终端 1，先起 TF 源，确保第一帧点云到达前 TF 已就绪）：

```bash
ros2 launch nav_replay_support replay_tf.launch.py use_sim_time:=true
```

> 该 launch 启动 `odom_to_tf_node`（按消息时间戳广播 TF `world→base_link`）与静态 TF `base_link→livox`（默认 `z=0.1`，与 Go2 xacro 外参一致）。不启动 bag play 与导航栈。

2. **回放数据源**（终端 2）：

```bash
ros2 bag play sim_bag_20XXXXXX_XXXXXX \
  --clock 40 \
  --topics /world_odom /livox/lidar
```

> `--clock 40`：以 40 Hz 合成发布 `/clock`；`--topics` 只回放雷达与里程计——`/cmd_vel`、`/nav_status` 必须排除，否则与导航节点现场发布的同名话题冲突；bag 若含 `/tf`、`/tf_static` 也必须排除。

3. **启动导航**（终端 3，`use_sim_time` 必须为 true；**保持 `simulation` 默认 true，不要传 false**）：

```bash
# 方案 A：旧栈 waypoint_nav
ros2 launch waypoint_nav waypoint_nav.launch.py use_sim_time:=true

# 方案 B：新栈 robotdog_nav2_bringup（与方案 A 互斥，两栈都会占用 /controller_server、/waypoint_goals、/cmd_vel、/nav_status）
ros2 launch robotdog_nav2_bringup world_nav2.launch.py use_sim_time:=true rviz:=false
```

**步骤 4：启动任务服务**（终端 4）：

```bash
ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1
```

**步骤 5：尽快下发任务**（终端 5，趁回放位姿尚在录制起点附近）。回放包不含 `/gps/data`，只能用局部坐标任务：

```bash
ros2 service call /agent/uav1/execute_task execute_task_server/srv/ExecuteTask "{
  request_json: '{\"mission_id\":\"bag_replay\",\"agent_id\":\"uav1\",\"capability_id\":\"goto_waypoints\",\"params\":{\"waypoint_frame\":\"local\",\"waypoints\":[{\"x\":1.0,\"y\":0.0,\"z\":0.0},{\"x\":2.0,\"y\":1.0,\"z\":0.0},{\"x\":0.0,\"y\":2.0,\"z\":0.0}]}}'
}"
```

**步骤 6：观察执行**：

```bash
ros2 topic echo /nav_status          # 状态流转
ros2 topic echo /cmd_vel             # 控制输出（对照录制时的原次输出）
```

## 5. 验收点

| 检查项 | 命令/方式 | 通过标准 |
|---|---|---|
| 时钟 | `ros2 topic hz /clock` | ≈40 Hz，且只有一个发布者 |
| 里程计回放 | `ros2 topic hz /world_odom` | ≈10 Hz |
| 里程计 → TF | `ros2 run tf2_ros tf2_echo world base_link` | 位姿随回放轨迹连续变化，无未来外推/超时错误 |
| 静态外参 | `ros2 run tf2_ros tf2_echo base_link livox` | 恒为平移 `(0, 0, 0.1)` |
| TF 发布者唯一 | `ros2 topic info /tf --verbose`、`/tf_static --verbose` | `/tf` 仅 `odom_to_tf_node`；`/tf_static` 仅回放 launch |
| 代价地图 | 导航日志 / RViz | lifecycle `active`；首帧 TF 后无 `Timed out waiting for transform`；点云进入代价地图 |
| 状态流转 | `ros2 topic echo /nav_status` | 任务接受后进入 `EXECUTING_PATH`；轨迹匹配时可见 `GOAL_REACHED`/`COMPLETED` |
| 控制输出 | `ros2 topic echo /cmd_vel` | 出现非零控制量；开环轨迹不匹配时表现为原地转向（`linear.x≈0, angular.z≠0`），属预期 |
| 无融合/Gazebo | `ros2 node list` | 无 `gps_imu_fusion_node`、`robot_state_publisher`、`gzserver` |

## 6. 限制与常见问题

### 6.1 任务下发时机（回放 FAILED 的最常见原因）

回放位姿是"已走过的轨迹"。录制时机器人用约 T 秒走完任务；回放时若下发过晚，位姿已越过目标点，而直线规划与纯跟踪控制器不支持倒车（`allow_backward: false`），机器人会原地绕圈直到超时 FAILED。**务必在 bag 起播后立刻下发与录制时相同的任务。**

### 6.2 重复回放同一 bag 会触发 `TF_OLD_DATA` / `Detected jump back in time`

每次 `ros2 bag play` 都把仿真时间拉回 bag 起点。不重启消费节点直接重放，tf2 检测到时间回跳会清空缓冲并忽略"过去"的数据，控制器持续 abort。**重放前必须重启导航栈（最好连 `replay_tf.launch.py` 一起重启）。**

### 6.3 bag 播放结束后仿真时间冻结

bag 播完 `/clock` 停止，Nav2 定时器与控制器可能报超时，状态机停留在最后状态。播放结束即代表本轮验证结束，及时停止导航栈。

### 6.4 RViz 中 markers / path 不显示（已修复）

`nav_waypoint_node` 原先未注入 `use_sim_time`，发布的 markers/path 带壁钟时间戳，RViz 切到仿真时间后视为"来自未来"而挂起。**已在 `waypoint_nav.launch.py` 中注入 `use_sim_time` 修复**；若使用旧版本工作区遇到此现象，先更新该 launch。

### 6.5 QoS 不兼容导致收不到数据

Gazebo 插件的发布 QoS 会被 `ros2 bag record` 自动记录并在回放时复现，一般无需处理。若回放时订阅端收不到数据，用 QoS 覆盖文件强制 best_effort：

```bash
ros2 bag play <bag> --qos-profile-overrides-path qos_override.yaml
```

### 6.6 回放倍速

`--rate 0.5` 慢放、`--rate 2.0` 倍速。注意倍速会压缩 costmap `transform_tolerance` 的容忍窗口（10 Hz 里程计 TF 的最大间隔约 0.1 s，`waypoint_nav` 已调至 0.3），倍速下可能偶发 TF 外推超时；对照分析建议保持 1.0。

### 6.7 服务调用报 "The passed service type is invalid"

原因：该终端没有 source 工作空间。`execute_task_server/srv/ExecuteTask` 只注册在 `install/setup.bash` 中，仅 source `/opt/ros/humble` 时找不到该类型。先在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

验证环境就绪：`ros2 interface show execute_task_server/srv/ExecuteTask` 能打印出 `request_json` 定义即可。

### 6.8 同一 bag 的重复回放是确定性的

相同 bag + 相同任务 + 相同配置，每次回放的 TF、`/cmd_vel` 输出一致，适合做算法改动的回归对照；这也意味着 bag 不能替代仿真做闭环调参。

### 6.9 `ros2 service call` 卡死不返回

按概率排查：

1. **服务端已退出**：`ros2 node list | grep execute_task_node` 看不到节点即为此因。服务已死但 `ros2 service list` 仍显示，是 daemon 陈旧缓存（`ros2 daemon stop` 清理后重查）。对死服务调用会永远等不到响应。
2. **终端间 `ROS_DOMAIN_ID` 不一致**（服务列表能看到、调用卡死的最典型原因）：列表查询走 daemon 所在域，`service call` 的客户端节点在当前终端的域。每个终端 `echo $ROS_DOMAIN_ID` 核对，本机 .bashrc 默认 41。
3. **客户端终端没 source 工作空间**：会报 "The passed service type is invalid"（见 §6.7），不会卡死；一并记录以便区分。

## 7. 附录：旧流程（回放 GPS/IMU + 运行 gps_imu_fusion）

早期流程回放融合输入、由 `gps_imu_fusion` 现场重算 `/world_odom` 与 TF，适用于验证融合算法本身：

```bash
# 录制（终端 4）：
ros2 bag record -o sim_bag_$(date +%Y%m%d_%H%M%S) \
  /gps/data /livox/imu /livox/lidar /cmd_vel /nav_status

# 回放（终端 2）：
ros2 bag play sim_bag_20XXXXXX_XXXXXX --clock 40 \
  --topics /gps/data /livox/imu /livox/lidar

# 定位 + 导航（终端 1、3）：
ros2 launch gps_imu_fusion gps_imu_fusion.launch.py use_sim_time:=true &
ros2 launch waypoint_nav waypoint_nav.launch.py use_sim_time:=true
```

与推荐流程的差异：需要录制/回放两个额外的传感器话题并多启动一个节点；其余时钟策略、开环限制、下发时机与 §4–§6 相同。
