# 设计：融合定位与导航栈解耦（rosbag 仅回放雷达 + 里程计）

- 日期：2026-10-20
- 状态：已实现（2026-10-20）
- 范围：新增 `nav_replay_support` 功能包，把任意 `nav_msgs/Odometry` 转成 TF，并在回放时提供 `base_link→livox` 静态外参；调整回放流程与文档，使 `waypoint_nav` 与 `robotdog_nav2_bringup` 两套导航栈都能只靠 `/livox/lidar` + `/world_odom` 的回放包运行，无需录制/回放 GPS、IMU 原始数据
- 关联：`doc/rosbag_record_replay.md`（现有回放手册）、`doc/devel/2026-10-20-design-nav2-world-stack.md`（新 Nav2 栈）

## 1. 背景与目标

当前定位与导航的链路为：

```text
Gazebo ─┬─ /gps/data ─┐
        └─ /livox/imu ─┴─ gps_imu_fusion ─┬─ /world_odom (nav_msgs/Odometry)
                                           └─ TF world→base_link
Gazebo ─── /livox/lidar ───────────────────→ Nav2 costmap / nav_waypoint_node

nav_msgs/Odometry 链路：
  /world_odom → controller_server（twist）
              → nav_waypoint_node（位姿缓存）
              → multi_goal_bridge_node（位姿缓存）
TF 链路：
  world→base_link → Costmap2DROS::getRobotPose()
                  → 点云 /livox/lidar 从 livox 变换到 world
```

问题在于：`gps_imu_fusion_node` 是**唯一**的 `world→base_link` TF 发布者，且它必须收到 `/gps/data` 或 `/livox/imu` 才开始发布。回放一个只含雷达与里程计的 bag 时：

- `/world_odom`、`/livox/lidar` 有数据；
- 没有 TF `world→base_link`，Nav2 costmap 无法激活、无法查询机器人位姿；
- 没有 `base_link→livox` 静态 TF（回放时没有 `robot_state_publisher`），点云无法变换到 `world`。

本设计目标：

1. 导航层只依赖稳定契约：里程计话题 `/world_odom` + TF `world→base_link` + TF `base_link→livox`，不感知定位数据从何而来。
2. 新增一个与融合层无关的通用 `odom → TF` 适配节点；回放时由它从 bag 中的 `/world_odom` 重建 TF。
3. 回放时**不启动** `gps_imu_fusion`，不录制 `/gps/data`、`/livox/imu`。
4. 同时支持 `waypoint_nav` 与 `robotdog_nav2_bringup` 两套导航栈。
5. 保持开环验证语义：验证数据流、TF、代价地图、多点任务状态机与控制输出；不承诺回放时机器人真正按新指令到达航点。

## 2. 现状与耦合点

### 2.1 融合层

`gps_imu_fusion_node`（`src/robotdog_nav/gps_imu_fusion/src/gps_imu_fusion_node.cpp`）：

- 订阅 `/gps/data`、`/imu`（launch 重映射为 `/livox/imu`）；
- 10 Hz 定时器发布 `/world_odom`（`frame_id=world`、`child_frame_id=base_link`）；
- 同一回调中广播 TF `world→base_link`，时间戳取最新 IMU 时间；
- 没有关闭 TF、替换里程计源或仅转发的参数。

### 2.2 导航层实际依赖

- `nav2_costmap_2d::Costmap2DROS`：激活前等待 `world→base_link`；`getRobotPose()` 与点云变换都走 TF。
- `controller_server`、`bt_navigator`、`velocity_smoother`、`multi_goal_bridge_node`：订阅 `/world_odom` 取 twist/位姿缓存。
- `waypoint_nav/nav_waypoint_node`：直接订阅 `/world_odom`（`nav_core.cpp:79-82`），不查 TF。
- `execute_task_server`：不消费位姿，只把 JSON 转成 `/waypoint_goals`。
- `multi_goal_bridge_node` 的 `/gps/data` 原点一致性检查只在 GPS 航点场景使用；局部坐标任务在无 `/gps/data` 时可正常下发。

结论：导航层与融合层**已经**只通过话题 + TF 通信，耦合点仅剩「TF 由谁发布」。因此解耦不需要改导航算法，只需在回放时提供一个等价的 TF 源。

### 2.3 现有 bag 与文档

- `sim_bag_20261013_143335/`：含 `/livox/lidar`、`/world_odom`（1099 条 / 88.74 s，约 10 Hz）、`/gps/data`、`/livox/imu`、`/cmd_vel`、`/nav_status`；**不含** `/tf`、`/tf_static`、`/clock`。
- 对该 bag 的实测：里程计消息间隔中位数 80 ms、p95 95 ms、最大 100 ms；每帧雷达时间戳到下一帧里程计 TF 的等待时间 p95 94 ms、最大 99 ms。
- `sim_bag_20261012_194610/`：不含 `/world_odom`，不适用于本方案，需要重新录制。
- `doc/rosbag_record_replay.md` 现有流程仍要求回放 `/gps/data` + `/livox/imu` 并运行 `gps_imu_fusion`，与本设计目标不同，需要新增/改写回放章节。

## 3. 设计原则

- **契约不变**：继续使用 `/world_odom` 与 `world`/`base_link`/`livox` 坐标系，不改两套导航栈的接口与参数键。
- **单一职责**：新增节点只做「里程计消息 → TF」，不融合、不改写里程计、不发布 `/world_odom`。
- **不重复发布**：回放时不启动 `gps_imu_fusion`、Gazebo、`robot_state_publisher`；同一时刻只有一个 `world→base_link` 动态发布者和一个 `base_link→livox` 静态发布者。
- **配置优先**：话题名、坐标系覆盖、静态外参均走 `config/params.yaml` 与 launch 参数。
- **边界校验**：只在订阅回调/参数读取处校验空 frame、零时间戳、非法四元数与非法参数；内部不做重复防御。

## 4. 方案总览

```text
ros2 bag play --clock 40 --topics /livox/lidar /world_odom
  ├─ /world_odom ──→ nav_replay_support/odom_to_tf_node ──→ TF world→base_link
  └─ /livox/lidar ────────────────────────────────────────→ Nav2 costmap

nav_replay_support/replay_tf.launch.py
  └─ static TF base_link→livox ──────────────────────────→ 点云坐标变换

execute_task_server → /waypoint_goals → 导航栈 → /cmd_vel
```

- `/livox/lidar`：由 bag 直接回放，供 costmap 障碍层使用。
- `/world_odom`：由 bag 直接回放，供 `controller_server`/`nav_waypoint_node`/`multi_goal_bridge_node` 使用；同时由 `odom_to_tf_node` 转成 TF。
- `base_link→livox`：由 `replay_tf.launch.py` 发布（默认 `z=0.10`，与 Go2 xacro 中 `trunk→livox` 的外参一致）。回放 launch 负责模拟 `robot_state_publisher` 的传感器外参。
- `/clock`：bag 不录制，由 `ros2 bag play --clock 40` 合成；所有节点 `use_sim_time:=true`。

## 5. 新增 `nav_replay_support` 功能包

### 5.1 目录结构

```text
src/robotdog_nav/nav_replay_support/
├── CMakeLists.txt                 # ament_cmake，C++14
├── package.xml
├── config/
│   └── params.yaml                # odom_topic、frame 覆盖
├── launch/
│   ├── odom_to_tf.launch.py       # 只启动适配节点
│   └── replay_tf.launch.py        # 适配节点 + 静态 base_link→livox
├── src/
│   └── odom_to_tf_node.cpp        # 单一职责：Odometry → TF
└── README.md                      # 用途与回放示例
```

选择新建包而非塞进 `gps_imu_fusion` 或 `robotdog_nav2_bringup` 的原因：

- `gps_imu_fusion` 的职责是 GPS+IMU 融合定位；`odom_to_tf` 是回放/通用适配工具，放进融合包会混淆层次，且会迫使回放用户启动一个「其他节点被禁止运行」的包。
- 两套导航栈都可直接依赖该包，不引入对另一套导航栈的依赖。
- 符合「新增功能优先新建文件/包、一个文件一个职责」。

### 5.2 节点接口

节点名：`odom_to_tf_node`。

| 方向 | 名称 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | 参数 `odom_topic`（默认 `/world_odom`） | `nav_msgs/msg/Odometry` | QoS：`KeepLast(10)`，reliable、volatile |
| 发布 | `/tf` | `tf2_msgs/msg/TFMessage`（经 `tf2_ros::TransformBroadcaster`） | 每条里程计消息广播一次 |
| 参数 | `parent_frame` | string，默认 `""` | 非空时覆盖 `header.frame_id` |
| 参数 | `child_frame` | string，默认 `""` | 非空时覆盖 `child_frame_id` |
| 参数 | `use_sim_time` | bool | 标准 ROS 参数，回放时为 `true` |

QoS 保持简单的 reliable/keep-last(10)，与融合发布端及现有 bag 一致；若未来接入 best-effort 里程计源，用 `ros2 bag play --qos-profile-overrides-path` 或后续再增加参数，不在本次预置。

### 5.3 回调行为与边界校验

```text
odomCallback(msg):
  parent = parent_frame 非空 ? parent_frame : msg.header.frame_id
  child  = child_frame  非空 ? child_frame  : msg.child_frame_id
  若 parent 或 child 为空：throttle WARN，丢弃
  若 parent == child：throttle WARN，丢弃
  若 header.stamp 为 0：throttle WARN，丢弃
  若 position 非有限值，或四元数含 NaN/Inf：throttle WARN，丢弃
  四元数范数 < 1e-6：throttle WARN，丢弃
  归一化四元数
  t.header.stamp = msg.header.stamp          // 精确使用消息时间戳，不做前移/偏移
  t.header.frame_id = parent
  t.child_frame_id  = child
  t.transform.translation = msg.pose.pose.position
  t.transform.rotation    = 归一化后的四元数
  broadcaster.sendTransform(t)
```

关键点：

- **使用消息自身时间戳**，不用 `this->now()`。bag 回放时消息戳是仿真时间，用壁钟会导致 TF 与点云时间对不上。
- **不做时间戳前移（future-dating）**。前移会让位姿超前于里程计数据，且不能消除对下一帧里程计的依赖；正确做法是把 costmap 的 `transform_tolerance` 调到覆盖 10 Hz 里程计的间隔（见 §6.3）。
- 事件驱动，按 odom 频率（约 10 Hz）发布，不额外开定时器。
- 帧名默认以消息为准，参数只做覆盖，便于复用非 `/world_odom` 的 bag。
- 启动时打印 `topic`、帧覆盖配置、QoS；异常消息用 `RCLCPP_WARN_THROTTLE` 并累计丢弃计数，避免刷屏。

### 5.4 launch 文件

**`odom_to_tf.launch.py`**（通用、仅节点）

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_sim_time` | `false` | 通用场景默认壁钟；回放时由 `replay_tf.launch.py` 传 `true` |
| `odom_topic` | `/world_odom` | 覆盖 YAML 中的话题名 |
| `parent_frame` | `""` | 空表示使用消息 frame |
| `child_frame` | `""` | 空表示使用消息 child frame |
| `params_file` | 包内 `config/params.yaml` | |

**`replay_tf.launch.py`**（回放专用）

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_sim_time` | `true` | 回放默认仿真时间 |
| `odom_topic` | `/world_odom` | |
| `parent_frame` / `child_frame` | `""` | |
| `publish_static_livox_tf` | `true` | 是否发布 `base_link→livox` |
| `livox_x/y/z` | `0.0 / 0.0 / 0.1` | 与 Go2 xacro 外参一致 |
| `livox_roll/pitch/yaw` | `0.0` | |

- 包含 `odom_to_tf.launch.py`，并透传参数。
- `publish_static_livox_tf:=true` 时启动 `tf2_ros/static_transform_publisher`，`--frame-id base_link --child-frame-id livox`。
- **不**启动 `ros2 bag play`、不启动导航栈、不启动任务服务，保持分终端、可替换 bag 的形态。

`config/params.yaml`：

```yaml
odom_to_tf_node:
  ros__parameters:
    odom_topic: "/world_odom"
    parent_frame: ""
    child_frame: ""
```

## 6. 导航侧改动（最小）

### 6.1 静态 TF `base_link→livox` 的归属

回放时没有 `robot_state_publisher`，必须显式发布 `base_link→livox`。本设计由 **`replay_tf.launch.py`** 统一负责：

- 回放时导航 launch 保持默认 `simulation:=true`，因此不会重复发布静态 TF；
- **不要**在回放时给导航 launch 传 `simulation:=false`，否则会出现两个相同的静态 TF 发布者；
- 导航 launch 的默认值不改，`simulation:=true` 对正常 Gazebo 运行仍然正确。

已核对 Go2 xacro：`base_link→trunk` 为 `0 0 0`，`trunk→livox` 为 `0 0 0.10`，与 `replay_tf.launch.py` 默认外参一致。Go2W/实机若安装位置不同，通过 launch 参数覆盖（外参标定不在本次范围）。

### 6.2 `use_sim_time` 注入修复

`waypoint_nav.launch.py` 中 `nav_waypoint_node` 目前没有注入 `use_sim_time`（`doc/rosbag_record_replay.md` §6.2 已记录），回放时节点自身时间与仿真钟不一致，markers/path 时间戳会异常。本设计在 `waypoint_nav_node` 的 `parameters` 中追加：

```python
{'use_sim_time': use_sim_time}
```

该改动不影响正常仿真（`use_sim_time` 原本就应为 true），属于回放兼容性修复。

### 6.3 costmap TF 容忍度

- `robotdog_nav2_bringup/config/common.yaml`：local 0.3 / global 0.5，已覆盖 10 Hz odom 周期。
- `waypoint_nav/config/params.yaml`：`local_costmap.transform_tolerance` 当前为 `0.1`。实测雷达时间戳到下一帧里程计 TF 的最大等待为 99 ms，`0.1` 在回放抖动下过紧，改为 **`0.3`**。
- `transform_tolerance` 是容忍度参数，不改变规划/控制行为；对正常仿真（TF 由 160 Hz IMU 时间戳驱动）没有副作用。

### 6.4 不需要的改动

- 不修改 `gps_imu_fusion_node`：仿真/实机仍由它发布 `/world_odom` + TF；回放只是替换了 TF 的实现来源。
- 不修改 `execute_task_server`、`nav_waypoint_node`、`multi_goal_bridge_node` 的接口。
- 不新增 `odom→base_link` 或 `map` 坐标系。

## 7. 录制流程（生成仅含雷达 + 里程计的 bag）

1. 按现有方式启动完整仿真 + `gps_imu_fusion` + 导航 + 任务服务（录制时融合仍正常运行，用于产生 `/world_odom`）。
2. 录制：

```bash
ros2 bag record -o sim_bag_$(date +%Y%m%d_%H%M%S) \
  /livox/lidar /world_odom \
  /cmd_vel /nav_status
```

| 话题 | 回放作用 | 是否必需 |
| --- | --- | --- |
| `/livox/lidar` | 代价地图输入 | 必需 |
| `/world_odom` | 里程计 + TF 来源 | 必需 |
| `/cmd_vel` | 录制时控制输出对照 | 可选 |
| `/nav_status` | 录制时状态流转对照 | 可选 |

- 不录 `/gps/data`、`/livox/imu`：回放不再需要融合输入。
- 不录 `/tf`、`/tf_static`：回放由适配节点与静态发布器重建，录制会导致重复发布者。
- 不录 `/clock`：回放用 `--clock 40` 合成，避免双 `/clock` 发布者。
- `/cmd_vel`、`/nav_status` 仅作离线对照，**回放时用 `--topics` 排除**。

3. 下发与录制时相同的局部坐标任务（见 §8.4）。

> 注意：`/world_odom` 是融合后的派生数据。若后续修改融合参数或坐标原点，需要重新录制 bag；旧 bag 仍可回放，但轨迹基准不变。

## 8. 回放流程

### 8.1 前置检查

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42          # 与录制时一致

ros2 topic info /clock           # Publisher count 必须为 0
pkill -f gzserver; pkill -f gzclient   # 清除残留 Gazebo
```

回放前确认没有残留的 `gps_imu_fusion`、Gazebo、`robot_state_publisher`，避免重复 `/world_odom` 或 TF 发布者。

### 8.2 启动回放与 TF（终端 1、2）

```bash
# 终端 1：只回放雷达与里程计，合成时钟
ros2 bag play sim_bag_YYYYMMDD_HHMMSS \
  --clock 40 \
  --topics /livox/lidar /world_odom

# 终端 2：里程计 → TF + 静态 base_link→livox
ros2 launch nav_replay_support replay_tf.launch.py use_sim_time:=true
```

启动顺序建议先起终端 2，再起终端 1，确保第一帧点云到达前 TF 已就绪；反过来也可，适配节点会持续接收后续里程计。

### 8.3 启动导航与任务服务（终端 3、4）

**方案 A：`waypoint_nav`（旧栈）**

```bash
# 终端 3：保持 simulation 默认 true，不要传 false
ros2 launch waypoint_nav waypoint_nav.launch.py use_sim_time:=true

# 终端 4
ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1
```

**方案 B：`robotdog_nav2_bringup`（新 Nav2 栈）**

```bash
# 终端 3：保持 simulation 默认 true，不要传 false
ros2 launch robotdog_nav2_bringup world_nav2.launch.py \
  use_sim_time:=true rviz:=false

# 终端 4
ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1
```

> 两套栈不能同时启动：都会占用 `/controller_server`、`/waypoint_goals`、`/cmd_vel`、`/nav_status`。
> 不要传 `simulation:=false`：静态 `base_link→livox` 已由 `replay_tf.launch.py` 发布。

### 8.4 下发任务（终端 5）

使用局部坐标，避免回放包没有 `/gps/data` 时依赖 GPS 原点：

```bash
ros2 service call /agent/uav1/execute_task execute_task_server/srv/ExecuteTask "{
  request_json: '{\"mission_id\":\"bag_replay\",\"agent_id\":\"uav1\",\"capability_id\":\"goto_waypoints\",\"params\":{\"waypoint_frame\":\"local\",\"waypoints\":[{\"x\":1.0,\"y\":0.0,\"z\":0.0},{\"x\":2.0,\"y\":1.0,\"z\":0.0},{\"x\":0.0,\"y\":2.0,\"z\":0.0}]}}'
}"
```

回放是开环的：bag 中的 `/world_odom` 是录制时的固定轨迹，不会响应回放期间的新 `/cmd_vel`。**必须在 bag 起播后尽快下发与录制时相同的任务**；下发过晚或任务不同，位姿已越过目标点，直线控制器又不支持倒车，会超时 `FAILED`。因此 `COMPLETED` 只能在与录制轨迹匹配时出现，不能作为保证的验收条件。

## 9. 验收

| 检查项 | 命令/方式 | 通过标准 |
| --- | --- | --- |
| 时钟 | `ros2 topic hz /clock` | ≈40 Hz，且只有一个发布者 |
| 里程计回放 | `ros2 topic hz /world_odom` | ≈10 Hz，消息有 `world`/`base_link` 帧 |
| 里程计 → TF | `ros2 run tf2_ros tf2_echo world base_link` | 位姿随回放轨迹连续变化，无未来外推/超时错误 |
| 静态外参 | `ros2 run tf2_ros tf2_echo base_link livox` | 平移 `(0,0,0.1)`，无查找失败 |
| TF 发布者唯一 | `ros2 topic info /tf --verbose`、`/tf_static --verbose` | `/tf` 仅 `odom_to_tf_node`；`/tf_static` 仅回放 launch |
| costmap 激活 | 导航日志 / RViz | lifecycle `active`；无 `Timed out waiting for transform`；点云进入代价地图 |
| 状态流转 | `ros2 topic echo /nav_status` | 进入 `EXECUTING_PATH`，按任务推进；与录制任务匹配时可见 `GOAL_REACHED`/`COMPLETED` |
| 控制输出 | `ros2 topic echo /cmd_vel --field linear` | 出现非零控制量，量级/方向与录制时对照合理 |
| 无融合/Gazebo | `ros2 node list` | 无 `gps_imu_fusion_node`、无 `robot_state_publisher`、无 `gzserver` |

快速自检：

```bash
ros2 run tf2_ros tf2_echo world base_link
ros2 run tf2_ros tf2_echo base_link livox
ros2 topic hz /livox/lidar
ros2 lifecycle get /controller_server        # active
ros2 param get /local_costmap transform_tolerance   # 0.3（waypoint_nav）
```

## 10. 风险与限制

1. **开环**：固定轨迹，不能验证闭环到达；闭环仍需 Gazebo/实机。`COMPLETED` 不保证出现。
2. **TF 时序**：10 Hz odom 与 10 Hz 雷达存在 0~100 ms 相位差。`transform_tolerance` 必须 ≥ 0.3；回放 `--rate` 保持 1.0，倍速会压缩容忍窗口。
3. **重复发布**：回放时启动 `gps_imu_fusion`/Gazebo/`robot_state_publisher`，或给导航 launch 传 `simulation:=false`，都会产生重复 `/world_odom` 或重复 TF；同一域内只能有一个 `odom_to_tf_node`。
4. **bag 内容**：若 bag 自带 `/tf`、`/tf_static`，回放时不要包含它们；`--topics` 必须显式列出。
5. **时钟**：必须恰好一个 `/clock` 发布者；残留 Gazebo 或录制的 `/clock` 都会破坏仿真时间。bag 播放结束后仿真时间停止，Nav2 定时器可能报超时，应及时停止导航栈。
6. **QoS**：`/world_odom` 在现有 bag 中为 reliable/volatile，默认可靠订阅可兼容；若订阅不到，用 `--qos-profile-overrides-path` 覆盖。
7. **twist 为零**：`/world_odom` 的 twist 恒为 0，新栈 `velocity_smoother` 已用 `OPEN_LOOP`；不要据此判断真实速度。
8. **GPS 航点**：回放包不含 `/gps/data`，只能用局部坐标任务（`waypoint_frame=local`）。
9. **启动顺序**：先起回放数据源并确认 TF，再起导航，否则 costmap 在 `on_activate` 阶段等待 TF，launch 看起来像卡住。
10. **静态外参**：`base_link→livox` 的 `z=0.1` 对 Go2 成立；Go2W/实机若安装位置不同，需用 launch 参数覆盖或实测标定。
11. **派生数据**：`/world_odom` 是融合产物，bag 的可复用性受融合参数/原点影响。

## 11. 实施步骤

- 在 git worktree（如 `.worktree/nav-replay-support`）中实施，完成后由用户决定是否合并。
- 新建 `src/robotdog_nav/nav_replay_support/`：`CMakeLists.txt`、`package.xml`、`src/odom_to_tf_node.cpp`、`config/params.yaml`、`launch/odom_to_tf.launch.py`、`launch/replay_tf.launch.py`、`README.md`。
- `waypoint_nav.launch.py` 给 `nav_waypoint_node` 注入 `use_sim_time`；`waypoint_nav/config/params.yaml` 的 `local_costmap.transform_tolerance` 调整为 `0.3`。
- 构建：

```bash
colcon build --packages-select nav_replay_support waypoint_nav --symlink-install
source install/setup.bash
```

- 用现有 `sim_bag_20261013_143335` 回放验证 TF、静态外参、costmap。
- 分别验证 `waypoint_nav` 与 `robotdog_nav2_bringup` 的任务下发与 `/nav_status`、`/cmd_vel`。
- 更新 `doc/rosbag_record_replay.md`：新增「仅雷达 + 里程计回放」章节，保留原有 GPS/IMU 回放流程作为对照；更新录制话题表与常见问题。
- 更新 `AGENTS.md` 的目录与解耦说明，加入 `nav_replay_support`；`README.md`/`QUICK_START.md` 只加回放文档的一行指针。

## 12. 不在本设计范围

- 一条命令全包的回放 launch（`ros2 bag play` + TF + 导航 + 任务服务）；本次按用户选择保持分终端。
- 闭环回放（bag 数据 + 运动学/仿真回灌，使机器人响应新 `/cmd_vel`）。
- GPS 航点在无 `/gps/data` 回放包中的支持（需要额外录制原点或提供原点参数）。
- Go2W/实机 livox 外参标定。
- 将 `gps_imu_fusion` 重构为「融合核心 + TF 发布器」的长期架构调整。

## 13. 验证记录

2026-10-20 在 worktree 内完成，全程未启动 Gazebo 与 `gps_imu_fusion`，数据源为现有
`sim_bag_20261013_143335`（含 `/livox/lidar` + `/world_odom`）：

- 构建：`colcon build --packages-select msg_set_msgs nav_replay_support waypoint_nav --symlink-install` 通过。
- `odom_to_tf_node`：启动参数打印正常；首帧里程计到达后输出 `First TF broadcast: world -> base_link`；`/tf` 全程仅 1 个发布者。
- 静态外参：`tf2_echo base_link livox` 恒为平移 `(0, 0, 0.1)`；`/tf_static` 仅 1 个发布者。
- 数据率：`/world_odom` ≈10 Hz、`/livox/lidar` ≈8.1 Hz、`/clock` ≈40 Hz。
- TF 连续性：`tf2_echo world base_link` 位姿随回放轨迹连续变化（位置推进、yaw 连续旋转），无外推错误。
- costmap：`controller_server` lifecycle `active`；`/local_costmap/local_costmap` 的 `transform_tolerance` 读回为 `0.3`；首帧 TF 广播之后 0 条 `Timed out waiting for transform`（激活前的等待日志属正常启动行为）。
- `nav_waypoint_node`：`use_sim_time` 参数读回为 `True`（launch 注入生效）。
- 任务链路：`execute_task_server` 接受 3 点局部坐标任务（`waypoint_count: 3`），FSM 进入并保持 `EXECUTING_PATH`；`controller_server` 经 `/follow_path` 输出非零 `/cmd_vel`（开环轨迹与航点不匹配时为原地转向 `angular.z≈±0.32`，属 §10 预期表现）。
- 反例确认：不重启导航栈重复重放同一 bag 会触发 `Detected jump back in time` / `TF_OLD_DATA`，控制器持续 abort——已写入回放手册 §6.2，重放前须重启导航栈。
- 未验证（需 Gazebo/实机）：闭环到达航点、真实避障、`COMPLETED` 全流程。
