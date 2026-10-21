# 设计：Nav2 world 无图导航栈（robotdog_nav2_bringup）

- 日期：2026-10-20
- 状态：已实现（2026-10-20）
- 范围：新增 `src/robotdog_nav/robotdog_nav2_bringup` 功能包，以标准 Nav2 栈替换 `waypoint_nav` 中自研的 planner/航点执行职责；保留现有 `execute_task_server → /waypoint_goals` 任务链
- 关联计划：本次实现按同日评审的实施方案执行

## 1. 背景与目标

当前链路为：

```text
execute_task_server
  → /waypoint_goals (msg_set_msgs/MultiGoal)
  → waypoint_nav/nav_waypoint_node
       ├─ 自行按直线插值生成 nav_msgs/Path
       ├─ FollowPath action → /follow_path
       │    └─ nav2_controller/controller_server + local_costmap
       └─ use_controller_server=false 时自行发 /cmd_vel
```

现有实现只启动 `controller_server`，没有 `planner_server`、`bt_navigator`、`behavior_server`、`smoother_server`、`waypoint_follower`、`velocity_smoother`；“planner” 实际是直线采样，航点执行也由自研 FSM 完成。

本设计新增独立包，目标：

1. 启动完整、标准的 Nav2 导航栈，所有 server 与插件使用成熟实现。
2. 保留 `/waypoint_goals` 入口，新增桥接节点把 `MultiGoal` 转为 Nav2 `FollowWaypoints`，任务服务与指控端无需改动。
3. controller/planner 插件通过配置切换，至少覆盖 DWB、MPPI、RPP 与 NavFn、SMAC 2D、ThetaStar。
4. 配置分层、接口清晰，后续增加自研 Nav2 插件只需新增 profile 文件。
5. 适配当前无图 GPS 定位：`world` 坐标系、`/world_odom`、TF `world→base_link`，不引入 AMCL/map_server。

## 2. 架构与数据流

```text
execute_task_server
  → /waypoint_goals (MultiGoal)
  → robotdog_nav2_bringup/multi_goal_bridge_node
       ├─ GPS(lat/lon/alt) → ENU(world)（GeographicLib LocalCartesian）
       ├─ FollowWaypoints action → /follow_waypoints
       │    └─ nav2_waypoint_follower
       │         └─ NavigateToPose action → nav2_bt_navigator（默认 BT）
       │              ├─ ComputePathToPose → nav2_planner/planner_server
       │              │    └─ global_costmap（world，障碍层 + 膨胀层，allow_unknown）
       │              └─ FollowPath → nav2_controller/controller_server
       │                   └─ local_costmap（world，/livox/lidar）
       │                        └─ /cmd_vel_nav
       │                             └─ nav2_velocity_smoother → /cmd_vel
       ├─ /speed_limit（按 MultiGoalPoint.vel 限速）
       └─ /nav_status（兼容旧测试）
```

- 定位层不变：`gps_imu_fusion` 发布 `/world_odom`（10 Hz）和 TF `world→base_link`。
- 不启动 `map_server`/`amcl`；不发布 `odom→base_link`，避免与融合 TF 冲突。
- `behavior_server` 的恢复行为（Spin/BackUp/Wait）直接发布 `/cmd_vel`，这是 Nav2 标准行为。
- Go2 的 champ `quadruped_controller_node` 与 Go2W 的 `hybrid_motion_controller` 继续消费 `/cmd_vel`，无需改动。

## 3. 接口设计

### 3.1 桥接节点

节点名：`multi_goal_bridge_node`

| 方向 | 名称 | 类型 |
| --- | --- | --- |
| 订阅 | `/waypoint_goals` | `msg_set_msgs/msg/MultiGoal` |
| 订阅 | `/world_odom` | `nav_msgs/msg/Odometry` |
| 订阅 | `/gps/data` | `sensor_msgs/msg/NavSatFix` |
| 订阅 | `/mavros/global_position/gp_origin` | `geographic_msgs/msg/GeoPointStamped`（仅 `origin_source:=topic`） |
| Action client | `/follow_waypoints` | `nav2_msgs/action/FollowWaypoints` |
| 发布 | `/nav_status` | `std_msgs/msg/String` |
| 发布 | `/speed_limit` | `nav2_msgs/msg/SpeedLimit` |
| Service | `/nav/cancel` | `std_srvs/srv/Trigger` |

桥接节点不发布 `/cmd_vel`，速度输出统一由 Nav2 controller/velocity_smoother/behavior_server 负责。

### 3.2 MultiGoal 语义

- `is_gps_aid=true`：`x_or_lat/y_or_lon` 为经纬度，转为 ENU；`false`：直接作为 `world` 局部坐标。
- `is_gps_hgt=true`：`z_or_alt` 为 GPS 高度；`false`：`z_or_alt` 为局部高度。Nav2 2D 控制器实际忽略 z。
- `yaw_mode=path`（默认）：中间点朝向下一点，最后点朝向前一段方向；单点朝当前 odom→目标方向。`yaw_mode=msg` 时直接使用 `MultiGoalPoint.yaw`。
- `vel`：通过 `nav2_msgs/msg/SpeedLimit`（`percentage=false`）限制当前航点速度，默认夹在 `[0.1, 0.6] m/s`；完成/失败/取消后发布 `0.0`（恢复控制器默认限速，不是停车）。
- `ctl_mode=0` 支持；`1` 记录 warning 后按 0 处理（Nav2 输出仍为 yaw rate）；其他值记录 warning 后按 0 处理。

### 3.3 任务状态

桥接节点维护以下状态并发布到 `/nav_status`，字符串与旧 `waypoint_nav/nav_fsm.cpp` 保持一致：

```text
WAITING_FOR_GOALS → INITIALIZING → EXECUTING_PATH
                                   ├─ GOAL_REACHED（短时保持）
                                   ├─ COMPLETED（保持 2 s 后回到 WAITING_FOR_GOALS）
                                   └─ FAILED（保持到新目标或 /nav/cancel）
```

- 收到新目标时先取消当前 `FollowWaypoints` goal，再发送新目标（latest-wins）。
- 通过 generation 编号/目标 UUID 丢弃旧目标的迟到回调。
- `FollowWaypoints` 返回 SUCCEEDED 且 `missed_waypoints` 为空才算 `COMPLETED`；ABORTED、missed 非空、action server 不存在、坐标转换失败、TF 超时均映射为 `FAILED`。
- `waypoint_follower` 的 action server 在 configure 阶段即存在，但未 activate 时会拒绝 goal；桥接在 `action_server_wait_timeout_sec` 内按 `goal_retry_interval_sec` 自动重试，超时后才置 `FAILED`。
- `/nav/cancel` 取消当前任务并回到 `WAITING_FOR_GOALS`。

### 3.4 GPS 原点

桥接节点与 `gps_imu_fusion` 必须使用一致的原点：

- `origin_source=params`：使用 `origin_lat/lon/alt` 参数，仿真默认 0/0/0。
- `origin_source=topic`：订阅 `/mavros/global_position/gp_origin`，复刻旧 `nav_core` 行为。
- 运行期用最新 `/gps/data` 转换结果与 `/world_odom` 位置做一致性检查，默认容差 5 m；超差拒绝 GPS 目标并打印 ERROR。
- 不使用自动估原点，避免在定位层之外静默改变坐标基准。

## 4. Nav2 栈与插件配置

### 4.1 启动节点

`launch/world_nav2.launch.py` 自行启动以下节点（不直接 include 系统 `navigation_launch.py`，因为本设计需要给 controller/planner 分别传多层参数文件）：

| 节点 | 包 |
| --- | --- |
| controller_server | nav2_controller |
| smoother_server | nav2_smoother |
| planner_server | nav2_planner |
| behavior_server | nav2_behaviors |
| bt_navigator | nav2_bt_navigator |
| waypoint_follower | nav2_waypoint_follower |
| velocity_smoother | nav2_velocity_smoother |
| lifecycle_manager_navigation | nav2_lifecycle_manager |
| multi_goal_bridge_node | robotdog_nav2_bringup |

关键 remap：

- `controller_server`: `cmd_vel → cmd_vel_nav`
- `velocity_smoother`: `cmd_vel → cmd_vel_nav`，`cmd_vel_smoothed → cmd_vel`
- `behavior_server` 不 remap，恢复行为直接输出 `/cmd_vel`
- `lifecycle_manager` 延迟 2 s 启动，等待各节点服务完成 DDS 发现，避免全新域首次 `change_state` 偶发超时
- 可选 RViz（`rviz:=true` 默认开启）：加载 `config/rviz/nav2_world.rviz`，显示 global/local costmap、`/plan`、`/local_plan`、footprint、`/livox/lidar`、`/world_odom`、TF 与 Nav2 面板（含 Nav2 Goal 工具）

### 4.2 分层 YAML

```text
config/
├── common.yaml                 # 所有节点公共参数 + global/local costmap
├── multi_goal_bridge.yaml      # 桥接节点参数
├── controller/
│   ├── rpp.yaml                # 内嵌定制 RPP（含 theta_star 局部重规划）
│   ├── dwb.yaml                # dwb_core::DWBLocalPlanner
│   └── mppi.yaml               # nav2_mppi_controller::MPPIController
└── planner/
    ├── navfn.yaml              # nav2_navfn_planner/NavfnPlanner
    ├── smac.yaml               # nav2_smac_planner/SmacPlanner2D
    └── theta.yaml              # nav2_theta_star_planner/ThetaStarPlanner
```

- controller/planner profile 只写 `FollowPath`/`GridBased` 别名下的 `plugin` 与插件参数；公共参数在 `common.yaml`。
- 插件别名固定为 `FollowPath` 与 `GridBased`，与 Nav2 默认 BT XML 一致；切换插件无需自定义 BT。
- launch 参数 `controller:=rpp|dwb|mppi`、`planner:=navfn|smac|theta`；profile 文件按名称查找并校验存在性。
- 多参数文件按顺序覆盖；Nav2 对未声明参数静默忽略，实施后必须用 `ros2 param get/dump` 验证关键键。

### 4.3 无图 costmap

- local costmap：`world` 坐标系、8 m × 8 m 滚动窗口、0.05 m 分辨率、`ObstacleLayer + InflationLayer`，观测源 `/livox/lidar`（PointCloud2，frame `livox`，量程 5 m）。
- global costmap：`world` 坐标系、200 m × 200 m、原点 (-100,-100)、0.1 m 分辨率、非滚动、`ObstacleLayer + InflationLayer`，不加载静态层。
- global planner 设置 `allow_unknown: true`，远处未知区域可规划；近处避障依赖 local costmap。

## 5. 版本策略

当前工作空间内嵌 `navigation2/` 的 11 个 Nav2 包为 1.1.18，source `install/setup.bash` 后会覆盖系统 1.1.20 的同名包；`planner_server`、`bt_navigator`、`behavior_server`、`waypoint_follower`、`velocity_smoother`、DWB、MPPI、NavFn、SMAC、ThetaStar 等来自系统 1.1.20。

- 新包只按包名引用，不写版本号和 `/opt/ros/humble` 绝对路径。
- 实施阶段先按当前混搭验证；后续迁移到系统 1.1.20 时，只需移除/禁用内嵌 Nav2 包并重新构建，launch/YAML 接口不变。
- 内嵌 RPP 与系统 RPP 同名，pluginlib 会优先加载工作空间版本；迁移后 RPP 内的 theta_star 局部重规划行为会消失，需要单独评估。

## 6. 验证方法

1. 构建：`colcon build --packages-select robotdog_nav2_bringup --symlink-install`。
2. 启动 Gazebo → `gps_imu_fusion` → `world_nav2.launch.py`，确认 7 个 lifecycle 节点 active。
3. 直接调用 `/navigate_to_pose`、`/follow_waypoints` 验证 Nav2 栈。
4. 通过 `execute_task_server` 发送局部/ GPS 航点，观察 `/nav_status`、`/cmd_vel`、`/cmd_vel_nav`。
5. 切换 controller/planner profile，用 `ros2 param get/dump` 确认插件类与参数生效。
6. 验证新目标抢占、`/nav/cancel`、错误 GPS 原点拒绝、MPPI 的 CPU 占用。

## 7. 风险与约束

- Nav2 版本混搭存在 ABI/行为差异风险，必须先跑通运行时验证。
- `/world_odom` 的 twist 恒为 0，`velocity_smoother` 必须使用 `OPEN_LOOP`。
- 5 m 雷达量程限制全局障碍感知范围；远处规划穿越 unknown，实际避障依赖 local costmap。
- 默认 BT 会调用 Spin/Wait/BackUp，`behavior_server` 必须包含对应插件。
- 新栈与旧 `waypoint_nav.launch.py` 不能同时启动：两者都启动 `/controller_server`、订阅 `/waypoint_goals`、发布 `/cmd_vel` 与 `/nav_status`。
- 现有 `test_world_odom_nav.py` 的 `is_gps_aid` 与局部坐标注释矛盾，不作为新栈验收依据；`test_full_system.sh` 的过期路径与绝对 GPS 原点问题不在本包范围内。

## 8. 验证记录

2026-10-20 在不启动 Gazebo 的情况下，用临时 `/world_odom` + TF 仿真脚本完成以下验证：

- 7 个 Nav2 lifecycle 节点（controller、smoother、planner、behavior、bt_navigator、waypoint_follower、velocity_smoother）全部 `active`。
- DWB + NavFn、MPPI + SMAC、RPP + NavFn（GPS 航点）均完成两点任务并返回 `COMPLETED`。
- RPP、DWB、MPPI、NavFn、SMAC 2D、ThetaStar 插件均成功加载。
- 目标抢占：goal A 执行中发送 goal B，A 被取消，B 完成，未出现旧任务的 `FAILED`/`COMPLETED` 干扰。
- `/nav/cancel` 在 goal C 执行中生效，状态回到 `WAITING_FOR_GOALS`，未出现 `FAILED`。
- `/speed_limit` 按航点速度发布，任务结束恢复 0；`/nav_status` 状态序列与旧测试约定一致。
- launch 后立即下发目标时，`waypoint_follower` 尚未 active 会拒绝 goal；桥接自动重试并在节点激活后接受，任务正常完成。
- 尚未在 Gazebo 全保真闭环中验证真实四足运动、点云 costmap 和 TF 时序；实机/Gazebo 验收时按 §6 检查 TF、costmap 与 `/cmd_vel`。
