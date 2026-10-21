# robotdog_nav2_bringup

基于标准 Nav2 的 `world` 无图导航栈，并提供 `MultiGoal → FollowWaypoints` 桥接，使现有 `execute_task_server → /waypoint_goals` 任务链无需改动即可使用。

## 架构

```text
execute_task_server
  → /waypoint_goals (MultiGoal)
  → multi_goal_bridge_node
       ├─ GPS/局部坐标 → world
       ├─ /follow_waypoints (FollowWaypoints)
       │    └─ nav2_waypoint_follower
       │         └─ nav2_bt_navigator → planner_server / controller_server
       │              └─ velocity_smoother → /cmd_vel
       ├─ /speed_limit
       └─ /nav_status
```

- 定位来自 `gps_imu_fusion`：`/world_odom` + TF `world→base_link`。
- 不启动 `map_server`/`amcl`；global/local costmap 均为 `world` 坐标系。
- global costmap 只含障碍层与膨胀层，planner 配置 `allow_unknown: true`。
- 新包与旧 `waypoint_nav.launch.py` **不能同时启动**：两者都会启动 `/controller_server`、订阅 `/waypoint_goals`、发布 `/cmd_vel` 和 `/nav_status`。

## 构建与启动

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select robotdog_nav2_bringup --symlink-install
source install/setup.bash
```

仿真启动顺序：

```bash
export ROS_DOMAIN_ID=42
ros2 launch robot_scene go2_lidar_gps.launch.py
ros2 launch gps_imu_fusion gps_imu_fusion.launch.py use_sim_time:=true
ros2 launch robotdog_nav2_bringup world_nav2.launch.py \
  use_sim_time:=true controller:=rpp planner:=navfn
ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1
```

实机：

```bash
ros2 launch robotdog_nav2_bringup world_nav2.launch.py \
  use_sim_time:=false simulation:=false controller:=rpp planner:=smac \
  origin_lat:=<与 gps_imu_fusion 一致> origin_lon:=... origin_alt:=...
```

`simulation:=false` 时 launch 会发布 `base_link → livox` 静态 TF；仿真中该 TF 由 `robot_state_publisher` 提供。

## RViz 可视化

`rviz` 参数默认开启，启动时自动加载 `config/rviz/nav2_world.rviz`（Fixed Frame 为 `world`）：

- Global/Local Costmap（`/global_costmap/costmap`、`/local_costmap/costmap`，Transient Local）
- Global Plan（`/plan`）、Local Plan（`/local_plan`）
- Local Footprint（`/local_costmap/published_footprint`）
- Livox PointCloud（`/livox/lidar`）
- World Odom（`/world_odom` 位姿箭头）
- TF（world/base_link/trunk/body/livox）、RobotModel（默认关闭，可手动打开）
- Nav2 面板 + "Nav2 Goal" 工具，可直接在 RViz 中向 `bt_navigator` 下发单个目标做测试

无显示环境（如远程无 GUI）启动时用 `rviz:=false`：

```bash
ros2 launch robotdog_nav2_bringup world_nav2.launch.py use_sim_time:=true controller:=rpp planner:=navfn rviz:=false
```

## 插件切换

controller profile：`config/controller/`

| 名称 | 插件 |
| --- | --- |
| `rpp` | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`（工作空间内嵌定制版） |
| `dwb` | `dwb_core::DWBLocalPlanner` |
| `mppi` | `nav2_mppi_controller::MPPIController` |

planner profile：`config/planner/`

| 名称 | 插件 |
| --- | --- |
| `navfn` | `nav2_navfn_planner/NavfnPlanner` |
| `smac` | `nav2_smac_planner/SmacPlanner2D` |
| `theta` | `nav2_theta_star_planner/ThetaStarPlanner` |

选择方式：

```bash
ros2 launch robotdog_nav2_bringup world_nav2.launch.py controller:=mppi planner:=smac
ros2 param get /controller_server FollowPath.plugin
ros2 param get /planner_server GridBased.plugin
ros2 param get /controller_server FollowPath.critics
ros2 param dump /controller_server
```

Nav2 对未声明参数不会报错，参数名拼错时只会静默使用默认值；切换后务必用 `ros2 param get/dump` 核对关键参数。

### 新增自定义插件

1. 新建插件包，实现 `nav2_core::Controller` 或 `nav2_core::GlobalPlanner`，并导出 pluginlib XML。
2. 在 `config/controller/` 或 `config/planner/` 新增 `myplugin.yaml`，保持别名 `FollowPath`/`GridBased` 不变，只改 `plugin:` 与插件参数。
3. 启动时传 `controller:=myplugin` 或 `planner:=myplugin`。launch 只按文件名查找，无需修改 launch 代码。
4. 用 `ros2 param get` 确认插件类已加载。

默认 BT XML 固定使用 `GridBased` 与 `FollowPath` 两个 ID，因此新增插件时不要改别名；若必须改，需要同时提供自定义 BT XML。

## 桥接节点

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

- `is_gps_aid=true` 时按经纬度转换到 `world`；`false` 时直接作为 `world` 局部坐标。
- `yaw_mode=path`（默认）按路径方向生成朝向；`yaw_mode=msg` 使用 `MultiGoalPoint.yaw`。
- `vel` 通过 `/speed_limit` 限制当前航点速度，默认 0.1–0.6 m/s；完成/失败/取消后恢复控制器默认限速。
- 新目标会取消当前任务并执行最新目标；`/nav/cancel` 可取消当前任务。
- 启动阶段 `waypoint_follower` 尚未 active 时 action server 会拒绝目标，桥接会在 `action_server_wait_timeout_sec` 内自动重试，超时后才报 `FAILED`。
- `/nav_status` 字符串与旧 `nav_fsm.cpp` 一致：`WAITING_FOR_GOALS`、`INITIALIZING`、`EXECUTING_PATH`、`GOAL_REACHED`、`COMPLETED`、`FAILED`。

### GPS 原点

`origin_lat/lon/alt` 必须与 `gps_imu_fusion` 的参数一致。仿真默认均为 0。桥接节点会用最新 `/gps/data` 与 `/world_odom` 做一致性检查，默认容差 5 m；超差时拒绝 GPS 目标并在日志中报错。绝对经纬度任务（如 30.06888,120.04646）必须同时修改 `gps_imu_fusion` 与桥接节点的原点。

## 版本说明

当前工作空间的 `src/robotdog_nav/navigation2/` 内嵌 Nav2 1.1.18，会覆盖系统 1.1.20 的 `nav2_controller`、`nav2_costmap_2d`、`nav2_core`、`nav2_lifecycle_manager` 等；`planner_server`、`bt_navigator`、`behavior_server`、`waypoint_follower`、`velocity_smoother`、DWB、MPPI、NavFn、SMAC、ThetaStar 来自系统 1.1.20。新包只按包名引用，不写版本与绝对路径。后续迁移到纯系统 Nav2 时，移除/禁用内嵌包并重新构建即可；注意内嵌定制 RPP 的 ThetaStar 局部重规划行为会随迁移消失。

## 已知限制

- `/world_odom` 的 twist 恒为 0，`velocity_smoother` 使用 `OPEN_LOOP`。
- `/livox/lidar` 有效量程约 5 m，远处障碍不可见；global planner 可穿越 unknown，近处避障依赖 local costmap。
- global costmap 固定为 200 m × 200 m、原点 (-100,-100)，更大场地需调整 `common.yaml` 的 `width/height/origin_x/origin_y`。
- MPPI 在 CPU 仿真机上可能吃满 CPU，可降低 `batch_size`/`time_steps` 或改用 DWB/RPP。
- 现有 `waypoint_nav/test/test_world_odom_nav.py` 的 `is_gps_aid=True` 与局部坐标注释矛盾，不能直接作为新栈验收脚本；测试时请改为 `is_gps_aid=False` 或使用正确的 GPS 坐标。
