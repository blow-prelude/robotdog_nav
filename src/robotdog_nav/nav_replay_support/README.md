# nav_replay_support

无 Gazebo 场景下导航栈的支撑包。两种用法：

1. **bag 回放**：把回放的里程计转成 TF 并提供传感器静态外参（开环，数据源为仅含
   `/livox/lidar` + `/world_odom` 的 bag）；
2. **虚拟场景（mock）**：合成障碍点云 + `/cmd_vel` 闭环运动学机器人 + `/clock`，
   在 RViz 中轻量闭环验证任务链路（无需 Gazebo、无需 rosbag）。

完整录制/回放手册见仓库根目录 `doc/rosbag_record_replay.md`；虚拟场景设计见
`doc/devel/2026-10-21-design-mock-scene.md`。

## 组成

| 文件 | 职责 |
|---|---|
| `src/odom_to_tf_node.cpp` | 订阅 `nav_msgs/Odometry`，按消息自身时间戳广播 TF（默认 `world→base_link`） |
| `src/mock_scene_node.py` | 虚拟场景：`/clock`（50 Hz 墙钟）+ `/world_odom`（运动学积分）+ `/livox/lidar`（合成障碍点云）+ `/mock_scene/markers`（场景锁存显示） |
| `launch/odom_to_tf.launch.py` | 只启动适配节点（通用） |
| `launch/replay_tf.launch.py` | 适配节点 + 静态 `base_link→livox`（回放专用） |
| `launch/mock_scene.launch.py` | mock 节点 + `replay_tf`（虚拟场景专用） |

节点不融合、不改写里程计、不发布 `/world_odom`；回放时**不要**启动 `gps_imu_fusion`，
否则会出现重复的 `/world_odom` 与 `world→base_link` 发布者。

## 参数

`config/params.yaml` / launch 参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `odom_topic` | `/world_odom` | 里程计话题 |
| `parent_frame` | `""` | 非空时覆盖 `header.frame_id` |
| `child_frame` | `""` | 非空时覆盖 `child_frame_id` |

`replay_tf.launch.py` 额外参数：`publish_static_livox_tf`（默认 `true`）、
`livox_x/y/z`（默认 `0 0 0.1`，与 Go2 xacro 外参一致）、`livox_roll/pitch/yaw`（默认 0）。

`config/mock_scene.yaml`（`mock_scene_node` 参数）：

| 参数组 | 参数 | 默认 | 说明 |
|---|---|---|---|
| 话题 | `cmd_vel_topic` `odom_topic` `lidar_topic` `clock_topic` `marker_topic` | `/cmd_vel` `/world_odom` `/livox/lidar` `/clock` `/mock_scene/markers` | 全部可改 |
| 频率/开关 | `clock_rate` `odom_rate` `lidar_rate` `publish_clock` `publish_markers` | 50/10/10 Hz，均开 | |
| 坐标系 | `world_frame` `base_frame` `sensor_frame` `sensor_z` | `world` `base_link` `livox` 0.1 | 与导航契约一致 |
| 运动学 | `max_linear_vel` `max_lateral_vel` `max_angular_vel` `cmd_timeout` `cmd_decay_rate` | 1.5/0.6/1.5、0.5 s、2.0 | 限幅不低于控制器上限（RPP 1.2、smoother 1.0） |
| 初始位姿 | `initial_x` `initial_y` `initial_yaw` | 0/0/0 | yaw 弧度 |
| 点云 | `sensor_range` `min_sensor_range` `point_spacing` `z_sample_min/max` `max_points` `point_intensity` | 5.0/0.2/0.1/0.45~0.95/5000/100 | z 采样窗须在 (0.4, 1.0)（两栈障碍高度带） |
| 场景 | `box_obstacles` `cylinder_obstacles` | 1 墙 + 2 柱 | 每项 `[cx,cy,cz_base,dx,dy,dz]` / `[cx,cy,cz_base,r,h]`，扁平 double 数组 |

## 回放示例

```bash
# 终端 1：回放数据源（合成时钟；只回放雷达与里程计）
ros2 bag play sim_bag_YYYYMMDD_HHMMSS --clock 40 --topics /livox/lidar /world_odom

# 终端 2：里程计 → TF + 静态外参
ros2 launch nav_replay_support replay_tf.launch.py use_sim_time:=true

# 终端 3：导航栈（保持 simulation 默认 true，不要传 false）
ros2 launch waypoint_nav waypoint_nav.launch.py use_sim_time:=true
# 或新栈：
# ros2 launch robotdog_nav2_bringup world_nav2.launch.py use_sim_time:=true rviz:=false

# 终端 4：任务服务
ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1
```

自检：

```bash
ros2 run tf2_ros tf2_echo world base_link      # 随回放轨迹连续变化
ros2 run tf2_ros tf2_echo base_link livox      # 恒为 (0, 0, 0.1)
```

## 虚拟场景示例（无 Gazebo、无 bag 的闭环验证）

```bash
# 终端 1：虚拟场景（mock 节点 + odom_to_tf + 静态 livox TF）
ros2 launch nav_replay_support mock_scene.launch.py

# 终端 2：导航栈（二选一，use_sim_time:=true；simulation 保持默认 true）
ros2 launch waypoint_nav waypoint_nav.launch.py use_sim_time:=true
# 或
ros2 launch robotdog_nav2_bringup world_nav2.launch.py use_sim_time:=true

# 终端 3：任务服务
ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1

# 终端 4：下发任务（仅 local 航点——无 /gps/data）
ros2 service call /agent/uav1/execute_task execute_task_server/srv/ExecuteTask \
  "{request_json: '{\"ability_id\":\"goto_waypoints\",\"waypoints\":[{\"x\":4.0,\"y\":0.0,\"vel\":0.5}],\"waypoint_frame\":\"local\"}'}"
```

RViz 中：半透明蓝色方块/圆柱 = 合成障碍（`/mock_scene/markers`），点云应与障碍表面贴合。

时钟规则与互斥（重要）：

- mock 节点自身跑墙钟并发布 `/clock`（墙钟时间），其余节点 `use_sim_time:=true`；
  任何节点漏配时钟仍保持一致（/clock = 墙钟）。
- mock 是唯一 `/clock`、唯一 `/world_odom`、唯一 `world→base_link` 发布者——
  **禁止**与 Gazebo、`ros2 bag play --clock`、`gps_imu_fusion` 同跑。
- 导航 launch 保持默认 `simulation:=true`；传 `simulation:=false` 会重复发布静态 `base_link→livox`。
- mock 崩溃 = 仿真时钟冻结（导航定时器停摆）；重启 mock 即可恢复（墙钟不回跳，
  无 `TF_OLD_DATA`，无需重启导航栈）。
- 换栈（waypoint_nav ↔ nav2_bringup）前必须先彻底关闭旧栈，否则双 `/cmd_vel` 污染运动学积分。

## 已知限制

- 回放为开环：bag 中的位姿不响应回放期间新发的 `/cmd_vel`，不验证闭环到达。
- `/world_odom` 的 twist 恒为 0，仅位姿有效。
- 回放包不含 `/gps/data`，任务只能用局部坐标（`waypoint_frame=local`）。
- mock 无碰撞检测：机器人可穿过障碍（waypoint_nav 直线规划必穿墙；绕障仅 nav2_bringup
  全局规划可见），适合链路验证不适合步态/控制调参。
- mock 仅支持 local 航点（无 `/gps/data`）；GPS 原点检查对 `is_gps_aid:false` 不生效。
