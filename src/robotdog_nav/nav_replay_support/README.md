# nav_replay_support

回放场景的导航栈支撑包：把 rosbag 回放的里程计转成 TF，并提供传感器静态外参，使导航栈
无需 Gazebo 与 `gps_imu_fusion` 即可运行（数据源为仅含 `/livox/lidar` + `/world_odom` 的 bag）。

完整录制/回放手册见仓库根目录 `doc/rosbag_record_replay.md`。

## 组成

| 文件 | 职责 |
|---|---|
| `src/odom_to_tf_node.cpp` | 订阅 `nav_msgs/Odometry`，按消息自身时间戳广播 TF（默认 `world→base_link`） |
| `launch/odom_to_tf.launch.py` | 只启动适配节点（通用） |
| `launch/replay_tf.launch.py` | 适配节点 + 静态 `base_link→livox`（回放专用） |

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

## 已知限制

- 回放为开环：bag 中的位姿不响应回放期间新发的 `/cmd_vel`，不验证闭环到达。
- `/world_odom` 的 twist 恒为 0，仅位姿有效。
- 回放包不含 `/gps/data`，任务只能用局部坐标（`waypoint_frame=local`）。
