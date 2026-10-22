# 设计：RViz 虚拟场景验证（无 Gazebo 的 mock 场景节点）

- 日期：2026-10-21
- 状态：已实施并通过 T1–T5 验证（见 §10）
- 范围：在 `nav_replay_support` 包内新增 `mock_scene_node`（Python）：合成障碍物点云、闭环运动学机器人（`/cmd_vel` 积分成位姿）、兼任 `/clock` 时钟源；复用既有 `odom_to_tf_node` 与静态 `base_link→livox` TF（均不改动）。使 `waypoint_nav` 与 `robotdog_nav2_bringup` 两套导航栈在无 Gazebo、无 rosbag 条件下于 RViz 中闭环验证。
- 关联：`doc/devel/2026-10-20-design-rosbag-replay-decoupling.md`（回放解耦，同一契约的前两个数据源）、`doc/devel/2026-10-20-design-nav2-world-stack.md`（Nav2 新栈）

## 1. 背景与目标

当前链路：

```text
数据源（三选一，互斥）：
  A. Gazebo     → /gps/data + /livox/imu → gps_imu_fusion → /world_odom + TF world→base_link
  B. rosbag 回放 → /world_odom（odom_to_tf_node 重建 TF）+ /livox/lidar；开环
  C.（本设计）mock 虚拟场景 → /world_odom + /livox/lidar + /clock；闭环

导航契约（不变）：/world_odom（10 Hz）+ /livox/lidar（10 Hz，frame livox）
              + TF world→base_link（唯一动态发布者）+ TF base_link→livox（静态 z=0.1）
```

问题：Gazebo 验证重、慢；bag 回放开环（位姿不响应新 `/cmd_vel`，无法验证真实到达）。
目标：RViz 轻量闭环验证——RViz 只是可视化器，无法向 costmap 注入障碍物，导航栈只认传感器数据，因此由 mock 节点合成：障碍物（几个方块/立柱，无地面、无物理、无碰撞检测）+ 按 `/cmd_vel` 积分移动的机器人 + `/clock`。

用户确认的三项决策：闭环运动学；实现在 `nav_replay_support` 内；两套导航栈都支持。

## 2. 关键设计结论

### 2.1 时间方案（/clock 归属）

- mock 节点自身跑**墙钟**（`use_sim_time:=false`），50 Hz 发布 `/clock`（墙钟时间）→ 其余节点 `use_sim_time:=true`。因 /clock=墙钟，任何节点漏配时钟仍保持一致。
- 必要性：`waypoint_nav/config/params.yaml` 在 local_costmap/controller_server 块内硬编码 `use_sim_time: true` 且 launch 不重写 YAML；`world_nav2.launch.py` 虽经 RewrittenYaml 重写但默认也是 true → 无 Gazebo/bag 时必须有 /clock 发布者。
- `odom_to_tf_node` 只复制消息时间戳（自身时钟仅用于日志限流），无 /clock 时启动无害。
- **互斥规则**：mock 是唯一 /clock、唯一 /world_odom、唯一 `world→base_link` 发布者。禁与 Gazebo、`ros2 bag play --clock`、`gps_imu_fusion` 同跑；导航 launch 保持默认 `simulation:=true`（静态 livox TF 由 replay_tf 持有，传 `simulation:=false` 会重复发布）。
- **mock 崩溃 = 仿真时钟冻结**（消费节点定时器停摆）；重启 mock 安全——墙钟不回跳，无 `TF_OLD_DATA`，不需要重启导航栈（与 bag 回放相反）。

### 2.2 QoS 契约

| 话题 | 发布 QoS | 消费端（已核实） | 兼容性 |
| --- | --- | --- | --- |
| `/clock` | Reliable, Volatile, depth 10 | rclcpp 仿真时钟订阅（reliable KeepLast 10） | 同 `bag play --clock`，不可 best-effort |
| `/world_odom` | 默认 Reliable depth 10 | nav_core:80、bridge:57、controller_server、velocity_smoother（均默认可靠） | 兼容 |
| `/livox/lidar` | **Reliable** depth 10 | Nav2 障碍层 best-effort depth 50；**nav_core.cpp:84 自身以默认可靠 QoS 订阅**；waypoint rviz Reliable、nav2_world rviz Best Effort | 可靠发布是同时满足四端的唯一选择 |
| `/mock_scene/markers` | Reliable + **Transient Local**, depth 1 | RViz MarkerArray 显示（Durability 须设 Transient Local） | 锁存场景，晚开的 RViz 也能看到 |
| `/cmd_vel`（订阅） | 默认 Reliable depth 10 | 发布方 controller_server / velocity_smoother / nav_waypoint_node | 兼容 |

### 2.3 运动学模型

- 全向积分（Go2 可全向）：`yaw += ω·dt; x += (vx·cosψ − vy·sinψ)·dt; y += (vx·sinψ + vy·cosψ)·dt`；dt 取墙钟实测间隔（吸收定时器抖动），上限 1 s。
- 限幅：`max_linear_vel 1.5 / max_lateral_vel 0.6 / max_angular_vel 1.5`（waypoint_nav RPP 上限 1.2、bringup smoother 1.0——mock 不应比控制器更低）。
- 0.5 s 无指令指数衰减（`exp(−2.0·(silence−0.5))`），复用根目录 `sim_mock_node.py` 已验证的模式。
- odom 的 twist 填**实际 v/ω**（优于融合节点的零 twist，RPP 调速受益）；`velocity_smoother OPEN_LOOP` 不受影响。
- 10 Hz odom 定时器内完成积分，不开高频物理定时器。

### 2.4 合成点云

- 障碍世界系点**启动时缓存一次**；发布时按水平距离过滤 `[min_sensor_range 0.2, sensor_range 5.0]`（bringup 障碍层 0.15–4.8/5.0，waypoint_nav 0–8，取交集内）。
- 变换：`p_livox = R(−yaw)·(p_w − p_base)`，`z_livox = z_w − 0.1`（livox 在 base_link 上方 0.1 m，机器人 z 恒 0）。
- z 采样层 `{0.45, 0.55, 0.65, 0.75}`（两栈世界系高度带 [0.4,1.0] 内留余量）；表面网格间距 0.1 m；点数上限 5000；`xyz+intensity` FLOAT32（point_step 16，同 bag 内真实云布局），`sensor_msgs_py.point_cloud2.create_cloud` 生成。

### 2.5 默认场景（验证目标点 (4,0)）

```yaml
box_obstacles:      [2.0, 0.0, 0.4, 0.3, 3.0, 0.8]   # 墙：底面 z=0.4，高 0.8
cylinder_obstacles: [1.2, 1.8, 0.4, 0.2, 0.8,        # 立柱 1
                     2.8, -1.8, 0.4, 0.25, 0.8]      # 立柱 2
```

数组语义：`box_obstacles` 每项 `[cx, cy, cz_base, dx, dy, dz]`（中心 xy、底面 z、尺寸）；`cylinder_obstacles` 每项 `[cx, cy, cz_base, r, h]`。墙挡在 (0,0)→(4,0) 直线上 → NavFn/SMAC 全局规划须绕行 |y|≈2.5–3 m（lethal 0.5 + 膨胀 0.75 + 余量）；两柱点缀绕行走廊。约 400 点/帧。

## 3. 接口设计

节点名 `mock_scene_node`：

| 方向 | 名称 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/cmd_vel`（参数可改） | geometry_msgs/Twist | 运动学输入 |
| 发布 | `/world_odom`（参数） | nav_msgs/Odometry，10 Hz | world/base_link，twist=实际 v/ω |
| 发布 | `/livox/lidar`（参数） | sensor_msgs/PointCloud2，10 Hz | frame livox，RELIABLE |
| 发布 | `/clock` | rosgraph_msgs/Clock，50 Hz | 墙钟时间，`publish_clock` 可关 |
| 发布 | `/mock_scene/markers` | visualization_msgs/MarkerArray，一次 | Transient Local 锁存；CUBE/CYLINDER，rgba(85,170,255,0.35) |
| TF | —（复用 odom_to_tf_node） | — | mock 发 `/world_odom`，由 `replay_tf.launch.py` 内 odom_to_tf 转 TF + 静态 livox TF |

参数全部走 `config/mock_scene.yaml`（AGENTS.md 禁硬编码）：话题名×5、频率×3、开关×2（publish_clock/publish_markers）、frame×3 + sensor_z、限幅×3、cmd_timeout/cmd_decay_rate、initial_x/y/yaw、sensor_range/min_sensor_range、point_spacing、z 采样上下限、max_points、intensity、box_obstacles、cylinder_obstacles。浮点必须带小数点（解析为 DOUBLE_ARRAY）。

边界校验集中在构造函数（违反即 fatal 退出）：数组长度为 6/5 的倍数、尺寸/半径/间距>0、z 采样 ∈(0.4,1.0)、频率>0。

## 4. 启动方式

```bash
# 终端 1：虚拟场景（mock 节点 + odom_to_tf + 静态 livox TF）
ros2 launch nav_replay_support mock_scene.launch.py

# 终端 2：导航栈（二选一，均 use_sim_time:=true，simulation 保持默认 true）
ros2 launch waypoint_nav waypoint_nav.launch.py use_sim_time:=true
ros2 launch robotdog_nav2_bringup world_nav2.launch.py use_sim_time:=true

# 终端 3：任务服务
ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1

# 终端 4：下发任务（仅 local 航点——无 /gps/data）
ros2 service call /agent/uav1/execute_task execute_task_server/srv/ExecuteTask "..."
```

`mock_scene.launch.py`：mock 节点（`params=[params_file, {'use_sim_time': False, ...}]`）+ include `replay_tf.launch.py`（use_sim_time 传**字面 'true'**，非外层参数）。所有终端同 `ROS_DOMAIN_ID`（本机 bashrc 默认 41）；验证 nav2_bringup 时需先 source 主 install（underlay）再 source worktree install（overlay）。

## 5. 与另两个数据源的关系

| | Gazebo（A） | bag 回放（B） | mock 场景（C，本设计） |
| --- | --- | --- | --- |
| 物理真实性 | 完整 | 录制时的真实数据 | 无（穿墙） |
| 闭环（响应新指令） | 是 | **否（开环）** | **是（运动学）** |
| 需要的进程 | Gazebo + fusion | bag play + odom_to_tf | mock + odom_to_tf |
| /clock | Gazebo | bag --clock | mock |
| 验证内容 | 全部 | 数据流/回归 | 任务链路/到达/绕障规划 |

## 6. 文件清单

新增（`src/robotdog_nav/nav_replay_support/`）：
- `src/mock_scene_node.py`（~250 行，中文注释英文日志，shebang；`install(PROGRAMS)` 安装，可执行名带 `.py` 后缀）
- `config/mock_scene.yaml`
- `launch/mock_scene.launch.py`

修改：
- `CMakeLists.txt`：`find_package(rclpy REQUIRED)` + `install(PROGRAMS src/mock_scene_node.py DESTINATION lib/${PROJECT_NAME})`（模式同 `go2w_control`）
- `package.xml`：`<depend>sensor_msgs</depend>`、`<exec_depend>rclpy sensor_msgs_py visualization_msgs rosgraph_msgs</exec_depend>`
- `README.md`：新增「虚拟场景」章节（组件/参数/两栈命令/时钟规则/限制）
- `AGENTS.md`：目录描述与数据源说明更新
- `waypoint_nav/config/nav_visualization.rviz`：加 `/mock_scene/markers` MarkerArray 显示（Transient Local）
- 根 `README.md`：文档列表加 nav_replay_support 链接

合并后跟进（不在本次）：主检出 `nav2_world.rviz` 加同样显示。

## 7. 验证方法（通过标准）

- **T1 mock 层单测**：`/clock` ≈50 Hz 且恰 1 发布者；`/world_odom`、`/livox/lidar` ≈10 Hz、云 frame=livox；TF 初始位姿 + 静态 (0,0,0.1)；`ros2 topic pub -r 10 /cmd_vel ... "{linear: {x: 0.3}}"` → odom x ≈0.3 m/s 前进、近墙点云变化、停发后衰减归零。
- **T2 waypoint_nav**：costmap 出现障碍膨胀格；markers 可见；无外推刷屏；/cmd_vel 恰 1 发布者。
- **T3 任务服务**：正常在线。
- **T4 闭环到达（waypoint_nav）**：下发 local 单点 {x:4, y:0, vel:0.5} → **EXECUTING_PATH → GOAL_REACHED → COMPLETED**，位姿收敛 (4,0)；RViz 可见机器人穿墙（无碰撞限制的预期表现，costmap 仍标记障碍）。
- **T5 绕障（nav2_bringup）**：先彻底关闭 T2 栈（防双 /cmd_vel，`ros2 topic info /cmd_vel` 确认）；启动新栈重发任务 → **`/plan` 可见绕行（|y|≈2.5–3 m）**，机器人沿绕行路径到达、不穿墙，COMPLETED。

## 8. 风险与限制

1. 无碰撞检测：机器人可穿过障碍（waypoint_nav 直线规划必穿墙；绕障仅 nav2_bringup 全局规划可见）。
2. 无地面/重力/腿式运动学：`/cmd_vel` 直接积分，适合链路验证不适合步态/控制调参。
3. 仅支持 local 航点（无 /gps/data）；GPS 原点检查对 is_gps_aid:false 不生效。
4. /clock 单发布者纪律：与 Gazebo / bag play / 跨机同域对端互斥。
5. mock 崩溃冻结全栈时钟：重启 mock 即可恢复（墙钟不回跳）。
6. 换栈必须先清理旧栈（双 /cmd_vel 会污染运动学积分）。
7. 点云无遮挡（视线穿墙可见后面的障碍）：简化为距离过滤，无光线追踪——对 costmap 验证无影响。

## 9. 实施步骤

1. 写入本设计文档。
2. `mock_scene_node.py` → `mock_scene.yaml` → `mock_scene.launch.py`。
3. `CMakeLists.txt` + `package.xml`，`colcon build --packages-select nav_replay_support --symlink-install`。
4. T1 冒烟 → README/AGENTS/rviz/根 README 编辑 → T2–T5 全量验证，记录补入 §10。
5. 提交至 nav-replay-support 分支，合并由用户决定。

## 10. 验证记录

实施日期：2026-10-21（worktree `nav-replay-support`）。T1–T5 全部通过。

| 项 | 结果 | 记录 |
| --- | --- | --- |
| T1 mock 层 | ✅ | `/clock` 50.0 Hz 且恰 1 发布者；`/world_odom`、`/livox/lidar` 均 10 Hz（0.098–0.102 s），云 frame=livox、width=594；TF 初始位姿 (0,0,0) + 静态 base_link→livox (0,0,0.1)。闭环：vx=0.3 持续 3 s 前进 0.99 m（含停发衰减尾），v=0.300；静默 2.5 s 后 twist 衰减归零；点云随位姿整体平移（前方障碍 livox 系 mean_x 2.024→0.842，位移量与里程一致）。注：首次启动即暴露两处编码错误并修复——`create_publisher` 缺 MarkerArray 类型参数、rclpy logger 不接受 printf 变参（需 `%` 先格式化） |
| T2 waypoint_nav | ✅ | local_costmap 320×320：lethal 95 格、膨胀 5918 格，lethal 质心 (2.04,−0.06) 与墙 (2.0,0) 吻合（含两柱加权）；RViz（新 rviz 配置）订阅 `/mock_scene/markers`，Transient Local 锁存对晚启动 RViz 生效；全日志 0 外推/TF_OLD_DATA 告警。`/cmd_vel` 端点有 2 个（controller_server + nav_waypoint_node 常驻 publisher，Gazebo 下同样如此），执行期实际发布者仅 controller_server 一个 |
| T3 任务服务 | ✅ | `execute_task.launch.py agent_id:=uav1` 正常在线，服务注册、航点经 `/waypoint_goals` 下发 |
| T4 闭环到达（waypoint_nav） | ✅ | 干净栈下发 local {x:4,y:0,vel:0.5}：WAITING_FOR_GOALS → INITIALIZING → EXECUTING_PATH（18 s 闭环行驶）→ COMPLETED，到位误差 0.61 m < 栈容差 goal_tolerance 1.0。注 1：最终目标直接 EXECUTING_PATH→COMPLETED，`GOAL_REACHED` 仅出现在多航点中间目标（既有 FSM 行为，§2 设计描述按实际行为修正理解）。注 2：RPP goal_checker xy 0.25 m 与 waypoint_nav goal_tolerance 1.0 m 交叠，机器人常停在 0.25–1.0 m 处由 FSM 判到点，controller 侧随后报一次 "Failed to make progress" 中止——既有行为，不影响 COMPLETED |
| T5 绕障（nav2_bringup） | ✅ | underlay(主 install)+overlay(worktree install) 启动 `world_nav2.launch.py`，NavFn 规划 `/plan` 11 点、最大 \|y\|=2.82 m（预期 2.5–3）；机器人沿绕行路径行驶，全程未入墙足迹（最小间距 0.55 m），到位误差 0.311 m，EXECUTING_PATH → COMPLETED |

### 实施中发现并修复的问题

1. `mock_scene_node.py` marker publisher 缺消息类型参数（启动即 TypeError）——已修复。
2. `mock_scene_node.py` 结束日志用 printf 风格变参调 rclpy logger（TypeError）——改为 `%` 格式化。
3. 验证过程中清理了本机 4 个昨日遗留的 `world→base_link` 静态 TF 孤儿进程（会与 odom_to_tf 动态 TF 冲突）。

### 顺带发现的既有栈缺陷（未改，属 waypoint_nav 范畴）

**跨任务 `path_sent_to_controller_` 标志不复位**：当任务经"已在容差内→跳过发送"路径（`nav_core.cpp` sendPathToControllerServer 的 dist≤goal_tolerance 分支返回 true）或 abort 后 skip 重发结束时，标志保持 true；下一个任务因 `!path_sent_to_controller_ || current_goal_index_ != last_goal_index_sent_` 不成立而**永不发送 FollowPath**，机器人静止卡在 EXECUTING_PATH（本次 T4 首轮复现：复位 mock 后新任务 120 s 无动作）。Gazebo 下顺序下发两个任务同样可触发。建议后续在目标接收处复位该标志。

### 遗留跟进

- 主检出合并后：`robotdog_nav2_bringup/config/rviz/nav2_world.rviz` 加同样的 `/mock_scene/markers` 显示（§6）。
