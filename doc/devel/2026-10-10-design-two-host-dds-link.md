# 设计：双机通信层（原生 DDS，纯配置）

- 日期：2026-10-10
- 状态：待评审
- 范围：A 机（x86_64 仿真机）与 B 机（arm64 导航机）经局域网 WiFi 的双向 ROS 2 通信
- 方案基调：**纯配置层**（零仓库代码改动；环境变量 + DDS XML profile + 脚本 + 文档）
- 实现文档：[2026-10-10-plan-two-host-dds-link.md](2026-10-10-plan-two-host-dds-link.md)

## 1. 背景与目标

### 1.1 现状与分工

本仓库为宇树 Go2/Go2W 四足航点导航栈（指控端 JSON 任务 → 导航 → Gazebo 闭环），原设计单机运行。现拆分为两机：

| 机器 | 硬件 / 网络 | 运行内容 |
| --- | --- | --- |
| A（仿真机） | x86_64 台式机；**仅有线网卡 enp2s0，10.88.103.177/24**，经网线接路由器 | Gazebo Classic 11 仿真（`ros2 launch robot_scene go2_lidar_gps.launch.py`）：champ 步态、ros2_control、传感器插件（/livox/lidar、/livox/imu、/gps/data、/clock、/tf）、hybrid_motion_controller |
| B（导航机） | arm64；WiFi 接入同一路由器/AP（IP 与网卡名待填） | gps_imu_fusion（/world_odom + TF world→base_link）、waypoint_nav（nav_waypoint_node + 内嵌 Nav2 controller_server/local_costmap）、RViz、execute_task_server |

### 1.2 目标与约束

- 两机经无线链路完成双向数据收发：A→B 传感器/时钟/TF，B→A /cmd_vel，无需人工逐话题转发。
- **最大程度与现有代码解耦**：机制全部落在环境变量 + DDS XML profile + 脚本 + 文档；现有 ROS 节点、launch、params.yaml 一律不动。
- **保持通信流畅**：发现可靠、带宽可控（大样本调优）、QoS 兼容、延迟/抖动可接受。
- **先实测再定降载**：点云是否降载由实测带宽决定，不预先削减保真度。

### 1.3 已确认决策（评审结论）

1. 定位 + 导航 + RViz 全部在 B 机；A 机只跑仿真。
2. 链路为普通 WiFi（同一路由器/AP，组播通常可用；方案同时准备单播兜底）。
3. 实现方式为纯配置层（零代码）。
4. 降载与否先实测。
5. RMW 以 CycloneDDS 为主，Fast DDS 为备选。

## 2. 架构拓扑

```
┌──────────────────────── A 机（x86_64，仿真）────────────────────────┐
│ Gazebo + champ + ros2_control + hybrid_motion_controller            │
│ 发布 /livox/lidar(10Hz, 480KB/帧) /livox/imu(200Hz) /gps/data(30Hz) │
│      /clock(≈200Hz) /tf(≈200Hz) /tf_static /robot_description       │
│ 订阅 /cmd_vel                                                       │
│ 网卡 enp2s0 10.88.103.177/24（有线，接路由器）                       │
└──────────────────────────────┬──────────────────────────────────────┘
                               │ 原生 ROS 2 DDS（RTPS/UDP）
                               │ 域 33 · CycloneDDS · spdp 发现 + 单播对端兜底
                               │ 大样本分片/缓冲调优（480KB/帧）
┌──────────────────────────────┴──────────────────────────────────────┐
│ 路由器 / AP  ←── WiFi ──────────────────────────────┐               │
└─────────────────────────────────────────────────────┼───────────────┘
┌──────────────────── B 机（arm64，导航）─────────────┴───────────────┐
│ gps_imu_fusion：/gps/data + /livox/imu → /world_odom + TF(10Hz)     │
│ waypoint_nav：nav_waypoint_node(FSM) + controller_server/           │
│               local_costmap（消费 /livox/lidar 建代价地图）          │
│ RViz · execute_task_server（/agent/uav1/execute_task）              │
│ 发布 /cmd_vel(10Hz) · 网卡 WiFi（IP/网卡名待填）                     │
└─────────────────────────────────────────────────────────────────────┘
```

通信层由三层机制构成，全部零代码：

1. **共享域**：两机 `ROS_DOMAIN_ID=33` + `ROS_LOCALHOST_ONLY=0`。
2. **统一 RMW + 调优 profile**：`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`、`CYCLONEDDS_URI=file://.../net/cyclonedds_{sim,nav}.xml`——pin 物理网卡、组播仅用于发现 + 对端 IP 单播兜底、大样本缓冲/水位/重传队列调优。
3. **环境脚本**：`source net/env_sim.sh`（A）/ `source net/env_nav.sh`（B）一键生效并打印摘要；`net/check_link.sh` 一键诊断。

## 3. 跨机数据流（频率 / QoS / 带宽均已核实）

| 方向 | 话题 | 类型 | 频率 | 带宽 | QoS（发布端） | 消费方 |
| --- | --- | --- | --- | --- | --- | --- |
| A→B | /livox/lidar | PointCloud2 | 10 Hz | **≈4.8 MB/s**（20000 点 × 24 B） | RELIABLE / keep10 | costmap(BEST_EFFORT)、RViz(**Reliable**)、nav_core(**Reliable 空回调**) |
| A→B | /tf | TFMessage | ≈200 Hz | ≈0.2–0.4 MB/s | RELIABLE / keep100 | costmap、RViz |
| A→B | /livox/imu | Imu | 200 Hz | ≈60 KB/s | BEST_EFFORT / keep5 | gps_imu_fusion（remap /imu→/livox/imu） |
| A→B | /gps/data | NavSatFix | 30 Hz | ≈4 KB/s | BEST_EFFORT / keep5 | gps_imu_fusion |
| A→B | /clock | Clock | ≈200 Hz | 极小 | **BEST_EFFORT / keep1** | B 侧所有 use_sim_time 节点 |
| A→B | /tf_static、/robot_description | latched | 一次性 | 小 | TRANSIENT_LOCAL | RViz RobotModel |
| B→A | /cmd_vel | Twist | 10 Hz | 极小 | RELIABLE / keep1 | champ / hybrid_motion_controller |

- **不过网（B 机本地）**：/world_odom、/waypoint_goals、/nav_status、markers、/local_costmap/*。
- **服务 / action 跨机可用**（原生 DDS 特性）：execute_task_server 放在 B 机与导航同机，指控端可经网络直接调用；`/follow_path` action 也在 B 机内部闭环。
- **QoS 兼容性**：点云发布端 RELIABLE，costmap 以 BEST_EFFORT 订阅（允许：offered ≥ requested）；RViz 与 nav_core 均以 RELIABLE 订阅——这三处订阅者各自占用一份独立单播流量（见 §5）。

## 4. 关键设计决策

| 决策项 | 选择 | 理由 |
| --- | --- | --- |
| 通信机制 | **原生 DDS 共享域**（不做桥/隧道） | 零代码、零外部二进制；服务/action 天然跨机（zenoh 桥不支持）；`ros2 topic list` 直见对端拓扑，排障直觉强。代价：无话题白名单，带宽只由"谁订阅了"决定 |
| RMW | **CycloneDDS（rmw_cyclonedds_cpp）为主**，Fast DDS profile 备选 | 网卡 pin 一行配置；单播对端语义最简；大样本水位/重传队列可调；WiFi 场景社区验证充分。备选覆盖 B 机无 cyclonedds 包的情形 |
| 域 | `ROS_DOMAIN_ID=33`（沿用 A 机 ~/.bashrc:103 既有值）、`ROS_LOCALHOST_ONLY=0` | 两机一致即可，改动集中在 env 脚本，不动任何现有文件 |
| 发现 | `AllowMulticast=spdp`（组播仅参与者发现）+ **双向单播 Peers 兜底** | 不依赖 AP 的组播策略：即使组播被丢弃，单播 SPDP 仍能建立发现 |
| 组播被阻断变体 | `AllowMulticast=false` + Peers 增加**本机 IP** + `ParticipantIndex=auto` + `MaxAutoParticipantIndex=120` | 单播模式下三项缺一不可，否则本机多进程互相发现不了（CycloneDDS issue #104） |
| 接口选择 | `Interfaces` = 物理口 + `lo` | 排除 docker0/br-*（DOWN 网桥）与潜在 VPN TUN（历史上 Mihomo TUN 198.18.0.1 抢默认路由）；lo 保住本机进程间通信 |
| 大样本调优 | WhcHigh=8MB、MaxQueuedRexmitBytes=8MB、DefragReliableMaxSamples=32、SocketReceiveBufferSize≥16MB（配套 sysctl rmem_max） | 单帧 480 KB 大于 CycloneDDS 默认门槛（WhcHigh 500 kB、MaxQueuedRexmitBytes 512 kB），不调会导致写者卡顿/重传受限；Ubuntu 默认 rmem_max≈208 KB 必须调大 |
| 启动约束 | B 机 `use_sim_time:=true`；保持 `simulation:=true`（默认） | 仿真时钟经 /clock 跨网；静态 TF base_link→livox 由 A 机 robot_state_publisher 提供，避免重复 TF |
| 已知偏差（不改） | use_sim_time 三处不一致、nav_waypoint_node 未收 use_sim_time、RViz 陈旧显示项（/local_plan、/camera/image_raw） | 属既有单机行为，与跨机无关；改动收益低、风险高，记录备查 |
| 与 zenoh 方案关系 | 不采用、不合并（`.worktree/zenoh-bridge/` 保留原样） | 用户决定暂不处理；原生 DDS 满足需求，zenoh 仅作最终回退项 |

## 5. 带宽模型与判定阈值

- 跨机总量 ≈ 小流合计（0.4–0.6 MB/s）+ **4.8 MB/s × N**，N = B 机点云订阅进程数（单播模式下每个订阅进程各占一份流量）。
- **基线 N=3**（costmap + RViz + nav_core 空回调）≈ **14–15 MB/s ≈ 115–125 Mbps**（再加 RTPS 分片开销约 5–8%）。
- 降载后 N=1（仅 costmap）≈ 5 MB/s；点云完全不过网 ≈ 0.5 MB/s。
- **判定阈值（实测后）**：B 侧聚合跨机流量 ≤ 链路实测容量（iperf3）的 50%，且连续 5 分钟 `/clock`、`/tf`、`/livox/lidar` 的 hz 最小值 ≥ 标称 95%、无线重传率（iw station dump）< 10%。达标则维持基线；不达标按 §8 降载。

## 6. 风险与对策

| 风险 | 现象 | 对策 |
| --- | --- | --- |
| AP 阻断/劣化组播 | 两机互相看不到节点 | spdp + 单播 Peers 双向兜底；完全阻断 → §4 变体配置 |
| docker 网桥 / VPN TUN 干扰 | 发现慢或不通 | Interfaces pin 物理口（DDS 直接绑口，不经默认路由）；check_link.sh 用 `ip route get` 校验 |
| 两机 RMW 不一致 | 单向/完全不可见且无报错 | env 脚本硬编码同一 RMW；发现探针数据带 RMW 名比对 |
| rmem_max 未调大 | 点云偶发丢包、重传放大 | 前置 sysctl + check_link.sh 校验 |
| RViz/nav_core 的 Reliable 点云订阅 | WiFi 重传压力、hz 抖动 | 阶段二 L1/L2（与算法零语义关系：一个只是显示、一个是空回调） |
| /tf 200 Hz 抖动 | B 侧偶发 TF extrapolation 告警 | costmap transform_tolerance 有余量；必要时 L5 降频 |
| /clock 丢包 | 仿真时间小幅跳跃 | 已核实为 best-effort keep1，跳变由 ROS time 正常处理，非致命 |
| 2.4 GHz 带宽不足 | 点云 hz 掉、整体抖动 | 强制 5 GHz；或 L1+L2+L4 |
| B 机无 cyclonedds apt 候选 | 无法装主方案 | 落后备 Fast DDS profile |
| 最终回退 | 以上均不达标 | 方案纯环境变量注入，恢复原环境变量即可完全回退；zenoh worktree 作为最后手段另行处理 |

## 7. 验证清单

| # | 机器 | 命令 | 判定 |
| --- | --- | --- | --- |
| 1 | 两机 | source env 脚本 | 摘要 RMW 相同、域=33、网卡/IP 正确 |
| 2 | B | `./net/check_link.sh` + 双向发现探针 | 发现层双向可见（探针带对端 RMW 名比对） |
| 3 | B | `ros2 topic list` / `ros2 node list` | 见 A 机全部跨机话题与节点 |
| 4 | B | `timeout 20 ros2 topic hz /clock` | ≥190 Hz，60 s 内无 >0.5 s 空档 |
| 5 | B | hz 探 /tf≈200、/livox/imu≈200、/gps/data≈30、/livox/lidar≈10 | 均 ≥标称 95% |
| 6 | A/B | A 本地 `ros2 topic bw /livox/lidar`；B 侧 /proc/net/dev 聚合；iperf3 测容量 | 单流≈4.8 MB/s；聚合÷单流=N；聚合 ≤ 容量 50% |
| 7 | B | `python3 test_execute_task_py.py`（先起 execute_task_server） | 4 用例 success |
| 8 | A | `ros2 topic echo /cmd_vel` | ≈10 Hz 非零（B→A 方向通） |
| 9 | 目视 | Gazebo 机器狗沿航点运动；RViz 见模型/路径/costmap；`tf2_echo world base_link` 持续输出 | 闭环成立 |
| 10 | B | 连续 5 min 观测 + `iw dev <wifi> station dump` | hz 最小值 ≥95%、重传率 <10% |
| 11 | A | `pkill -f gzserver` 后重启仿真 | B 侧自动恢复，无需重启 B |

反向对照（一次）：A 机不 source env 时 B 应看不到 A 的话题（证明隔离生效、无漏配）。

## 8. 分阶段实施与降载档

**阶段一（实施主体，零现有代码改动）**：net/ 配置与脚本、两机前置条件、双机联调、基线实测与记录。详见实现文档。

**阶段二（延后，仅实测不足时执行）**：

| 档 | 改动点 | 预计节省 | 涉及既有文件 |
| --- | --- | --- | --- |
| L1 | RViz 关点云显示并改 Best Effort（`nav_visualization.rviz:517/534/535` 已核实：Enabled=true、Reliable、/livox/lidar）；推荐新增 `net/rviz_nav_slim.rviz` + launch 加 `use_rviz`/`rviz_config` 参数 | −4.8 MB/s 且消除 Reliable 重传风险 | 是（1 文件 3 行 / 或新增文件+launch） |
| L2 | remap nav_core 空订阅（`nav_core.cpp:84-86` 订阅、`:156-160` 空回调）：`waypoint_nav.launch.py:81-83` 注释位加 `('/livox/lidar', '/livox/lidar_unused')`，并加 launch 参数（默认保持现行为） | −4.8 MB/s | 是（launch 数行） |
| L3 | `ros2 param set /nav_waypoint_node use_controller_server false`（动态回调已核实：`nav_core.cpp:52-68`）+ 停 controller_server 进程 → 点云完全不过网（失去避障） | 省 N×4.8 MB/s | 否（运行期操作） |
| L4 | `mid360.xacro:140` downsample 1→2 或 4 | −2.4/−3.6 MB/s 每流，免重编（symlink-install） | 是（xacro 1 行） |
| L5 | `description.launch.py:31` publish_frequency 200.0→50.0 | −0.2~0.3 MB/s 且显著降抖动 | 是（launch 1 行） |

判定树：聚合 ≤ 容量 50% 且 hz 稳定 → 不动；超过 → L1+L2；仍紧 → L4；以抖动为主 → L5；2.4 GHz 环境建议阶段一即默认 L1+L2。

## 9. 未核实项与核验方式

1. **CycloneDDS 0.10.5 配置键**（WhcHigh / WhcLow / MaxQueuedRexmitBytes / DefragReliableMaxSamples / FragmentSize / MaxMessageSize 默认值 / Tracing 是否接受 `Verbosity=config`）：初版不写未核实键；A 机单机回归时开启 Tracing 打印生效配置核验后定稿。
2. **Fast DDS 2.6 XML 细节**（userTransports/useBuiltinTransports 位置、LARGE_DATA 模式支持情况）：仅备选，核验优先级低。
3. **B 机环境**：Ubuntu 版本、`apt-cache policy ros-humble-rmw-cyclonedds-cpp` 是否有候选、WiFi 网卡名与 IP（配置中占位，check_link.sh 强制校验替换）。
4. 初版不写入的调参项（实测再定）：NackDelay、WhcAdaptive、显式 MaxMessageSize。

## 10. 证据链（file:line 索引）

- `src/robotdog_nav/waypoint_nav/src/nav_core.cpp:84-86`：/livox/lidar 订阅（RELIABLE 默认档）；`:156-160`：回调为空实现；`:52-68`：use_controller_server 动态参数回调
- `src/robotdog_nav/waypoint_nav/config/params.yaml:68`：costmap 观测话题 /livox/lidar；`:89`：odom_topic /world_odom；`:48/:88/:130`：use_sim_time 不一致
- `src/robotdog_nav/waypoint_nav/launch/waypoint_nav.launch.py:47-57`：RViz 无条件下启动且配置路径硬编码；`:69-84`：nav 节点参数为 share 硬编码路径（params_file 参数对其无效）；`:81-83`：remap 注释位；`:98-107`：静态 TF 仅 simulation:=false 时启动
- `src/robotdog_nav/gps_imu_fusion/launch/gps_imu_fusion.launch.py:34`：remap /imu→/livox/imu
- `src/gazebo_quadbot/robot_scene/xacro/mid360.xacro:9`：point_rate 200000；`:138`：samples=point_rate/update_rate=20000；`:140`：downsample；`:7`：laser_max_range 5.0
- `src/gazebo_quadbot/champ/champ_description/launch/description.launch.py:31`：publish_frequency 200.0（/tf 频率来源）
- `src/robotdog_nav/waypoint_nav/config/nav_visualization.rviz:517/534/535`：点云 Enabled=true / Reliability=Reliable / /livox/lidar
- `/opt/ros/humble/include/rclcpp/rclcpp/qos.hpp:340-357`：ClockQoS = Best effort / KeepLast(1)
- `.gitignore:2`：`devel/` 规则会吞掉 `docs/devel/`，需改 `/devel/` 方可在 git 中可见
- 环境：`~/.bashrc:103` 导出 ROS_DOMAIN_ID=33；本机 `ip -br addr` 仅 enp2s0=10.88.103.177/24（无 WiFi 网卡），docker0/br-* 均为 DOWN

## 11. 参考链接

- [CycloneDDS Configuration guide (0.10.5)](https://cyclonedds.io/docs/cyclonedds/0.10.5/config/cyclonedds_specifics.html)
- [CycloneDDS config_file_reference](https://cyclonedds.io/docs/cyclonedds/latest/config/config_file_reference.html)
- [CycloneDDS issue #104：单播发现的 loopback/participant-index 前提](https://github.com/eclipse-cyclonedds/cyclonedds/issues/104)
- [Fast DDS 2.6.12 Transport descriptors](https://fast-dds.docs.eprosima.com/en/v2.6.12/fastdds/xml_configuration/transports.html)
- [rmw_fastrtps issue #862：Humble 使用 FASTRTPS_DEFAULT_PROFILES_FILE](https://github.com/ros2/rmw_fastrtps/issues/862)
- [ROS 2 Humble FastDDS-Configuration 教程](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Advanced/FastDDS-Configuration.rst)
