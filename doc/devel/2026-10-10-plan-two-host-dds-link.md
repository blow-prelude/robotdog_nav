# 实现计划：双机通信层（原生 DDS，纯配置）

- 日期：2026-10-10
- 设计文档：[2026-10-10-design-two-host-dds-link.md](2026-10-10-design-two-host-dds-link.md)
- 实施位置：本仓库主检出（**纯新增文件** + .gitignore / AGENTS.md 各 1 行修改；不触碰任何现有代码）
- 状态：待实施

## 1. 文件清单

| 文件 | 动作 | 职责 |
| --- | --- | --- |
| net/cyclonedds_sim.xml | 新增 | A 机 CycloneDDS profile：Interfaces=enp2s0+lo；AllowMulticast=spdp；Peers 含 B 机 IP（占位 192.168.1.100）；ParticipantIndex=auto、MaxAutoParticipantIndex=120；大样本调优（WhcHigh/MaxQueuedRexmitBytes/Defrag/缓冲） |
| net/cyclonedds_nav.xml | 新增 | B 机同构 profile：Interfaces=B 机 WiFi 网卡名（占位）+lo；Peers=10.88.103.177 |
| net/fastdds_sim.xml | 新增 | A 机 Fast DDS 备选 profile（B 机无 cyclonedds 候选时启用）：UDPv4 transport（interfaceWhiteList 含 127.0.0.1、8MB/16MB 缓冲、non_blocking_send）+ initialPeersList + 关闭组播 locator |
| net/fastdds_nav.xml | 新增 | B 机 Fast DDS 备选 profile（同构） |
| net/env_sim.sh | 新增 | A 机环境：`source` 即导出 ROS_DOMAIN_ID=33 / ROS_LOCALHOST_ONLY=0 / RMW_IMPLEMENTATION=rmw_cyclonedds_cpp / CYCLONEDDS_URI，并打印生效摘要（RMW/域/网卡/IP/对端/出口路由）；支持 `NET_RMW=fastdds` 切换备选 |
| net/env_nav.sh | 新增 | B 机环境（同构，指向 cyclonedds_nav.xml；自动探测 WiFi 网卡） |
| net/check_link.sh | 新增 | 一键诊断：接口/IP/路由（TUN 抢路由检测）、ping+MTU、DDS 端口（域 33→15650 起）、sysctl 缓冲校验、组播探针、双向发现探针（`ros2 topic pub -1` ↔ `echo --once`，数据带 RMW 名）、hz/bw 采样、XML 占位符未替换报警、iperf3 提示 |
| net/README.md | 新增 | 中文使用指南：拓扑/前置/启动顺序/验证/带宽测量/组播阻断变体/排障速查 |
| .gitignore | 修改 1 行 | `devel/` → `/devel/`（否则 docs/devel/ 被静默忽略） |
| AGENTS.md | 修改 1 行 | 关键目录表加 `net/` 一行 |

## 2. 两机前置条件（各执行一次）

| 项 | 命令 | 校验 |
| --- | --- | --- |
| DDS 包 | `sudo apt install ros-humble-rmw-cyclonedds-cpp`（B 机先 `apt-cache policy ros-humble-rmw-cyclonedds-cpp` 确认有候选；无候选则改用 Fast DDS 备选：env 脚本 `NET_RMW=fastdds`） | `dpkg -l` 含 cyclonedds 条目 |
| 内核缓冲 | `echo -e "net.core.rmem_max=16777216\nnet.core.wmem_max=8388608" \| sudo tee /etc/sysctl.d/99-ros2-dds.conf && sudo sysctl --system` | `sysctl net.core.rmem_max net.core.wmem_max` ≥ 16MB/8MB |
| 测速工具 | `sudo apt install iperf3` | `command -v iperf3` |
| B 机工作区 | 本仓库已同步到 B 机（含 net/ 目录）；`colcon build --symlink-install --packages-up-to waypoint_nav execute_task_server gps_imu_fusion`（不需要 Gazebo 相关包） | `ros2 pkg executables waypoint_nav` |

XML 中两处占位符需替换：B 机实际 IP（`192.168.1.100`）与 B 机 WiFi 网卡名（`wlan0`）；`check_link.sh` 会强制校验。

## 3. 启动顺序（逐终端，命令可直接复制）

**A 机（仿真）**：

```bash
cd /home/ww/program/projects/robotdog_nav
source /opt/ros/humble/setup.bash && source install/setup.bash
source net/env_sim.sh
ros2 daemon stop          # 切换 RMW/域后清理 CLI 缓存（一次性）
ros2 launch robot_scene go2_lidar_gps.launch.py
```

**B 机（导航）终端 1**：

```bash
cd <B 机工作区路径>
source /opt/ros/humble/setup.bash && source install/setup.bash
source net/env_nav.sh
./net/check_link.sh                 # 先过诊断，再起导航
ros2 launch waypoint_nav waypoint_nav.launch.py use_sim_time:=true   # 保持 simulation:=true 默认
```

**B 机终端 2（任务服务，与导航同机）**：

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && source net/env_nav.sh
ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1
```

注：A 机不再启动 waypoint_nav / gps_imu_fusion / execute_task_server（全部移到 B）；两机每个终端都必须先 `source net/env_*.sh`。

## 4. 验证清单

- [ ] 1. 两机 env 摘要一致：RMW 相同、ROS_DOMAIN_ID=33、ROS_LOCALHOST_ONLY=0、网卡/IP 正确
- [ ] 2. B：`./net/check_link.sh` 全绿；双向发现探针互通（探针消息带对端 RMW 名，比对一致）
- [ ] 3. B：`ros2 topic list` / `ros2 node list` 可见 A 机全部跨机话题与节点（/clock /tf /tf_static /livox/lidar /livox/imu /gps/data /robot_description /cmd_vel）
- [ ] 4. B：`timeout 20 ros2 topic hz /clock` ≥190 Hz，60 s 内无 >0.5 s 空档（仿真时间不停走）
- [ ] 5. B：`ros2 topic hz` 探 /tf≈200、/livox/imu≈200、/gps/data≈30、/livox/lidar≈10，均 ≥标称 95%
- [ ] 6. 带宽：A 本地 `ros2 topic bw /livox/lidar` ≈4.8 MB/s；B 侧 `/proc/net/dev`（或 nload）聚合流量 ÷ 单流 = N（基线应为 3）；`iperf3 -u` 实测链路容量，聚合 ≤ 容量 50%
- [ ] 7. B：起 execute_task_server 后 `python3 test_execute_task_py.py` 4 用例返回 success
- [ ] 8. A：`ros2 topic echo /cmd_vel` ≈10 Hz 非零（B→A 方向通）
- [ ] 9. 闭环：Gazebo 中机器狗沿航点运动；RViz 见模型/路径/costmap；`ros2 run tf2_ros tf2_echo world base_link` 持续输出
- [ ] 10. 持续 5 min：三路 hz 最小值 ≥95%；`iw dev <wifi> station dump` 重传率 <10%
- [ ] 11. 恢复：A 机 `pkill -f gzserver` 后重启仿真，B 侧自动恢复（无需重启 B）
- [ ] 反向对照（一次）：A 机不 source env 时 B 应看不到 A 话题

判定：全过 → 维持基线（阶段一完成）；第 6/10 项不达标 → 按设计文档 §8 执行降载档（另开改动）。

## 5. 实施顺序与自测记录

- [x] 设计文档 docs/devel/2026-10-10-design-two-host-dds-link.md
- [x] 实现文档 docs/devel/2026-10-10-plan-two-host-dds-link.md（本文件）
- [x] .gitignore 修正（`devel/` → `/devel/`）
- [ ] AGENTS.md 关键目录表加 `net/` 一行
- [ ] net/ 八个文件落地（XML 先按设计文档初版，未核实键不写）
- [ ] A 机单机回归：source env_sim.sh → 起仿真 → 本机 hz 正常（pin 网卡未破坏本机发现）；Tracing 核验配置键无报错（把设计文档 §9 的"未核实"项转成已核实后定稿 XML）
- [ ] B 机前置 + check_link.sh 通过 + 占位符替换
- [ ] 双机联调：按 §4 清单 1–11 逐项执行并记录基线数据（N 值、聚合流量、iperf3 容量、重传率）
- [ ] 按阈值决定是否进入阶段二降载（L1/L2 优先）

## 6. 阶段二（延后，见设计文档 §8）

L1 RViz 点云显示（nav_visualization.rviz:517/534/535）· L2 remap nav_core 空订阅（nav_core.cpp:84-86 / :156-160，launch:81-83）· L3 运行时切纯跟踪（nav_core.cpp:52-68）· L4 mid360.xacro:140 downsample · L5 description.launch.py:31 tf 降频。触发条件与收益见设计文档 §8。
