# 局域网多机 ROS 域冲突导致 /clock 错乱（时间倒退）

> 2026-10-10 排查记录，Jetson NX 环境

## 问题现象

1. 启动仿真（`./start_simulation.sh` 或 `ros2 launch robot_scene go2_lidar_gps.launch.py`）后，`robot_state_publisher` 持续刷警告：
   ```
   [robot_state_publisher-1] [WARN]: Moved backwards in time, re-publishing joint transforms!
   ```
2. 伴随的诡异现象（同根因）：
   - `ros2 node list` 输出为空，或 `ros2 topic info -v` 显示节点名为 `_NODE_NAME_UNKNOWN_`
   - 本机**没有运行任何 Gazebo**，`ros2 topic echo /clock` 却能收到数据
   - 多个不同时基的 /clock 数据流交错到达（话题速率远超单个发布者）

## 原因分析

本工作空间的 DDS 默认使用 **ROS_DOMAIN_ID=0**，而 DDS 多播会**穿透整个局域网**。当局域网内另一台机器（同样使用默认域 0）正在运行 Gazebo 仿真时：

```
Jetson 本地 gzserver      → /clock（时基从 0 开始，如 5s、6s、7s…）
局域网远端 gzserver       → /clock（时基已运行数小时，如 10152s…）
        ↓ 两路数据交错进入同一个 /clock 话题
robot_state_publisher 收到的时间反复从"未来"跳回"过去"
        ↓
Moved backwards in time 警告刷屏；依赖 sim time 的节点（TF、控制器、导航）全部紊乱
```

关键点：
- **本机不跑仿真时一切正常**（只有远端一路时钟，单调递增），所以该问题容易被误判为偶发；
- 其他现象同理可解释：远端 DDS 参与者会混入本机节点表，造成节点名无法解析、`ros2 node list` 异常等。

## 排查方法

**决定性测试**——对比「含局域网」和「仅本机」两种模式下的 /clock 接收情况：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 默认模式（含局域网多播）
timeout 4 ros2 topic echo /clock --field clock > /tmp/c1.txt 2>&1
grep -c 'sec:' /tmp/c1.txt        # 实测 5366 条（本机无 Gazebo 时仍收到！）

# 仅本机回环
ROS_LOCALHOST_ONLY=1 timeout 4 ros2 topic echo /clock --field clock > /tmp/c2.txt 2>&1
grep -c 'sec:' /tmp/c2.txt        # 实测 0 条 → 证明发布者在局域网其他机器上
```

辅助手段：
- `ros2 topic info /clock -v`：发布者/订阅者数量与节点名解析情况
- `ps aux | grep gzserver`：确认本机确实没有 Gazebo 进程
- `tcpdump -ni eth0 udp`：抓 DDS 多播包来源 IP（需要 root 权限）

## 解决方案：ROS_DOMAIN_ID 域隔离

本机固定使用独立的域 ID（本项目选定 **42**），与局域网默认域 0 的设备互不可见：

已在以下位置配置（内容相同）：
- `~/.bashrc`（覆盖所有交互终端）
- `start_simulation.sh`、`launch.sh`、`launch_service.sh`（脚本自包含，保护非交互场景）

```bash
export ROS_DOMAIN_ID=42
```

### 注意事项

1. **已开着的终端不会自动生效**，需重新打开终端或手动 `export ROS_DOMAIN_ID=42`；旧的 ros2 daemon 缓存了域 0 的图信息，建议执行一次 `ros2 daemon stop`（新命令会按新域自动拉起 daemon）。
2. **跨机 ROS 通信的对端必须使用相同域 ID**：若指控端/`multi_task_bridge` 需要跨机下发任务到本机的 `execute_task_server`，对端机器需同样 `export ROS_DOMAIN_ID=42`（或两端约定统一值）。
3. 验证方法：配置后在新终端执行 `ros2 topic echo /clock`，本机未跑仿真时应收不到任何数据（远端域 0 的时钟已不可见）。
4. 域 ID 合法范围 0-101（0-232 理论可用但部分 RMW 有限制）；同一局域网内建议按机器/项目分配不同域，避免再次撞车。
