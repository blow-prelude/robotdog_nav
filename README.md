# ROBOTDOG_NAV

> 💡 **快速开始**: 如果想快速测试功能，请查看 [QUICK_START.md](QUICK_START.md) - 提供了三种测试方案，从 1 分钟快速测试到完整仿真测试。

## 功能包介绍

### 公共类
- **global2local**: 无人机共用的全局坐标系到本地坐标系的转换包，暂未使用，最终方案不一定接 PX4 的融合定位
- **msg_set_msgs**: 无人机共用的消息类型包，也包含导航相关的消息类型，如 MultiGoal，用于和指控端话题对齐

### 基础算法组件类
- **gps_imu_fusion**: 一个示例 GPS+IMU 融合定位包，后续需替换成最终融合定位方案
- **navigation2**: 基于 navigation2 的定制修改后导航组件，目前包含无图导航功能最小功能包集合，后续可能加上全局规划、行为树等组件

### 外层项目应用类
- **nav2_straight_planner**: 定点无图导航应用，可基于此进行课题 5 等开发
- **waypoint_nav**: 适配了 msg_set_msgs 中的 MultiGoal 消息类型输入，可基于此进行 ZNY 的 waypoint 多点巡航开发
- **execute_task_server**: 指控端接口服务，接收 JSON 格式任务请求，转换为 MultiGoal 消息发布给 waypoint_nav 执行

---

## 系统架构

```
指控端 → /uav1/swarms_backend/global/multitask (MultiTask)
           ↓
multi_task_bridge → 解析并转发
           ↓
/agent/uav1/execute_task (ExecuteTask.srv)
           ↓
execute_task_server → 转换为 MultiGoal 消息
           ↓
/waypoint_goals (MultiGoal)
           ↓
waypoint_nav → 执行多点导航
```

---

## 测试命令

### 方式一：直接发布 MultiGoal 话题（快速测试）

适用于直接测试 waypoint_nav 导航功能。

> **前提**: 需要先构建仿真环境包。如果未构建，请先执行：
> ```bash
> colcon build --packages-select robot_scene go2_description mid360_simulation --symlink-install
> ```

#### 终端 1：启动 Gazebo 仿真

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_scene go2_lidar_gps.launch.py
```

> **提示**: 如果 `robot_scene` 包未找到，请确保已构建 gazebo_quadbot 相关包。也可以直接使用 Gazebo 加载已有世界文件：
> ```bash
> gz sim -r /media/lenovo/1/project/gazebo/0713/robotdog_nav/src/gazebo_quadbot/robot_scene/worlds/your_world.sdf
> ```

#### 终端 2：启动导航节点

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./launch.sh dognav
```

#### 终端 3：发送航点目标

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub --once /waypoint_goals msg_set_msgs/msg/MultiGoal "{
  multi_goal_points: [
    {
      x_or_lat: 0.00012,
      y_or_lon: 0.00002,
      z_or_alt: 0.0,
      yaw: 0.0,
      vel: 1.0
    },
    {
      x_or_lat: -0.00015,
      y_or_lon: 0.00013,
      z_or_alt: 0.0,
      yaw: 1.0,
      vel: 0.8
    },
    {
      x_or_lat: -0.00005,
      y_or_lon: -0.00010,
      z_or_alt: 0.0,
      yaw: 1.5,
      vel: 0.8
    },
    {
      x_or_lat: 0.00008,
      y_or_lon: -0.00008,
      z_or_alt: 0.0,
      yaw: 0.5,
      vel: 1.0
    },
  ],
  is_gps_aid: true,
  is_gps_hgt: false,
  ctl_mode: 0
}"
```

---

### 方式二：通过指控端接口服务（完整链路测试）

适用于测试从指控端到导航执行的完整链路。

> **前提**: 需要先构建仿真环境包和相关导航包：
> ```bash
> colcon build --packages-select robot_scene go2_description mid360_simulation waypoint_nav gps_imu_fusion execute_task_server --symlink-install
> ```

#### 启动 4 个终端

**终端 1：启动 Gazebo 仿真**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_scene go2_lidar_gps.launch.py
```

**终端 2：启动导航节点**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./launch.sh dognav
```

**终端 3：启动指控端接口服务**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./launch_service.sh taskserver
```

**终端 4：运行测试脚本**
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 test_execute_task_py.py
```

> **提示**: 
> - 每个终端都需要先执行 `source` 命令激活环境
> - 等待每个节点启动完成后再启动下一个（观察日志输出）
> - 测试完成后，可在 RViz 中查看航点标记和规划路径

#### 测试说明

测试脚本会执行以下 4 个测试用例：

1. **局部坐标航点** (3 个点) - 验证局部坐标系导航
2. **GPS 坐标航点** (4 个点) - 验证 GPS 坐标转换和导航
3. **错误的 agent ID** - 验证服务拒绝非本机任务
4. **无效的能力** - 验证服务拒绝不支持的任务类型

所有测试通过后，机器狗应该完成所有航点的跟踪，RViz 中会显示：
- 绿色球体：当前目标航点
- 蓝色球体：后续航点
- 绿色路径线：规划的路径
- 实际轨迹：机器狗的运动轨迹

---

### 方式三：与 multi_task_bridge 集成（真实场景）

适用于完整的指控端系统集成测试。

> **前提**: 需要构建所有相关包：
> ```bash
> # 构建 robotdog_nav
> colcon build --packages-select robot_scene go2_description mid360_simulation waypoint_nav gps_imu_fusion execute_task_server --symlink-install
> 
> # 构建 multi_task_bridge
> cd /media/lenovo/1/project/gazebo/multi_task_bridge
> colcon build --symlink-install
> ```

#### 终端 1：启动仿真和导航

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动 Gazebo
ros2 launch robot_scene go2_lidar_gps.launch.py &

# 启动定位
ros2 launch gps_imu_fusion gps_imu_fusion.launch.py use_sim_time:=true &

# 启动导航
ros2 launch waypoint_nav waypoint_nav.launch.py use_sim_time:=true &

# 启动执行服务
ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1
```

#### 终端 2：启动 multi_task_bridge

```bash
source /opt/ros/humble/setup.bash
cd /media/lenovo/1/project/gazebo/multi_task_bridge
source install/setup.bash

# 启动桥接节点
ros2 launch multi_task_bridge multi_task_bridge.launch.py drone_name:=uav1
```

#### 终端 3：从指控端发送 MultiTask 消息

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 发布 MultiTask 消息
ros2 topic pub --once /uav1/swarms_backend/global/multitask msg_set_msgs/msg/MultiTask "{
  drones_id: [1],
  task_stream: [{
    task_type: 7,  # LINE_TASK
    task_area: {
      type: 0,  # POINT
      data: [30.06888, 120.04646]
    }
  }]
}"
```

---

## 构建命令

### 快速构建（仅测试 execute_task_server）

```bash
cd /media/lenovo/1/project/gazebo/0713/robotdog_nav
colcon build --packages-select execute_task_server msg_set_msgs --symlink-install
```

### 标准构建（测试导航链路）

```bash
colcon build --packages-select waypoint_nav gps_imu_fusion execute_task_server msg_set_msgs --symlink-install
```

### 完整构建（包含仿真环境）

```bash
# 首次构建需要 20-30 分钟
colcon build --symlink-install
```

> **注意**: 
> - 如果首次构建失败，请清理构建缓存后重试：
>   ```bash
>   rm -rf build/ install/ log/
>   colcon build --symlink-install
>   ```
> - 详细构建说明请参考 [QUICK_START.md](QUICK_START.md)

---

## 快速启动

### 启动仿真环境

**方法 1：使用快速启动脚本（推荐，自动构建依赖）**

```bash
./start_simulation.sh
```

**方法 2：手动构建并启动**

```bash
# 首次需要构建所有包
cd /media/lenovo/1/project/gazebo/0713/robotdog_nav
rm -rf build/ install/ log/  # 清理旧构建
colcon build --symlink-install

# 启动仿真
source install/setup.bash
ros2 launch robot_scene go2_lidar_gps.launch.py
```

**方法 3：不使用仿真（仅算法测试）**

如果不想启动 Gazebo 仿真，可以直接测试导航算法：

```bash
# 构建导航包
colcon build --packages-select waypoint_nav gps_imu_fusion execute_task_server --symlink-install

# 启动导航
source install/setup.bash
ros2 launch waypoint_nav waypoint_nav.launch.py
```

### 启动导航系统

```bash
# 使用 launch.sh 脚本（原始版本）
./launch.sh dognav

# 或使用 launch_service.sh（增强版本）
./launch_service.sh full

# 或单独启动
source install/setup.bash
ros2 launch waypoint_nav waypoint_nav.launch.py use_sim_time:=true
```

### 发送航点指令

```bash
# 方法 1：直接发布话题
source install/setup.bash
ros2 topic pub --once /waypoint_goals msg_set_msgs/msg/MultiGoal "{...}"

# 方法 2：通过服务接口（使用 launch_service.sh）
./launch_service.sh taskserver  # 启动服务节点
python3 test_execute_task_py.py  # 运行测试

# 方法 3：使用快速测试脚本
./quick_test.sh
```

---

## 依赖安装

```bash
# nlohmann-json (用于 JSON 解析)
sudo apt install nlohmann-json3-dev

# 如果缺少 multi_task_bridge，需先克隆并编译
cd /media/lenovo/1/project/gazebo
git clone <multi_task_bridge_repo_url>
cd multi_task_bridge
colcon build --symlink-install
```

---

## 监控和调试

### 查看节点列表

```bash
ros2 node list
```

### 查看话题连接

```bash
# 查看所有话题
ros2 topic list

# 查看话题信息
ros2 topic info /waypoint_goals --verbose

# 监听话题消息
ros2 topic echo /waypoint_goals
```

### 查看服务

```bash
# 查看所有服务
ros2 service list

# 查看服务类型
ros2 service type /agent/uav1/execute_task
```

### 查看日志

```bash
# 查看节点日志
ros2 topic echo /rosout --filter 'node_name=="execute_task_node"'
ros2 topic echo /rosout --filter 'node_name=="nav_waypoint_node"'
```

### 参数配置

```bash
# 查看参数
ros2 param dump /nav_waypoint_node

# 动态调整参数（切换到纯追踪模式）
ros2 param set /nav_waypoint_node use_controller_server false

# 动态调整参数（切换到外部控制器模式）
ros2 param set /nav_waypoint_node use_controller_server true
```

---

## 相关文档

- [execute_task_server 详细文档](src/robotdog_nav/execute_task_server/README.md)
- [waypoint_nav 详细文档](src/robotdog_nav/waypoint_nav/README.md)
- [multi_task_bridge 接口说明](/media/lenovo/1/project/gazebo/multi_task_bridge/接口软件包介绍.md)
- [测试报告](TEST_REPORT.md)
- [快速开始](QUICK_START.md)
- [常见问题 (FAQ)](doc/FAQ.md)
- [Gazebo ROS 插件安装](doc/install_gazebo_ros.md)
- [Gazebo 启动问题](doc/gazebo_launch_issue.md)
- [机器狗运动问题排查](doc/robot_moving_issue.md)
- [机器狗姿态问题](doc/robot_pose_issue.md)
- [launch_service 使用说明](doc/launch_service.md)

---

## 常见问题

### Q: Gazebo 启动失败，gzserver 崩溃

**现象**:
```
[ERROR] [gzserver]: process has died [exit code 255]
[ERROR] [spawn_entity]: Service /spawn_entity unavailable
```

**原因**: Gazebo ROS 插件缺失 (`libgazebo_ros_init.so`)

**解决**:
```bash
# 方法 1：运行安装脚本（需要 sudo）
sudo ./install_gazebo_ros.sh

# 方法 2：手动安装
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev

# 方法 3：使用简化启动（不需要该插件）
ros2 launch robot_scene go2_description.launch.py
```

详细文档：[doc/install_gazebo_ros.md](doc/install_gazebo_ros.md)

### Q: 机器狗在 Gazebo 中直立且静止不动

**原因**: `launch.sh dognav` 只启动导航节点，不直接控制关节。需要完整的控制链路。

**解决**:
```bash
# 1. 启动 Gazebo
ros2 launch robot_scene go2_lidar_gps.launch.py

# 2. 键盘控制测试
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 3. 或启动完整导航
ros2 launch gps_imu_fusion gps_imu_fusion.launch.py use_sim_time:=true &
./launch_service.sh dognav
python3 test_execute_task_py.py
```

详细文档：[doc/robot_pose_issue.md](doc/robot_pose_issue.md)

### Q: 机器狗持续运动不停

**原因**: 闭环里程计启用导致机器狗不断"纠正"位置。

**解决**:
```bash
# 使用静态版本（禁用闭环里程计）
ros2 launch robot_scene go2_lidar_gps_static.launch.py
```

详细文档：[doc/robot_moving_issue.md](doc/robot_moving_issue.md)

### Q: 如何测试 execute_task_server

**答案**: 不需要 Gazebo，直接运行：
```bash
colcon build --packages-select execute_task_server msg_set_msgs
ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1
python3 test_execute_task_py.py
```

### Q: 服务不可用

**A:** 确保 execute_task_server 已启动：
```bash
ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1
```

### Q: 航点不执行

**A:** 检查是否有 `/world_odom` 里程计数据：
```bash
ros2 topic echo /world_odom
```

### Q: 坐标转换问题

**A:** GPS 坐标会自动转换为 ENU 局部坐标。使用 `is_gps_aid: true` 表示 GPS 坐标，`is_gps_aid: false` 表示局部坐标。
