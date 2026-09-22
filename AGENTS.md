# Repository Guidelines（仓库指南）

本仓库是一个 ROS 2 Humble 工作空间，用于宇树 Go2/Go2W 四足机器人航点导航：指控端通过 ROS 服务下发 JSON 任务，自研无图导航器将其转换为 GPS/ENU 航点并执行，由 Gazebo Classic 11 四足仿真闭环验证。AI 助手注意：假定遵循 colcon/ament 规范，节点为 C++14，使用 ROS launch 文件，注释与文档均为中文。

## 项目概述

- 面向四足机器人（宇树 Go2 足式版 / Go2W 轮足版）的航点导航栈，由集群指控端（fleet command-and-control）驱动。
- 任务以 JSON 形式经 ROS 服务进入，转换为 `MultiGoal` 消息，经 Nav2 直线控制器或内置纯跟踪（pure-pursuit）执行，并通过仿真 GPS + Livox Mid-360 激光/IMU 传感器套件闭环。
- 同时支持实机部署（`doc/go2w_real_deployment.md`）：导航栈不变，传感器经 Unitree SDK / Livox ROS2 SDK / nmea_navsat_driver 接入，`simulation:=false`。

## 架构与数据流

三层结构。完整链路：

```
指控端
  → /uav1/swarms_backend/global/multitask (MultiTask)          [外部 multi_task_bridge 工作空间]
  → /agent/uav1/execute_task (ExecuteTask.srv, JSON)           [execute_task_server]
  → /waypoint_goals (msg_set_msgs/MultiGoal)                   [execute_task_node]
  → FSM + GPS→ENU 转换 + FollowPath action / 纯跟踪            [waypoint_nav nav_waypoint_node]
  → /cmd_vel                                                   [nav2 controller_server 或 waypoint_nav]
  → 腿部 /cmd_vel/smooth + 轮毂 /joint_group_velocity_controller/commands  [hybrid_motion_controller]
  → champ quadruped_controller_node + ros2_control → Gazebo
  → 传感器 /gps/data, /livox/imu, /livox/lidar
  → /world_odom + TF world→base_link                           [gps_imu_fusion]
  → 回流至 nav_waypoint_node（10 Hz 闭环）
```

- **任务层** — `execute_task_server`：服务 `/agent/<agent_id>/execute_task`（`string request_json → string response_json`），能力 ID 为 `goto_waypoints`/`takeoff`/`return`；校验 agent ID 与能力；`is_gps_aid: true` 表示航点字段为经纬度，`false` 为局部坐标 x/y。
- **导航层** — `waypoint_nav`：10 Hz 有限状态机（`IDLE → WAITING_FOR_GOALS → INITIALIZING → EXECUTING_PATH → GOAL_REACHED/FAILED/COMPLETED`，见 `include/waypoint_nav/nav_fsm.hpp`）。GPS→ENU 经 GeographicLib `LocalCartesian`（原点来自参数或 `/mavros/global_position/gp_origin`）。`use_controller_server: true` 时生成直线路径（≤10 个插值点），经 `nav2_msgs/action/FollowPath` 发给内置的 Nav2 `controller_server`（RegulatedPurePursuitController）；`false` 时自行计算纯跟踪 Twist。Nav2 代价地图在 `world` 坐标系下消费 `/livox/lidar`。
- **定位层** — `gps_imu_fusion`：融合 `/gps/data` + `/livox/imu` → `/world_odom` @10 Hz + TF `world→base_link`。注意 TF 分工：`world→base_link`（融合定位）与 `odom→base_footprint`（Gazebo 里程计插件）并存；导航配置使用的坐标系为 `world`/`base_link`。
- **仿真层** — `gazebo_quadbot`：Gazebo Classic 11、champ 步态控制器、ros2_control（腿部 effort、轮毂 velocity）、自研 Livox Mid-360 Gazebo 插件。
- 两个仿真入口：`go2_lidar_gps.launch.py`（足式 Go2，多数文档/测试使用）与 `go2w_lidar_gps.launch.py`（轮足 Go2W）。若 `go2_lidar_gps.launch.py` 下机器人漂移（champ `close_loop_odom: true` 所致），改用 `go2_lidar_gps_static.launch.py`。

## 关键目录

```
src/robotdog_nav/            # 应用层 + 算法层软件包（ament_cmake，C++）
  execute_task_server/       # JSON 任务服务 → MultiGoal（nlohmann_json）
  waypoint_nav/              # nav_waypoint_node、nav_core、nav_fsm、params.yaml、launch/
  gps_imu_fusion/            # GPS+IMU → /world_odom、TF world→base_link
  msg_set_msgs/              # 约 60 个 rosidl 消息类型（MultiGoal、MultiGoalPoint、MultiTask、…）
  nav2_straight_planner/     # 独立 lifecycle 直线规划器（未接入任何 launch）
  navigation2/               # 内嵌（vendored）Nav2 子集（含自定义 theta_star.cpp）——勿直接换成上游版本
  global2local/              # 遗留 MAVROS 无人机节点 —— 已加 COLCON_IGNORE，不参与构建
src/gazebo_quadbot/          # 仿真环境（fork 自 dfl-rlab gazebo_go2_simulation）
  robot_scene/               # xacro 模型（go2w_description.urdf.xacro、gps.xacro、mid360.xacro）、
                             #   worlds/（约 23 个，默认 agriculture.world）、maps/、7 个 launch 文件
  champ/                     # 内嵌 CHAMP 四足框架（champ_base 节点、bringup）
  robots/go2w_control/       # hybrid_motion_controller.py：将 /cmd_vel 拆分为腿部 + 轮毂指令
  robots/mid360_simulation/  # Livox Mid-360 Gazebo 插件（C++）、扫描模式 CSV
  robots/go2_description/ go2w_description/ go2_config/ go2w_config/
doc/                         # 故障排查笔记（中文）：Gazebo 问题、位姿/漂移、实机部署
.devcontainer/               # GPU devcontainer 配置 + setup_gpu.sh
```

## 开发命令

每个终端必须先 source 环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

构建（仓库根目录执行；`build/ install/ log/` 为 gitignore 的 colcon 输出）：

```bash
./build.sh                                   # colcon build --symlink-install --parallel-workers $(nproc) \
                                             #   --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --packages-select execute_task_server msg_set_msgs --symlink-install   # 最快迭代
colcon build --symlink-install               # 全量构建，首次约 20-30 分钟
rm -rf build/ install/ log/ && ./build.sh    # 构建异常时清理后重建
```

运行（典型全链路流程）：

```bash
./start_simulation.sh                        # 缺包时自动构建 champ/go2/go2w/mid360/robot_scene，并启动仿真
ros2 launch robot_scene go2_lidar_gps.launch.py   # 或 go2w_lidar_gps.launch.py
./launch.sh dognav                           # waypoint_nav + gps_imu_fusion + RViz（use_sim_time:=true）
./launch_service.sh taskserver               # execute_task_server（agent_id:=uav1）
./launch_service.sh full                     # 三个组件在同一终端后台/前台一并启动
python3 test_execute_task_py.py              # 4 用例端到端测试
./stop_robot.sh                              # 向 /cmd_vel 发布零 Twist
./stop_robot_gazebo.sh                       # gz service set_pose / world reset
./fix_gazebo.sh                              # Gazebo 诊断 + 缓存清理
```

调试开关：

```bash
ros2 param set /nav_waypoint_node use_controller_server false   # 切换为内置纯跟踪
ros2 topic echo /nav_status                                     # FSM 状态
ros2 topic info /waypoint_goals --verbose                       # 校验发布/订阅接线
ros2 control list_controllers                                   # controller_manager 状态
ros2 run teleop_twist_keyboard teleop_twist_keyboard            # 手动遥控
```

## 代码规范与常见模式

- **软件包均为 ament_cmake（format 3）**；全仓库无 `setup.py`。Python 节点（如 `go2w_control/src/*.py`）经 CMake `install(PROGRAMS ...)` 以可执行程序方式安装。
- **C++14** 编写节点（见 `.vscode/c_cpp_properties.json`），C 为 GNU11。关键依赖：rclcpp、nav2 msgs/util、GeographicLib、nlohmann_json（需 `libnlohmann-json3-dev`）、Eigen3、yaml-cpp、mavros_msgs（仅遗留代码）。
- **节点接线模式**：每个节点包包含 `src/<node>.cpp`、`include/<pkg>/…`、`config/params.yaml`（ROS 参数，禁止硬编码）、`launch/<pkg>.launch.py`，并在构造函数中声明参数。新增节点遵循此布局。
- **FSM 状态管理**：航点执行是手写枚举状态机（`nav_fsm.hpp`），由 10 Hz 定时回调驱动——不是行为树。状态迁移保持在 FSM 类内部。
- **配置优先于代码**：可调项（goal_tolerance、linear_velocity、控制器选择）集中在 `config/params.yaml` / launch 参数；实机调参使用降速值（goal_tolerance 0.5、linear_velocity 0.5）——见 `doc/go2w_real_deployment.md`。
- **坐标系约定**：航点坐标系为 `gps`（经纬度，自动转 ENU）或局部 `world` ENU，按任务经 `is_gps_aid` 选择；yaw 单位弧度，每点 `vel` 单位 m/s。
- 注释、README 与 doc/ 均为中文，保持该风格。
- 内嵌代码（`navigation2/`、`champ/`）是带本地修改的上游源码——只做外科手术式修改，严禁从上游重新生成覆盖。

## 重要文件

- `src/robotdog_nav/execute_task_server/src/execute_task_node.cpp` — 任务服务入口，JSON 任务格式定义处
- `src/robotdog_nav/execute_task_server/srv/ExecuteTask.srv` — `request_json`/`response_json`
- `src/robotdog_nav/waypoint_nav/src/nav_core.cpp` + `include/waypoint_nav/nav_fsm.hpp` — 导航核心逻辑
- `src/robotdog_nav/waypoint_nav/config/params.yaml` — 导航 + Nav2 控制器/代价地图参数
- `src/robotdog_nav/gps_imu_fusion/config/params.yaml` — GPS 原点（`origin_lat/lon/alt`）、yaw 偏移
- `src/gazebo_quadbot/robot_scene/launch/go2_lidar_gps.launch.py` / `go2w_lidar_gps.launch.py` — 仿真 bringup 入口
- `src/gazebo_quadbot/robot_scene/xacro/go2w_description.urdf.xacro` — 机器人模型 + ros2_control + Gazebo 插件
- `src/gazebo_quadbot/robots/go2w_control/src/hybrid_motion_controller.py` — cmd_vel 腿/轮拆分
- `src/gazebo_quadbot/robots/mid360_simulation/src/mid360_points_plugin.cpp` — LiDAR 仿真插件
- `build.sh`、`start_simulation.sh`、`launch.sh`、`launch_service.sh` — 运维包装脚本
- `QUICK_START.md` — 三档测试方案（仅服务 → 无 Gazebo 导航 → 全量仿真）；`TEST_REPORT.md` — 测试通过记录

## 运行时/工具链偏好

- **必需运行时**：Ubuntu 22.04、ROS 2 Humble（`/opt/ros/humble`）、Python 3.10、Gazebo Classic 11 + `ros-humble-gazebo-ros-pkgs` + `gazebo_ros2_control`（插件缺失 → `sudo ./install_gazebo_ros.sh`）。
- **标准环境**：devcontainer 镜像 `dddmr_gz:20260407_05`（`.devcontainer/devcontainer.json`）——工作空间挂载至 `/workspace`，`--privileged --network=host --gpus=all`，X11 透传，NVIDIA GL 由 `postCreateCommand`（`setup_gpu.sh`）配置。在无容器的 WSL2 上需保证 GPU 渲染，否则退化为 llvmpipe 软渲染。
- **包管理**：系统依赖用 apt + 构建用 colcon；项目无 pip 依赖。
- **外部依赖**：`multi_task_bridge` 位于独立工作空间（README 引用的是过期绝对路径）。当前检出路径为 `/home/wtr/projects/robotdog_nav` —— README/QUICK_START/测试脚本中的 `/media/lenovo/1/project/gazebo/...` 均为过期路径，执行时需替换为本地路径。`test_full_system.sh` source 了过期路径，未修复前必然失败。
- **易踩坑点**：`global2local/` 已被 `COLCON_IGNORE`（遗留无人机代码）；champ `bringup.launch.py` 中 `state_estimation_node` 与两个 EKF 节点已被注释；`nav2_straight_planner` 未被任何 launch 引用。

## 测试与质量保障

无 pytest/colcon 测试框架——质量保障为驱动真实 ROS 节点的黑盒集成脚本。未配置 lint/格式化工具。

| 测试 | 内容 | 前置条件 |
|---|---|---|
| `python3 test_execute_task_py.py` | 纯 rclpy 脚本（非 pytest）：对 `/agent/uav1/execute_task` 的 4 个用例——局部坐标航点、GPS 坐标航点、错误 agent ID 拒绝、无效能力拒绝 | taskserver 运行中；需手动先 source 环境 |
| `./test_execute_task.sh` | 同样 4 用例，经 `ros2 service call`（用例 4 发送畸形 JSON） | 工作空间已构建 + taskserver；脚本自行 source |
| `./test_full_system.sh` | 3 次服务调用，含真实 GPS 坐标（30.06888, 120.04646） | execute_task + 导航节点均运行；**过期 source 路径需先修复** |
| `./quick_test.sh` | 编排器：检查服务、自动启动 `execute_task.launch.py agent_id:=uav1`、运行 `test_execute_task_py.py`、随后杀掉服务 | 仅需已构建工作空间（无需导航节点/Gazebo） |
| `src/robotdog_nav/waypoint_nav/test/test_world_odom_nav.py` | 独立 rclpy 节点：直接发布 MultiGoal，分阶段监视 `/cmd_vel` + `/nav_status` | 导航栈运行中 |

通过与否依据服务 JSON 响应与 rosout 日志判断，无断言。全保真运行需 Gazebo + gps_imu_fusion + waypoint_nav 就绪（QUICK_START.md 方案 C）；服务级测试无需仿真（方案 A）。RViz 中观察结果（`waypoint_nav/config/nav_visualization.rviz`）：绿色球体 = 当前目标，蓝色球体 = 待执行航点，绿色线 = 规划路径。
