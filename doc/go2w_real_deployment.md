# Go2w 真实机器狗导航功能部署清单

本文档描述如何在真实的 Unitree Go2w 轮式机器狗上部署导航功能。

---

## 📋 一、系统架构对比

| 组件 | 仿真环境 | 真实机器狗 |
|------|----------|------------|
| 机器人模型 | Gazebo URDF | 真实 Go2w 硬件 |
| 传感器 | Gazebo 插件 | 真实传感器 |
| 控制器 | ros2_control | Unitree SDK |
| 定位 | gps_imu_fusion | gps_imu_fusion |
| 导航算法 | waypoint_nav | waypoint_nav |

---

## 🔧 二、硬件配置需求

### 2.1 机器狗本体
- [x] Unitree Go2w 轮式机器狗
- [ ] 电池（满电状态）
- [ ] 遥控器（用于紧急停止）

### 2.2 传感器配置
| 传感器 | 用途 | Topic | 状态 |
|--------|------|-------|------|
| Livox Mid-360 激光雷达 | 避障 | `/livox/lidar` | 需安装 |
| GPS 模块 (RTK 推荐) | 全局定位 | `/gps/data` | 需安装 |
| IMU | 姿态融合 | `/livox/imu` | 内置 |
| 轮式里程计 | 局部定位 | `/odom` | 内置 |

### 2.3 计算平台
- [ ] 机载电脑（如 NVIDIA Jetson Orin/NX）
- [ ] 或 外部计算机 + WiFi/5G 通信
- [ ] 网络交换机/路由器

---

## 💻 三、软件环境需求

### 3.1 基础软件
```bash
# 操作系统
Ubuntu 22.04 LTS

# ROS2 版本
ROS2 Humble Hawksbill

# 依赖库
- GeographicLib (GPS 坐标转换)
- tf2 (坐标变换)
- nav2 (导航框架)
```

### 3.2 Unitree 官方 SDK
```bash
# Unitree Go2 SDK
git clone https://github.com/unitreerobotics/unitree_ros.git
git clone https://github.com/unitreerobotics/unitree_ros2.git

# 编译
colcon build --symlink-install
```

### 3.3 项目依赖
```bash
# 克隆项目
cd ~/robotdog_nav
./build.sh

# 安装依赖
rosdep install --from-paths src --ignore-src -y
```

---

## 🔌 四、硬件接口配置

### 4.1 传感器连接

#### Livox Mid-360 激光雷达
```bash
# 安装驱动
git clone https://github.com/Livox-SDK/Livox-ROS2-SDK.git

# 配置文件 (config/MID360_config.json)
{
  "lidar_summary_info": {
    "lidar_type": 1
  },
  "lidar_configs": [{
    "ip": "192.168.1.100",
    "pcl_data_type": 1,
    "pattern_mode": 2,
    "extrinsic_parameter": {
      "roll": 0.0,
      "pitch": 0.0,
      "yaw": 0.0,
      "x": 0.0,
      "y": 0.0,
      "z": 0.0
    }
  }]
}

# 启动
ros2 launch livox_lidar2_launch.py
```

#### GPS 模块
```bash
# 如果使用 NMEA 输出的 GPS
# 安装 nmea_navsat_driver
sudo apt install ros-humble-nmea-navsat-driver

# 启动 (根据实际串口修改)
ros2 run nmea_navsat_driver nmea_serial_driver_node \
  --ros-args -p port:=/dev/ttyUSB0 -p baud:=9600
```

### 4.2 坐标系定义

```
base_link 坐标系:
  - 原点：机器狗几何中心
  - X 轴：前进方向
  - Y 轴：左侧
  - Z 轴：向上

livox 坐标系:
  - 相对于 base_link 的变换需配置
  - 推荐：base_link 上方 0.1m
```

### 4.3 TF 变换配置

修改 `waypoint_nav.launch.py` 中的静态变换：

```python
# base_link 到 livox 的变换（根据实际安装位置调整）
static_transform_publisher \
  --x 0.0 --y 0.0 --z 0.1 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link \
  --child-frame-id livox
```

---

## 🚀 五、部署步骤

### 步骤 1: 环境准备
```bash
# 1. 更新系统
sudo apt update && sudo apt upgrade -y

# 2. 安装 ROS2 Humble
# 参考：https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# 3. 安装依赖
sudo apt install \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-geographiclib-ros \
  ros-humble-nmea-navsat-driver
```

### 步骤 2: 编译项目
```bash
cd /media/nhy/office/Gazebo/0403/robotdog_nav
./build.sh
source install/setup.bash
```

### 步骤 3: 配置参数

#### 修改 `gps_imu_fusion/config/params.yaml`:
```yaml
gps_imu_fusion_node:
  ros__parameters:
    origin_lat: 30.XXXXX  # 设置当地 GPS 原点纬度
    origin_lon: 120.XXXXX # 设置当地 GPS 原点经度
    origin_alt: XX.X      # 设置当地 GPS 原点高度
    yaw_offset: 0.0       # IMU yaw 偏移（根据安装调整）
```

#### 修改 `waypoint_nav/config/params.yaml`:
```yaml
nav_waypoint_node:
  ros__parameters:
    goal_tolerance: 0.5        # 到达容差 (m)
    yaw_tolerance: 0.3         # 角度容差 (rad)
    linear_velocity: 0.5       # 线速度 (m/s) - 降低速度保证安全
    angular_velocity_limit: 0.8 # 角速度 (rad/s)
    robot_radius: 0.5          # 机器狗半径
```

### 步骤 4: 启动传感器
```bash
# 终端 1: 启动激光雷达
ros2 launch livox_lidar2_launch.py

# 终端 2: 启动 GPS 节点
ros2 run nmea_navsat_driver nmea_serial_driver_node \
  --ros-args -p port:=/dev/ttyUSB0 -p baud:=9600

# 验证传感器数据
ros2 topic echo /livox/lidar
ros2 topic echo /gps/data
ros2 topic echo /livox/imu
```

### 步骤 5: 启动 Unitree 机器狗控制
```bash
# 启动 Go2w 驱动（根据实际包名调整）
ros2 launch unitree_go2w_bringup go2w_bringup.launch.py

# 验证速度指令
ros2 topic echo /cmd_vel
```

### 步骤 6: 启动导航系统
```bash
# 启动定位融合（与导航已解耦，需分别启动；实机用系统时钟）
source install/setup.bash
ros2 launch gps_imu_fusion gps_imu_fusion.launch.py &

# 启动导航（真实模式）
ros2 launch waypoint_nav waypoint_nav.launch.py \
  simulation:=false \
  use_sim_time:=false
```

### 步骤 7: 发送航点任务
```bash
# 发送测试航点（小范围测试）
ros2 topic pub --once /waypoint_goals msg_set_msgs/msg/MultiGoal "{
  multi_goal_points: [
    {x_or_lat: 2.0, y_or_lon: 0.0, z_or_alt: 0.0, yaw: 0.0, vel: 0.3},
    {x_or_lat: 2.0, y_or_lon: 2.0, z_or_alt: 0.0, yaw: 1.57, vel: 0.3},
    {x_or_lat: 0.0, y_or_lon: 2.0, z_or_alt: 0.0, yaw: 3.14, vel: 0.3},
    {x_or_lat: 0.0, y_or_lon: 0.0, z_or_alt: 0.0, yaw: 0.0, vel: 0.3}
  ],
  is_gps_aid: false
}"
```

---

## 🔍 六、Topic 对照表

| Topic | 类型 | 方向 | 说明 |
|-------|------|------|------|
| `/livox/lidar` | PointCloud2 | 传感器 → | 激光雷达点云 |
| `/livox/imu` | Imu | 传感器 → | IMU 数据 |
| `/gps/data` | NavSatFix | 传感器 → | GPS 数据 |
| `/world_odom` | Odometry | 导航 ← | 融合定位输出 |
| `/cmd_vel` | Twist | 导航 → | 速度指令 |
| `/waypoint_goals` | MultiGoal | 外部 → | 航点任务输入 |

---

## ⚠️ 七、安全注意事项

### 7.1 测试前检查
- [ ] 确保测试场地开阔、平坦
- [ ] 清除地面上的障碍物
- [ ] 准备紧急停止遥控器
- [ ] 确认电池电量充足
- [ ] 确认 WiFi/通信稳定

### 7.2 首次测试建议
1. **室内小范围测试** (1m x 1m)
2. **降低速度参数** (0.3 m/s)
3. **人员在安全距离观察**
4. **随时准备急停**

### 7.3 速度参数建议
| 测试阶段 | 线速度 | 角速度 |
|----------|--------|--------|
| 首次测试 | 0.2 m/s | 0.3 rad/s |
| 室内测试 | 0.3 m/s | 0.5 rad/s |
| 室外测试 | 0.5 m/s | 0.8 rad/s |
| 正常运行 | 0.8 m/s | 1.0 rad/s |

---

## 🐛 八、常见问题排查

### 问题 1: 传感器数据不发布
```bash
# 检查 Topic 列表
ros2 topic list

# 检查传感器连接
ls -l /dev/ttyUSB*  # GPS
rostopic hz /livox/lidar
```

### 问题 2: TF 变换错误
```bash
# 检查 TF 树
ros2 run tf2_tools view_frames.py
evince frames.pdf

# 检查 TF 发布
ros2 topic echo /tf
```

### 问题 3: 机器狗不响应速度指令
```bash
# 检查 /cmd_vel 是否有输出
ros2 topic echo /cmd_vel

# 检查机器狗控制状态
ros2 topic list | grep go2w
```

### 问题 4: GPS 信号弱
- 确保 GPS 天线在室外开阔地带
- 检查 RTK 固定状态
- 考虑使用视觉/激光 SLAM 辅助定位

---

## 📊 九、性能优化建议

### 9.1 计算资源优化
```bash
# 限制导航节点 CPU 使用
taskset -c 0-3 ros2 launch robotdog_nav waypoint_nav.launch.py
```

### 9.2 网络优化
```bash
# 如果使用 WiFi 通信
- 使用 5GHz 频段
- 确保信号强度 > -60dBm
- 考虑使用定向天线
```

### 9.3 定位优化
```yaml
# 如果使用 RTK GPS
gps_imu_fusion_node:
  ros__parameters:
    origin_lat: <精确测量的原点纬度>
    origin_lon: <精确测量的原点经度>
    yaw_offset: <实地校准的 yaw 偏移>
```

---

## 📝 十、检查清单

### 出发前检查
- [ ] 电池充电
- [ ] 传感器校准
- [ ] WiFi/通信测试
- [ ] 遥控器电量
- [ ] 笔记本电脑电量

### 现场设置检查
- [ ] GPS 信号质量
- [ ] 激光雷达视野
- [ ] TF 树完整性
- [ ] Topic 数据流
- [ ] 紧急停止功能

### 测试后检查
- [ ] 数据记录完整
- [ ] 电池状态
- [ ] 硬件无损坏
- [ ] 日志文件保存

---

## 🔗 十一、参考资源

- [Unitree Go2 文档](https://github.com/unitreerobotics/unitree_ros)
- [Nav2 文档](https://navigation.ros.org/)
- [Livox ROS2 驱动](https://github.com/Livox-SDK/Livox-ROS2-SDK)
- [ROS2 Humble 文档](https://docs.ros.org/en/humble/)

---

**文档版本**: 1.0  
**更新日期**: 2026-04-20  
**适用项目**: robotdog_nav
