# 增强版启动脚本 - launch_service.sh

## 说明

此脚本是原始 `launch.sh` 的增强版本，添加了更多功能选项。

## 用法

```bash
./launch_service.sh [选项]
```

## 可用选项

| 选项 | 说明 |
|------|------|
| `dognav` | 启动航路点导航节点（waypoint_nav） |
| `gpsloc` | 启动 GPS 定位融合节点（gps_imu_fusion） |
| `taskserver` | 启动指控端任务服务节点（execute_task_server） |
| `full` | 启动完整导航系统（GPS + 导航 + 任务服务） |

## 使用示例

### 启动航路点导航

```bash
./launch_service.sh dognav
```

等价于：
```bash
ros2 launch waypoint_nav waypoint_nav.launch.py use_sim_time:=true
```

### 启动 GPS 定位

```bash
./launch_service.sh gpsloc
```

等价于：
```bash
ros2 launch gps_imu_fusion gps_imu_fusion.launch.py use_sim_time:=true
```

### 启动任务服务

```bash
./launch_service.sh taskserver
```

等价于：
```bash
ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1
```

### 启动完整系统

```bash
./launch_service.sh full
```

这会依次启动：
1. GPS/IMU 融合定位
2. 航路点导航
3. 指控端任务服务

## 与原始 launch.sh 的区别

| 功能 | launch.sh | launch_service.sh |
|------|-----------|-------------------|
| dognav | ✅ | ✅ |
| gpsloc | ✅ | ✅ |
| taskserver | ❌ | ✅ |
| full | ❌ | ✅ |

## 配合测试脚本使用

```bash
# 启动服务
./launch_service.sh taskserver

# 运行测试
python3 test_execute_task_py.py

# 或使用快速测试
./quick_test.sh
```

## 注意事项

- 使用前确保已执行 `colcon build --symlink-install`
- `full` 模式会自动在后台启动前两个组件
- 使用 Ctrl+C 可以停止当前进程
