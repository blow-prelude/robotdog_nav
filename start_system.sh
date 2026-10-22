#!/usr/bin/env bash
# start_system.sh —— 一键启动机器狗导航系统
#
# 按「仿真层(可选) → 定位层 → 导航层 → 任务层」顺序启动：
#   gazebo               Gazebo Classic 仿真（--no-gazebo / --real 时跳过）
#   gps_imu_fusion       定位：/gps/data + /livox/imu → /world_odom + TF world→base_link
#   waypoint_nav         导航：Nav2 controller_server + nav_waypoint_node + RViz
#   execute_task_server  任务：/agent/<agent_id>/execute_task 服务
#
# 用法：
#   ./start_system.sh                          完整仿真（默认，Gazebo + 定位 + 导航 + 任务）
#   ./start_system.sh --no-gazebo              无 Gazebo（rosbag 回放等场景，仍用仿真时钟）
#   ./start_system.sh --real                   实机：系统时钟、simulation:=false、无 Gazebo
#   ./start_system.sh --agent uav2             指定 agent ID（默认 uav1）
#   ./start_system.sh --world go2w_lidar_gps.launch.py
#                                              指定 Gazebo 入口（默认 go2_lidar_gps.launch.py，
#                                              足式 Go2；轮足 Go2W 用 go2w_lidar_gps.launch.py）
#
# 各组件输出写入 log/start_system/<时间戳>/<组件名>.log；任一组件退出则停止全部并报告。
set -u

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR"

usage() {
  cat <<'EOF'
用法：./start_system.sh [选项]

选项：
  --no-gazebo        不启动 Gazebo 仿真（rosbag 回放等场景，仍使用仿真时钟）
  --real             实机模式：系统时钟、simulation:=false、不启动 Gazebo
  --agent  <id>      指定 agent ID（默认 uav1）
  --world  <file>    Gazebo launch 文件（默认 go2_lidar_gps.launch.py）
  -h, --help         显示本帮助
EOF
}

# ---------- 参数解析 ----------
GAZEBO=1
REAL=0
AGENT_ID=uav1
WORLD_LAUNCH=go2_lidar_gps.launch.py

while [ $# -gt 0 ]; do
  case "$1" in
    --no-gazebo) GAZEBO=0 ;;
    --real)      REAL=1; GAZEBO=0 ;;
    --agent)
      [ $# -ge 2 ] || { echo "错误：--agent 缺少参数"; usage; exit 1; }
      AGENT_ID=$2; shift ;;
    --world)
      [ $# -ge 2 ] || { echo "错误：--world 缺少参数"; usage; exit 1; }
      WORLD_LAUNCH=$2; shift ;;
    -h|--help)   usage; exit 0 ;;
    *)           echo "未知选项：$1"; usage; exit 1 ;;
  esac
  shift
done

# ---------- 环境准备 ----------
if [ ! -f install/setup.bash ]; then
  echo "错误：未找到 install/setup.bash，请先构建工作空间：./build.sh"
  exit 1
fi
# ROS 的 setup 脚本不兼容 set -u（如 setup.bash 第 8 行引用未定义的 AMENT_TRACE_SETUP_FILES），
# source 前后需临时关闭 nounset
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

# DDS 域隔离（见 doc/ros_domain_clock_conflict.md）：默认 42，外部已设置则沿用
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

USE_SIM_TIME=true
SIMULATION=true
if [ "$REAL" -eq 1 ]; then
  USE_SIM_TIME=false
  SIMULATION=false
fi

if [ -z "${DISPLAY:-}" ] && [ "$GAZEBO" -eq 1 ]; then
  echo "警告：未检测到 DISPLAY，Gazebo/RViz 图形界面可能无法显示"
fi

LOG_DIR="log/start_system/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

# ---------- 进程管理 ----------
# 每个组件经 setsid 放入独立进程组（组长 pid 即 pgid），停止时对整组发信号，
# 确保覆盖 ros2 launch 派生的全部子进程（节点、gzserver、RViz 等，它们对 SIGINT
# 响应慢甚至忽略，逐个 pid 杀会漏）
declare -a PIDS=()
declare -a NAMES=()

# 进程是否仍在运行（Z 为已退出未回收的僵尸进程，视同已停止）
is_running() {
  kill -0 "$1" 2>/dev/null && [ "$(ps -o stat= -p "$1" 2>/dev/null)" != "Z" ]
}

any_running() {
  local pid
  for pid in "${PIDS[@]}"; do
    is_running "$pid" && return 0
  done
  return 1
}

force_kill() {
  local pid
  for pid in "${PIDS[@]}"; do
    kill -KILL -- "-$pid" 2>/dev/null
  done
  wait 2>/dev/null
}

cleanup() {
  # 清理期间再次 Ctrl-C 不重入本函数，直接升级为强杀退出
  trap 'force_kill; exit 130' INT TERM
  echo ""
  echo "正在停止全部组件（再按 Ctrl-C 立即强制结束）..."
  local pid i
  for pid in "${PIDS[@]}"; do
    kill -INT -- "-$pid" 2>/dev/null
  done
  # 最多等 5 秒让组件优雅退出（ros2 launch 收尾需要时间），超时强杀
  for i in $(seq 1 10); do
    any_running || break
    sleep 0.5
  done
  force_kill
  echo "已全部停止。日志保留在 $LOG_DIR/"
}
trap 'cleanup; exit 130' INT TERM

start_component() {
  local name=$1
  shift
  echo "[$(date +%H:%M:%S)] 启动 $name"
  setsid "$@" >"$LOG_DIR/$name.log" 2>&1 &
  PIDS+=($!)
  NAMES+=("$name")
}

# ---------- 按层启动 ----------
echo "模式：$([ "$REAL" -eq 1 ] && echo 实机 || echo 仿真$([ "$GAZEBO" -eq 0 ] && echo '（无 Gazebo）'))，agent=$AGENT_ID，ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "组件日志目录：$LOG_DIR"

if [ "$GAZEBO" -eq 1 ]; then
  start_component gazebo ros2 launch robot_scene "$WORLD_LAUNCH"

  echo "等待 Gazebo 时钟 /clock 就绪（最多 60 秒）..."
  if timeout 60 ros2 topic echo --once /clock > /dev/null 2>&1; then
    echo "Gazebo 已就绪"
  else
    echo "警告：60 秒内未收到 /clock，继续启动（Gazebo 可能仍在加载，详见 $LOG_DIR/gazebo.log）"
  fi
fi

start_component gps_imu_fusion \
  ros2 launch gps_imu_fusion gps_imu_fusion.launch.py use_sim_time:=$USE_SIM_TIME
sleep 2  # 等 /world_odom 与 TF world→base_link 先起来

start_component waypoint_nav \
  ros2 launch waypoint_nav waypoint_nav.launch.py use_sim_time:=$USE_SIM_TIME simulation:=$SIMULATION
# start_component robotdog_nav2_bringup \
#   ros2 launch robotdog_nav2_bringup world_nav2.launch.py controller:=mppi planner:=smac
sleep 2

start_component execute_task_server \
  ros2 launch execute_task_server execute_task.launch.py agent_id:=$AGENT_ID

# ---------- 运行监控 ----------
echo ""
echo "全部组件已启动，按 Ctrl-C 停止全部。"
echo "实时查看某组件输出：tail -f $LOG_DIR/<组件名>.log"
echo "下发测试任务参考 QUICK_START.md 或 test_full_system.sh"

# 任一组件退出即视为系统异常：报告后停止其余组件
wait -n

echo ""
for i in "${!PIDS[@]}"; do
  if ! is_running "${PIDS[$i]}"; then
    echo "组件 ${NAMES[$i]} 已退出，日志：$LOG_DIR/${NAMES[$i]}.log"
  fi
done
cleanup
exit 1
