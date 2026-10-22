#!/usr/bin/env bash
# start_mock_system.sh —— 一键启动 mock 虚拟场景导航系统（无 Gazebo、无 rosbag）
#
# 按「虚拟场景 → 导航层 → 任务层」顺序启动：
#   mock_scene           虚拟场景：/clock（50Hz 墙钟）+ /cmd_vel 运动学积分成 /world_odom
#                        + 合成障碍点云 /livox/lidar（launch 内含 odom_to_tf 与静态 livox TF）
#   导航栈（二选一）      nav2      robotdog_nav2_bringup world_nav2.launch.py（默认，mppi+smac，
#                                  NavFn 全局规划绕开 mock 障碍）
#                        waypoint  waypoint_nav.launch.py（直线到达，验证闭环）
#   execute_task_server  任务：/agent/<agent_id>/execute_task 服务
#
# 互斥规则（见 doc/devel/2026-10-21-design-mock-scene.md §2.1）：
#   mock 是唯一 /clock、唯一 /world_odom、唯一 world→base_link 发布者，
#   禁止与 Gazebo、`ros2 bag play --clock`、gps_imu_fusion 同跑；
#   启动前检测到已有 /clock 发布者将拒绝启动（--force 跳过检测）。
#
# mock 崩溃 = 仿真时钟冻结（导航定时器停摆）：直接 Ctrl-C 后重启本脚本即可，
# 墙钟不回跳，无 TF_OLD_DATA。
#
# 用法：
#   ./start_mock_system.sh                    默认 nav2 栈（mppi + smac）
#   ./start_mock_system.sh --stack waypoint   waypoint_nav 直线栈
#   ./start_mock_system.sh --agent uav2       指定 agent ID（默认 uav1）
#   ./start_mock_system.sh --force            跳过 /clock 冲突检测
#
# 各组件输出写入 log/start_mock_system/<时间戳>/<组件名>.log；任一组件退出则停止全部并报告。
set -u

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR"

usage() {
  cat <<'EOF'
用法：./start_mock_system.sh [选项]

选项：
  --stack <name>     导航栈：nav2（默认，robotdog_nav2_bringup）或 waypoint（waypoint_nav）
  --agent <id>       指定 agent ID（默认 uav1）
  --force            跳过启动前的 /clock 发布者冲突检测
  -h, --help         显示本帮助
EOF
}

# ---------- 参数解析 ----------
STACK=nav2
AGENT_ID=uav1
FORCE=0

while [ $# -gt 0 ]; do
  case "$1" in
    --stack)
      [ $# -ge 2 ] || { echo "错误：--stack 缺少参数"; usage; exit 1; }
      case "$2" in nav2|waypoint) STACK=$2 ;; *) echo "错误：--stack 仅支持 nav2 或 waypoint"; usage; exit 1 ;; esac
      shift ;;
    --agent)
      [ $# -ge 2 ] || { echo "错误：--agent 缺少参数"; usage; exit 1; }
      AGENT_ID=$2; shift ;;
    --force)     FORCE=1 ;;
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
# ROS 的 setup 脚本不兼容 set -u，source 前后需临时关闭 nounset（同 start_system.sh）
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

# DDS 域隔离（见 doc/ros_domain_clock_conflict.md）：外部已设置（如 ~/.bashrc）则沿用
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

if [ -z "${DISPLAY:-}" ]; then
  echo "警告：未检测到 DISPLAY，RViz 可能无法显示（不影响无头验证）"
fi

LOG_DIR="log/start_mock_system/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

# ---------- 启动前冲突检测：/clock 必须无发布者 ----------
if [ "$FORCE" -eq 0 ]; then
  echo "检测 /clock 发布者冲突（最多等 10 秒，--force 跳过）..."
  CLOCK_PUBS=$(timeout 10 ros2 topic info /clock 2>/dev/null | sed -n 's/^Publisher count: //p')
  CLOCK_PUBS=${CLOCK_PUBS:-0}
  if [ "$CLOCK_PUBS" -gt 0 ]; then
    echo "错误：检测到 /clock 已有 $CLOCK_PUBS 个发布者（Gazebo / bag play --clock / 另一个 mock？）。"
    echo "      mock 必须是唯一 /clock 发布者，请先关闭冲突源，或用 --force 强制启动。"
    exit 1
  fi
  echo "无冲突"
fi

# ---------- 进程管理（同 start_system.sh：setsid 进程组 + 整组信号） ----------
declare -a PIDS=()
declare -a NAMES=()

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
echo "模式：mock 虚拟场景，导航栈=$STACK，agent=$AGENT_ID，ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "组件日志目录：$LOG_DIR"

# 1. 虚拟场景（时钟源必须最先就绪，导航节点 use_sim_time 依赖 /clock）
start_component mock_scene ros2 launch nav_replay_support mock_scene.launch.py
echo "等待 mock 时钟 /clock 就绪（最多 30 秒）..."
if timeout 30 ros2 topic echo --once /clock > /dev/null 2>&1; then
  echo "mock 场景已就绪（/clock + /world_odom + /livox/lidar）"
else
  echo "警告：30 秒内未收到 /clock，继续启动（详见 $LOG_DIR/mock_scene.log）"
fi

# 2. 导航栈（use_sim_time 消费 mock 的 /clock；simulation 保持默认 true，静态 livox TF 由 mock_scene 持有）
if [ "$STACK" = "nav2" ]; then
  start_component nav_stack \
    ros2 launch robotdog_nav2_bringup world_nav2.launch.py controller:=mppi planner:=smac
else
  start_component nav_stack \
    ros2 launch waypoint_nav waypoint_nav.launch.py use_sim_time:=true
fi
sleep 2

# 3. 任务层
start_component execute_task_server \
  ros2 launch execute_task_server execute_task.launch.py agent_id:=$AGENT_ID

# ---------- 运行监控 ----------
echo ""
echo "全部组件已启动，按 Ctrl-C 停止全部。"
echo "实时查看某组件输出：tail -f $LOG_DIR/<组件名>.log"
echo ""
echo "下发测试任务（局部坐标，mock 默认场景目标 (4,0)；换栈前先 Ctrl-C 关闭旧栈）："
cat <<'EOF'
  ros2 service call /agent/uav1/execute_task execute_task_server/srv/ExecuteTask '{
    request_json: "{\"mission_id\":\"mock_01\",\"agent_id\":\"uav1\",\"capability_id\":\"goto_waypoints\",\"params\":{\"waypoint_frame\":\"local\",\"waypoints\":[{\"x\":4.0,\"y\":0.0,\"vel\":0.5}]}}"
  }'
EOF
echo ""

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
