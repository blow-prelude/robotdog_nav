#!/usr/bin/env bash
# ============================================================================
# 切换到本地编译的、未打 0010-armhf-singleton 补丁的 Gazebo 11.10.2
#
# 背景：arm64 上只能从 PPA(openrobotics/gazebo11-non-amd64) 装到
#   11.10.2+dfsg-1osrf6，它带 0010-armhf-singleton.patch（把 GZ_SINGLETON_DECLARE
#   宏清空），导致 SingletonT 在每个 .so 里各有一份 —— 跨进程 gazebo transport
#   的 advertise 全部静默丢失：gzclient 收不到场景/时钟（机器人不显示、黑屏），
#   IMU 收不到 link_data（/livox/imu 无数据）。
#
# 修复：用 Ubuntu jammy 原生源码（gazebo 11.10.2+dfsg-1，无该补丁）编译安装到
#   ~/gzbuild/prefix，运行时优先使用它。
#
# 用法：source ./env_gazebo_fixed.sh   然后再 ./start_simulation.sh / ros2 launch ...
# ============================================================================

GAZEBO_FIXED_PREFIX="${GAZEBO_FIXED_PREFIX:-$HOME/gzbuild/prefix}"

if [ ! -x "$GAZEBO_FIXED_PREFIX/bin/gzserver" ]; then
  echo "env_gazebo_fixed.sh: 找不到 $GAZEBO_FIXED_PREFIX/bin/gzserver" >&2
  return 1 2>/dev/null || exit 1
fi

export PATH="$GAZEBO_FIXED_PREFIX/bin:$PATH"
# 必须放在最前，覆盖 /usr/lib/aarch64-linux-gnu 下 PPA 装的那套 libgazebo*.so.11
export LD_LIBRARY_PATH="$GAZEBO_FIXED_PREFIX/lib:${LD_LIBRARY_PATH:-}"

echo "已切换 gazebo: $(command -v gzserver) ($("$GAZEBO_FIXED_PREFIX/bin/gzserver" --version 2>/dev/null | head -1))"
