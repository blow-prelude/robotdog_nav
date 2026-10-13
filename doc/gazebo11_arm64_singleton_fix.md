# arm64 (Jetson) 上 Gazebo 机器人不显示 / 传感器无数据：根因与修复

> 2026-10-13 在 Jetson NX（aarch64, Ubuntu 22.04, ROS 2 Humble）上定位并修复。
> 症状：`ros2 launch robot_scene go2_lidar_gps.launch.py` 后 Gazebo 里看不到机器狗、
> 有时黑屏；`/livox/imu`、`/livox/lidar` 完全没有数据（`/gps/data` 却正常）。

## 一、根因：arm64 版 gazebo11 被 arm 补丁破坏了跨库单例

本机 gazebo 只能从 PPA `ppa:openrobotics/gazebo11-non-amd64` 装到
`11.10.2+dfsg-1osrf6`，它带 `debian/patches/0010-armhf-singleton.patch`。

对比 `/usr/include/gazebo-11/gazebo/common/SingletonT.hh`（打补丁后）与上游
（Ubuntu jammy 源码 `gazebo_11.10.2+dfsg-1`，未打该补丁）：

```cpp
// 上游（GCC 分支）——单例只在“用宏的那一个库”里显式实例化一次并导出
#define GZ_SINGLETON_DECLARE(visibility, n1, n2, singletonType) \
  namespace n1 { namespace n2 { class singletonType; } } \
  template class visibility ::SingletonT<n1::n2::singletonType>;

#ifdef __clang__
#define GZ_SINGLETON_DECLARE(...)      // 空
#endif
```

补丁把 GCC 分支也变成了「空宏」，于是 `SingletonT<T>::GetInstance()` 里的
函数局部静态（`static T t;`）在**每个 .so 各有一份**：

| 库 | `SingletonT` 导出符号数 |
|---|---|
| 本机 PPA 包 `libgazebo_transport.so` | **0** |
| 用原生源码编译的同一库 | **24**（其中 12 个是 `TopicManager`） |

后果（`TopicManager`/`ConnectionManager` 每库一份）：

- **`TopicManager::Advertise` 是内联模板** → 用到了从未 `Init()` 的私有副本 →
  全部 advertise 包被静默丢弃。实测 `strace -f -e trace=sendto,sendmsg gzserver`
  在旧库下**看不到任何 advertise 包**，新库下能看到
  `/gazebo/default/pose/local/info`、`physics/contacts`、`gui` 等。
- **gzclient 是独立进程**，收不到 scene/clock/pose → 世界能加载但**机器狗永不出现、
  视图空白/黑屏**；`gz topic -i` 显示「无类型、无发布者」，`gz stats` 无输出。
- **IMU 传感器靠 gazebo transport 收 `link_data`**（`ImuSensor::OnLinkData`）。
  gdb 断点实测：`ImuSensor::UpdateImpl` 一直在被调用，但 `OnLinkData` 一次都没进
  → 传感器在跑却拿不到数据 → `/livox/imu` 静默。（`/gps/data` 正常，因为 GPS 直接
  读 link 位姿，不走 transport。）

> 另注：`SingletonT` 家族还包含 `SensorManager`、`RenderEngine`、`RTShaderSystem`、
> `MeshManager`、`SystemPaths` 等，受影响面很大，所以「换掉这个 gazebo」是唯一根治办法。

## 二、第二个问题：激光雷达传感器缺 `always_on`

`robot_scene/xacro/mid360.xacro` 的 ray 传感器没有 `<always_on>`（同机 gps/imu 都写了）。
gazebo 默认 `always_on=false`，传感器不主动更新（gdb 实测 `RaySensor::UpdateImpl`
一次都没进）。已在 xacro 中补上：

```xml
<always_on>true</always_on>
```

否则只在 gzclient 订阅了它的可视化话题时才会被「激活」——无 GUI 跑仿真时雷达会静默。

> 注：在这个有 bug 的 gazebo 上，一旦雷达被激活，`Mid360OdeMultiRayShape::UpdateRays()`
> 里的 `dSpaceCollide2` 会在首次扫描时把 gzserver 打成 SIGSEGV（exit -11）。换成
> 未打补丁的 gazebo 后该崩溃消失。

## 三、修复步骤（把原生源码编到用户前缀）

apt 里没有可用的 arm64 二进制：Ubuntu jammy **从未发布 arm64 的 gazebo11**（只有
arch:all 的 gazebo-common/doc），OSRF 的 arm64 源只有 gz-sim 系，Debian 只有 11.1.0。
所以用 Ubuntu jammy 的原生源码自行编译（源码不带那个补丁）：

```bash
# 1) 取源码（不带 singleton 补丁）
mkdir -p ~/gzbuild/src && cd ~/gzbuild/src
BASE=https://snapshot.ubuntu.com/ubuntu/20250101T000000Z/pool/universe/g/gazebo
for f in gazebo_11.10.2+dfsg.orig.tar.xz gazebo_11.10.2+dfsg-1.debian.tar.xz gazebo_11.10.2+dfsg-1.dsc; do
  curl -C - -O $BASE/$f
done
dpkg-source -x gazebo_11.10.2+dfsg-1.dsc      # 解包（会应用 8 个 Ubuntu 兼容补丁，均与单例无关）

# 2) 补上被打包删掉的 qtpropertybrowser（GUI 依赖；jammy 只给 amd64 编了 libqtpropertybrowser5）
cd gazebo-11.10.2+dfsg/gazebo/gui/qtpropertybrowser
for f in qtbuttonpropertybrowser.cpp qtbuttonpropertybrowser.h qteditorfactory.cpp \
         qteditorfactory.h qtgroupboxpropertybrowser.cpp qtgroupboxpropertybrowser.h \
         qtpropertybrowser.cpp qtpropertybrowser.h qtpropertybrowserutils.cpp \
         qtpropertybrowserutils_p.h qtpropertymanager.cpp qtpropertymanager.h \
         qttreepropertybrowser.cpp qttreepropertybrowser.h qtvariantproperty.cpp \
         qtvariantproperty.h; do
  curl -O https://raw.githubusercontent.com/gazebosim/gazebo-classic/gazebo11_11.10.2/gazebo/gui/qtpropertybrowser/$f
done
cd - && patch -p1 -R < debian/patches/0003-use-system-qtpropertybrowser.patch   # 改回用自带的

# 3) 依赖（只需这几个，其余本机已装）
sudo apt install libode-dev libspnav-dev libgraphviz-dev libignition-msgs5-dev \
                 fonts-roboto-unhinted ccache

# 4) 编译安装（8 核约 25~35 分钟）
mkdir -p ~/gzbuild/build && cd ~/gzbuild/build
export PATH=/usr/lib/ccache:$PATH
cmake ../src/gazebo-11.10.2+dfsg \
  -DCMAKE_INSTALL_PREFIX=$HOME/gzbuild/prefix -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DBUILD_TESTING=OFF
make -j7 && make install
```

## 四、日常使用

```bash
source ./env_gazebo_fixed.sh            # 把 ~/gzbuild/prefix 的 gzserver/gzclient 提到最前
./start_simulation.sh                   # 或 ros2 launch robot_scene go2_lidar_gps.launch.py
```

`env_gazebo_fixed.sh` 只做两件事：`PATH` 与（关键的）`LD_LIBRARY_PATH` 前置，
以覆盖 `/usr/lib/aarch64-linux-gnu` 下 PPA 装的那套 `libgazebo*.so.11`。不装系统、
随时可回退（取消 source 即回到旧行为）。

## 五、验证要点（修复后实测）

```bash
gz stats -d 2                     # 旧库无输出；新库有 Factor/SimTime/RealTime
gz model -m go2 -p                # 能查到机器狗位姿
```
- `/livox/lidar` 4~6 Hz（无 GUI 也一样）、`/livox/imu` ~100 Hz、`/gps/data` ~16 Hz、`/foot_contacts` 50 Hz
- Gazebo 窗口能正常渲染世界 + 机器狗，底部状态栏有 `Sim Time / FPS / Real Time Factor`

## 六、遗留（未做，按需）

1. **装到系统路径**：`sudo make install`（前缀改 `/usr/local`）后需确认
   `/etc/ld.so.conf` 顺序，避免与 PPA 那套混用；或继续用 `env_gazebo_fixed.sh`。
2. **重编工作空间里链 gazebo 的插件**（`mid360_simulation`、`/home/wtr/projects/gazebo_ros_pkgs`）：
   现有二进制是在旧（打补丁）头文件下编译的，其内联单例仍是每库一份。影响面很小
   （目前只有 mid360 插件的调试用 gazebo 话题），但为了完全一致建议重编，
   编译时把 `CMAKE_PREFIX_PATH` 指到 `$HOME/gzbuild/prefix`。
3. **spawn 时序**：`champ_gazebo/launch/gazebo.launch.py` 只用 `TimerAction(period=2.0)`
   就去 spawn，而 world + Mid-360 插件（解析 80 万行 CSV）要 ~10-15 秒才就绪，
   曾观察到 `Entity pushed to spawn queue, but spawn service timed out`。可把 2.0 调大。
4. **足端接触**：`/foot_contacts` 目前恒为 false（机器人未给 /cmd_vel 时腿基本趴着，
   接触点不在足端），跑完整导航（有 /cmd_vel）时再确认。
