#!/usr/bin/env python3
# 虚拟场景节点：无 Gazebo 的轻量闭环验证数据源（中文注释，英文日志）。
#
# 三合一职责：
#   1. /clock     —— 墙钟时间 50 Hz（本节点自身必须 use_sim_time:=false，
#                    其余节点 use_sim_time:=true，等效 `ros2 bag play --clock`）
#   2. /world_odom —— /cmd_vel 全向运动学积分成位姿（闭环：机器人真的会走）
#   3. /livox/lidar —— 合成障碍场景点云（按传感器量程过滤后变换到 livox 系）
#
# TF 不由本节点发布：复用 odom_to_tf_node 从 /world_odom 重建（replay_tf.launch.py）。
# 互斥规则：本节点是唯一 /clock、唯一 /world_odom、唯一 world→base_link 来源，
# 禁止与 Gazebo、`ros2 bag play --clock`、gps_imu_fusion 同跑。

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


class MockSceneNode(Node):

    def __init__(self):
        super().__init__('mock_scene_node')

        # ---- 话题与频率 ----
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/world_odom')
        self.declare_parameter('lidar_topic', '/livox/lidar')
        self.declare_parameter('clock_topic', '/clock')
        self.declare_parameter('marker_topic', '/mock_scene/markers')
        self.declare_parameter('clock_rate', 50.0)
        self.declare_parameter('odom_rate', 10.0)
        self.declare_parameter('lidar_rate', 10.0)
        self.declare_parameter('publish_clock', True)
        self.declare_parameter('publish_markers', True)

        # ---- 坐标系 ----
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('sensor_frame', 'livox')
        self.declare_parameter('sensor_z', 0.1)  # livox 在 base_link 上方的高度

        # ---- 运动学限幅与超时衰减 ----
        self.declare_parameter('max_linear_vel', 1.5)
        self.declare_parameter('max_lateral_vel', 0.6)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('cmd_decay_rate', 2.0)

        # ---- 初始位姿 ----
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        # ---- 点云采样 ----
        self.declare_parameter('sensor_range', 5.0)
        self.declare_parameter('min_sensor_range', 0.2)
        self.declare_parameter('point_spacing', 0.1)
        self.declare_parameter('z_sample_min', 0.45)
        self.declare_parameter('z_sample_max', 0.95)
        self.declare_parameter('max_points', 5000)
        self.declare_parameter('point_intensity', 100.0)

        # ---- 障碍场景：扁平 double 数组 ----
        #   box_obstacles      每项 [cx, cy, cz_base, dx, dy, dz]（中心 xy、底面 z、尺寸）
        #   cylinder_obstacles 每项 [cx, cy, cz_base, r, h]
        self.declare_parameter('box_obstacles', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('cylinder_obstacles', Parameter.Type.DOUBLE_ARRAY)

        # ---- 边界校验集中在构造处（内部实现信任已校验数据）----
        self.boxes_ = self._parse_obstacles('box_obstacles', 6, 'cx,cy,cz_base,dx,dy,dz')
        self.cylinders_ = self._parse_obstacles('cylinder_obstacles', 5, 'cx,cy,cz_base,r,h')
        for item in self.boxes_:
            if item[3] <= 0.0 or item[4] <= 0.0 or item[5] <= 0.0:
                raise RuntimeError('box_obstacles dimension must be positive: %s' % (item,))
        for item in self.cylinders_:
            if item[3] <= 0.0 or item[4] <= 0.0:
                raise RuntimeError('cylinder_obstacles radius/height must be positive: %s' % (item,))

        get = lambda name: self.get_parameter(name).value
        self.z_sample_min_ = get('z_sample_min')
        self.z_sample_max_ = get('z_sample_max')
        self.point_spacing_ = get('point_spacing')
        if not 0.4 < self.z_sample_min_ < self.z_sample_max_ < 1.0:
            raise RuntimeError('z_sample range must be inside (0.4, 1.0): %s..%s'
                               % (self.z_sample_min_, self.z_sample_max_))
        if self.point_spacing_ <= 0.0:
            raise RuntimeError('point_spacing must be positive')

        # ---- 机器人状态（墙钟时间驱动）----
        self.x_ = get('initial_x')
        self.y_ = get('initial_y')
        self.yaw_ = get('initial_yaw')
        self.max_lin_ = get('max_linear_vel')
        self.max_lat_ = get('max_lateral_vel')
        self.max_ang_ = get('max_angular_vel')
        self.cmd_timeout_ = get('cmd_timeout')
        self.cmd_decay_rate_ = get('cmd_decay_rate')
        self.last_cmd_ = None
        self.last_cmd_sec_ = 0.0
        self.last_tick_sec_ = self._now_sec()

        # ---- 障碍世界系点缓存（启动一次成型）----
        self.world_points_ = self._build_world_points()
        self.intensity_ = get('point_intensity')
        self.fields_ = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # ---- 发布/订阅 ----
        self.clock_pub_ = self.create_publisher(Clock, get('clock_topic'), 10)
        self.odom_pub_ = self.create_publisher(Odometry, get('odom_topic'), 10)
        # 点云必须 RELIABLE：nav_core 以默认可靠 QoS 订阅，Nav2 障碍层 best-effort，
        # 可靠发布同时兼容两者与两套 RViz 显示
        self.lidar_pub_ = self.create_publisher(PointCloud2, get('lidar_topic'), 10)
        self.marker_pub_ = self.create_publisher(
            MarkerArray, get('marker_topic'),
            QoSProfile(depth=1,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=QoSReliabilityPolicy.RELIABLE))
        self.cmd_sub_ = self.create_subscription(
            Twist, get('cmd_vel_topic'), self.cmdCallback, 10)

        # ---- 定时器 ----
        self.create_timer(1.0 / get('clock_rate'), self.onClock)
        self.create_timer(1.0 / get('odom_rate'), self.onOdom)
        self.create_timer(1.0 / get('lidar_rate'), self.onLidar)

        # 场景 markers 一次发布，Transient Local 锁存（晚开的 RViz 也能看到）
        if get('publish_markers'):
            self.publishMarkers()

        self.get_logger().info(
            'mock_scene_node ready: boxes=%d cylinders=%d points=%d, '
            'initial=(%.2f, %.2f, %.1f deg), sensor_range=%.1f m'
            % (len(self.boxes_), len(self.cylinders_), len(self.world_points_),
               self.x_, self.y_, math.degrees(self.yaw_), get('sensor_range')))

    # ---------------------------------------------------------------- 参数解析

    def _parse_obstacles(self, name, stride, meaning):
        raw = self.get_parameter(name).value
        if raw is None:
            raw = []
        if len(raw) % stride != 0:
            raise RuntimeError('%s length %d not a multiple of %d (%s)'
                               % (name, len(raw), stride, meaning))
        return [tuple(raw[i:i + stride]) for i in range(0, len(raw), stride)]

    # ---------------------------------------------------------------- 场景构建

    def _z_layers(self, base, top):
        """障碍 [base, top] 与采样窗 (z_sample_min, z_sample_max) 的交集采样层"""
        lo = max(self.z_sample_min_, base)
        hi = min(self.z_sample_max_, top)
        layers = []
        z = lo
        while z <= hi + 1e-6:
            layers.append(z)
            z += self.point_spacing_
        return layers

    def _build_world_points(self):
        """所有障碍表面点（世界系）缓存。方块取四个侧面网格，圆柱按周长取环。"""
        s = self.point_spacing_
        points = []

        for cx, cy, cz, dx, dy, dz in self.boxes_:
            x0, x1 = cx - dx / 2.0, cx + dx / 2.0
            y0, y1 = cy - dy / 2.0, cy + dy / 2.0
            for z in self._z_layers(cz, cz + dz):
                n_x = max(1, int(round(dx / s))) + 1
                n_y = max(1, int(round(dy / s))) + 1
                for i in range(n_x):
                    t = x0 + (x1 - x0) * i / (n_x - 1)
                    points.append((t, y0, z))
                    points.append((t, y1, z))
                for j in range(n_y):
                    t = y0 + (y1 - y0) * j / (n_y - 1)
                    points.append((x0, t, z))
                    points.append((x1, t, z))

        for cx, cy, cz, r, h in self.cylinders_:
            for z in self._z_layers(cz, cz + h):
                n = max(8, int(round(2.0 * math.pi * r / s)))
                for i in range(n):
                    a = 2.0 * math.pi * i / n
                    points.append((cx + r * math.cos(a), cy + r * math.sin(a), z))

        max_points = self.get_parameter('max_points').value
        if len(points) > max_points:
            step = len(points) / float(max_points)
            points = [points[int(i * step)] for i in range(max_points)]
        return points

    # ---------------------------------------------------------------- 回调与定时

    def cmdCallback(self, msg):
        self.last_cmd_ = msg
        self.last_cmd_sec_ = self._now_sec()

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _applied_velocity(self, now_sec):
        """限幅 + 超时指数衰减后的实际速度（复用 sim_mock_node.py 的模式）"""
        if self.last_cmd_ is None:
            return 0.0, 0.0, 0.0
        silence = now_sec - self.last_cmd_sec_
        decay = 1.0
        if silence > self.cmd_timeout_:
            decay = math.exp(-self.cmd_decay_rate_ * (silence - self.cmd_timeout_))
        vx = max(-self.max_lin_, min(self.max_lin_, self.last_cmd_.linear.x * decay))
        vy = max(-self.max_lat_, min(self.max_lat_, self.last_cmd_.linear.y * decay))
        wz = max(-self.max_ang_, min(self.max_ang_, self.last_cmd_.angular.z * decay))
        if abs(vx) < 0.01:
            vx = 0.0
        if abs(vy) < 0.01:
            vy = 0.0
        if abs(wz) < 0.01:
            wz = 0.0
        return vx, vy, wz

    def onClock(self):
        if self.get_parameter('publish_clock').value:
            msg = Clock()
            msg.clock = self.get_clock().now().to_msg()
            self.clock_pub_.publish(msg)

    def onOdom(self):
        """全向运动学积分并发布里程计（twist 填实际 v/ω，非零）"""
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9
        dt = min(max(now_sec - self.last_tick_sec_, 0.0), 1.0)
        self.last_tick_sec_ = now_sec

        vx, vy, wz = self._applied_velocity(now_sec)
        self.yaw_ += wz * dt
        cos_y = math.cos(self.yaw_)
        sin_y = math.sin(self.yaw_)
        self.x_ += (vx * cos_y - vy * sin_y) * dt
        self.y_ += (vx * sin_y + vy * cos_y) * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.get_parameter('world_frame').value
        odom.child_frame_id = self.get_parameter('base_frame').value
        odom.pose.pose.position.x = self.x_
        odom.pose.pose.position.y = self.y_
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw_ / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw_ / 2.0)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub_.publish(odom)

    def onLidar(self):
        """障碍点按量程过滤后变换到 livox 系发布（p_livox = R(-yaw)(p_w - p_base)）"""
        now = self.get_clock().now()
        r_min = self.get_parameter('min_sensor_range').value
        r_max = self.get_parameter('sensor_range').value
        r_min2 = r_min * r_min
        r_max2 = r_max * r_max
        sensor_z = self.get_parameter('sensor_z').value
        cos_y = math.cos(-self.yaw_)
        sin_y = math.sin(-self.yaw_)

        out = []
        for wx, wy, wz in self.world_points_:
            dx = wx - self.x_
            dy = wy - self.y_
            d2 = dx * dx + dy * dy
            if d2 < r_min2 or d2 > r_max2:
                continue
            out.append((cos_y * dx - sin_y * dy,
                        sin_y * dx + cos_y * dy,
                        wz - sensor_z,
                        self.intensity_))

        header = Header()
        header.stamp = now.to_msg()
        header.frame_id = self.get_parameter('sensor_frame').value
        self.lidar_pub_.publish(point_cloud2.create_cloud(header, self.fields_, out))

    def publishMarkers(self):
        """场景 markers 一次发布：CUBE/CYLINDER，半透明蓝色"""
        now = self.get_clock().now().to_msg()
        world = self.get_parameter('world_frame').value
        array = MarkerArray()
        for idx, (cx, cy, cz, dx, dy, dz) in enumerate(self.boxes_):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = world
            m.ns = 'mock_scene'
            m.id = idx
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = cz + dz / 2.0
            m.pose.orientation.w = 1.0
            m.scale.x = dx
            m.scale.y = dy
            m.scale.z = dz
            m.color.r = 85.0 / 255.0
            m.color.g = 170.0 / 255.0
            m.color.b = 1.0
            m.color.a = 0.35
            array.markers.append(m)
        offset = len(self.boxes_)
        for idx, (cx, cy, cz, r, h) in enumerate(self.cylinders_):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = world
            m.ns = 'mock_scene'
            m.id = offset + idx
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = cz + h / 2.0
            m.pose.orientation.w = 1.0
            m.scale.x = 2.0 * r
            m.scale.y = 2.0 * r
            m.scale.z = h
            m.color.r = 85.0 / 255.0
            m.color.g = 170.0 / 255.0
            m.color.b = 1.0
            m.color.a = 0.35
            array.markers.append(m)
        self.marker_pub_.publish(array)


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = MockSceneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
