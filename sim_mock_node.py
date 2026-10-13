#!/usr/bin/env python3
"""
RobotDog Nav Simulation Mock Node
模拟仿真端，用于验证三个话题的发布/订阅关系：
  Pub:  /robot_1/mavros/odom     (nav_msgs/Odometry)   30Hz
  Pub:  /robot_1/livox/lidar     (sensor_msgs/PointCloud2)  10Hz
  Sub:  /robot_1/cmd_vel         (geometry_msgs/Twist)
运动学：接收 cmd_vel → 积分得到位置 → 发布里程计
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseWithCovariance, TwistWithCovariance
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import math
import struct


class RobotSimMockNode(Node):
    def __init__(self):
        super().__init__('robot_sim_mock')

        # --- 话题接口 ---
        self.odom_pub = self.create_publisher(Odometry, '/robot_1/mavros/odom', 10)
        self.lidar_pub = self.create_publisher(PointCloud2, '/robot_1/livox/lidar', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/robot_1/cmd_vel', self.cmd_vel_callback, 10
        )

        # --- 定时器 ---
        self.create_timer(1.0 / 30.0, self.publish_odom)   # 30Hz
        self.create_timer(1.0 / 10.0, self.publish_lidar)  # 10Hz

        # --- 状态 ---
        self.cmd_vel = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # rad

        self.get_logger().info('=' * 50)
        self.get_logger().info('Robot Sim Mock Node started')
        self.get_logger().info('  Pub: /robot_1/mavros/odom     @ 30Hz')
        self.get_logger().info('  Pub: /robot_1/livox/lidar    @ 10Hz')
        self.get_logger().info('  Sub: /robot_1/cmd_vel')
        self.get_logger().info('=' * 50)

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel = msg
        self.last_cmd_time = self.get_clock().now()

    def _get_clamped_velocity(self):
        """返回经限幅和超时衰减后的速度"""
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9

        if dt > 0.5:
            decay = math.exp(-2.0 * (dt - 0.5))
            vx = self.cmd_vel.linear.x * decay
            vy = self.cmd_vel.linear.y * decay
            vz = self.cmd_vel.angular.z * decay
        else:
            vx = self.cmd_vel.linear.x
            vy = self.cmd_vel.linear.y
            vz = self.cmd_vel.angular.z

        return (
            max(-1.5, min(1.5, vx)),
            max(-0.6, min(0.6, vy)),
            max(-1.0, min(1.0, vz)),
        )

    def publish_odom(self):
        vx, vy, vz = self._get_clamped_velocity()
        dt = 1.0 / 30.0

        # Simple motion model (2D robot)
        self.yaw += vz * dt
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (vx * cos_yaw - vy * sin_yaw) * dt
        self.y += (vx * sin_yaw + vy * cos_yaw) * dt

        # Build Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'robot_1/odom'
        odom.child_frame_id = 'robot_1/base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vz

        self.odom_pub.publish(odom)

    def publish_lidar(self):
        """生成一个简单的虚拟点云（8 个点，半径 ~2m）"""
        num_points = 8
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 16

        pts = []
        for i in range(num_points):
            angle = i * 2 * math.pi / num_points + self.get_clock().now().nanoseconds * 1e-9 * 0.5
            dist = 2.0 + math.sin(angle * 2) * 0.3
            pts.extend([
                dist * math.cos(angle),
                dist * math.sin(angle),
                math.sin(angle * 3) * 0.2,
                50.0 + math.sin(angle) * 30.0,
            ])

        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = 'robot_1/livox_frame'
        cloud.height = 1
        cloud.width = num_points
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = point_step
        cloud.row_step = point_step * num_points
        cloud.data = struct.pack(f'<{4 * num_points}f', *pts)
        cloud.is_dense = True

        self.lidar_pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = RobotSimMockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()