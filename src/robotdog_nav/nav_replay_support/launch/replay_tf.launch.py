"""回放专用 launch：odom→TF 适配节点 + 静态 base_link→livox 外参。

用法（回放场景）：
    ros2 launch nav_replay_support replay_tf.launch.py use_sim_time:=true

注意：
- 本 launch 不启动 ros2 bag play、导航栈与任务服务，保持分终端流程；
- 静态 base_link→livox 由本 launch 负责，导航栈保持默认 simulation:=true，
  不要再给导航 launch 传 simulation:=false，否则会出现重复静态 TF；
- 回放时不启动 gps_imu_fusion，TF world→base_link 由 odom_to_tf_node 重建。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_topic = LaunchConfiguration('odom_topic')
    parent_frame = LaunchConfiguration('parent_frame')
    child_frame = LaunchConfiguration('child_frame')
    publish_static_livox_tf = LaunchConfiguration('publish_static_livox_tf')
    livox_x = LaunchConfiguration('livox_x')
    livox_y = LaunchConfiguration('livox_y')
    livox_z = LaunchConfiguration('livox_z')
    livox_roll = LaunchConfiguration('livox_roll')
    livox_pitch = LaunchConfiguration('livox_pitch')
    livox_yaw = LaunchConfiguration('livox_yaw')

    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation/replay clock'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/world_odom',
            description='Odometry topic to convert to TF'),
        DeclareLaunchArgument(
            'parent_frame', default_value='',
            description='Override TF parent frame (empty: use odometry header.frame_id)'),
        DeclareLaunchArgument(
            'child_frame', default_value='',
            description='Override TF child frame (empty: use odometry child_frame_id)'),
        DeclareLaunchArgument(
            'publish_static_livox_tf', default_value='true',
            description='Publish static base_link -> livox TF (replaces robot_state_publisher)'),
        # 默认外参与 Go2 xacro 一致：base_link→trunk 为 0，trunk→livox 为 (0, 0, 0.10)
        DeclareLaunchArgument(
            'livox_x', default_value='0.0',
            description='base_link -> livox static TF x offset'),
        DeclareLaunchArgument(
            'livox_y', default_value='0.0',
            description='base_link -> livox static TF y offset'),
        DeclareLaunchArgument(
            'livox_z', default_value='0.1',
            description='base_link -> livox static TF z offset'),
        DeclareLaunchArgument(
            'livox_roll', default_value='0.0',
            description='base_link -> livox static TF roll'),
        DeclareLaunchArgument(
            'livox_pitch', default_value='0.0',
            description='base_link -> livox static TF pitch'),
        DeclareLaunchArgument(
            'livox_yaw', default_value='0.0',
            description='base_link -> livox static TF yaw'),
    ]

    # 复用通用 launch 启动适配节点
    include_odom_to_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav_replay_support'),
            'launch', 'odom_to_tf.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'odom_topic': odom_topic,
            'parent_frame': parent_frame,
            'child_frame': child_frame,
        }.items(),
    )

    # 仿真之外（bag 回放）没有 robot_state_publisher，需显式发布传感器外参
    static_livox_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox_static_tf_publisher',
        output='screen',
        condition=IfCondition(publish_static_livox_tf),
        arguments=['--x', livox_x, '--y', livox_y, '--z', livox_z,
                   '--roll', livox_roll, '--pitch', livox_pitch, '--yaw', livox_yaw,
                   '--frame-id', 'base_link', '--child-frame-id', 'livox'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(declare_args + [include_odom_to_tf, static_livox_tf])
