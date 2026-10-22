"""虚拟场景 launch：mock_scene_node（墙钟）+ odom_to_tf + 静态 base_link→livox。

用法（无 Gazebo/bag 的 RViz 闭环验证）：
    ros2 launch nav_replay_support mock_scene.launch.py

时间方案：mock 节点自身 use_sim_time 恒为 false（墙钟），它发布 /clock；
odom_to_tf / 静态 TF 以 use_sim_time:=true 运行（仿 bag play --clock）。

互斥规则（违反会产生双发布者冲突）：
- 禁止与 Gazebo、`ros2 bag play --clock`、gps_imu_fusion 同跑（/clock、/world_odom、
  world→base_link 各只允许一个发布者）；
- 导航 launch 保持默认 simulation:=true，不要传 simulation:=false
  （静态 base_link→livox 由本 launch 的 replay_tf 部分持有）；
- mock 节点崩溃即仿真时钟冻结，重启 mock 即可（墙钟不回跳，无需重启导航栈）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    odom_topic = LaunchConfiguration('odom_topic')
    params_file = LaunchConfiguration('params_file')
    publish_clock = LaunchConfiguration('publish_clock')
    publish_static_livox_tf = LaunchConfiguration('publish_static_livox_tf')
    livox_x = LaunchConfiguration('livox_x')
    livox_y = LaunchConfiguration('livox_y')
    livox_z = LaunchConfiguration('livox_z')

    declare_args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('nav_replay_support'),
                'config', 'mock_scene.yaml'),
            description='Path to the mock scene parameters YAML'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/world_odom',
            description='Odometry topic produced by the mock robot'),
        DeclareLaunchArgument(
            'publish_clock', default_value='true',
            description='Let the mock node publish /clock (must stay the only publisher)'),
        DeclareLaunchArgument(
            'publish_static_livox_tf', default_value='true',
            description='Publish static base_link -> livox TF'),
        DeclareLaunchArgument(
            'livox_x', default_value='0.0',
            description='base_link -> livox static TF x offset'),
        DeclareLaunchArgument(
            'livox_y', default_value='0.0',
            description='base_link -> livox static TF y offset'),
        DeclareLaunchArgument(
            'livox_z', default_value='0.1',
            description='base_link -> livox static TF z offset'),
    ]

    # 虚拟场景节点：墙钟运行（use_sim_time 恒 false，/clock 即墙钟时间）
    mock_scene_node = Node(
        package='nav_replay_support',
        executable='mock_scene_node.py',
        name='mock_scene_node',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': False,
             'odom_topic': odom_topic,
             'publish_clock': publish_clock},
        ],
    )

    # 复用回放 TF 链：odom_to_tf（/world_odom → TF world→base_link）+ 静态 livox 外参
    # 注意：use_sim_time 传字面 'true'（消费 /clock），不是本 launch 的参数
    include_replay_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav_replay_support'),
            'launch', 'replay_tf.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'odom_topic': odom_topic,
            'publish_static_livox_tf': publish_static_livox_tf,
            'livox_x': livox_x,
            'livox_y': livox_y,
            'livox_z': livox_z,
        }.items(),
    )

    return LaunchDescription(declare_args + [mock_scene_node, include_replay_tf])
