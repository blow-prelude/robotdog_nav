"""通用 odom→TF 适配节点 launch：只启动 odom_to_tf_node，不含 bag play / 静态 TF / 导航栈。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_topic = LaunchConfiguration('odom_topic')
    parent_frame = LaunchConfiguration('parent_frame')
    child_frame = LaunchConfiguration('child_frame')
    params_file = LaunchConfiguration('params_file')

    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
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
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('nav_replay_support'),
                'config', 'params.yaml'),
            description='Path to the node parameters YAML'),
    ]

    odom_to_tf_node = Node(
        package='nav_replay_support',
        executable='odom_to_tf_node',
        name='odom_to_tf_node',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time,
             'odom_topic': odom_topic,
             'parent_frame': parent_frame,
             'child_frame': child_frame},
        ],
    )

    return LaunchDescription(declare_args + [odom_to_tf_node])
