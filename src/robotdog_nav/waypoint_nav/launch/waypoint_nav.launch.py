from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    simulation = LaunchConfiguration('simulation')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    declare_params_file_argument = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('waypoint_nav'),
            'config',
            'params.yaml'
        ),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_simulation_argument = DeclareLaunchArgument(
        'simulation',
        default_value='true',
        description='Enable simulation mode')

    # 定位层已解耦：gps_imu_fusion 不再由本 launch 包含，需单独启动
    # （ros2 launch gps_imu_fusion gps_imu_fusion.launch.py 或仓库根目录 ./start_system.sh），
    # 层间仅通过 /world_odom 与 TF world→base_link 通信

    # RViz2 node to visualize navigation
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('waypoint_nav'),
            'config',
            'nav_visualization.rviz'
        )],
        output='screen'
    )

    # Navigation2 controller server node - now configured to handle timing issues
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        # arguments=['--ros-args', '--log-level', 'debug']
    )

    waypoint_nav_node = Node(
        package='waypoint_nav',
        executable='nav_waypoint_node',
        name='nav_waypoint_node',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('waypoint_nav'),
                'config',
                'params.yaml'
            ),
            # 注入仿真时间：回放 bag 时节点自身时间须与仿真钟一致，
            # 否则发布的 markers/path 带壁钟时间戳，RViz 视为“来自未来”而不显示
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # ('/world_odom', '/mavros/local_position/odom'),
        ]
    )

    # Lifecyle manager to manage the navigation2 nodes
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['controller_server']}]
    )

    # Static transform publisher for base_link to livox in non-simulation mode
    static_tf_publisher = Node(
        condition=UnlessCondition(simulation),  # Only run when simulation is false
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox_static_tf_publisher',
        output='screen',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.1',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'livox']
    )

    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_params_file_argument,
        declare_simulation_argument,
        rviz_node,
        # External Nav2 controller server
        controller_server_node,
        waypoint_nav_node,
        lifecycle_manager,
        static_tf_publisher
    ])