from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
import os


def _as_bool(value):
    return value.strip().lower() in ('true', '1', 'yes', 'on')


def _as_float(name, value):
    try:
        return float(value)
    except ValueError as exc:
        raise RuntimeError(f'Invalid float value for {name}: {value}') from exc


def _profile_path(package_share, kind, name):
    path = os.path.join(package_share, 'config', kind, f'{name}.yaml')
    if not os.path.isfile(path):
        raise RuntimeError(
            f'Unknown {kind} profile "{name}": {path} does not exist. '
            f'Add a profile file under config/{kind}/ or choose an existing one.')
    return path


def launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory('robotdog_nav2_bringup')

    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    autostart_str = LaunchConfiguration('autostart').perform(context)
    use_sim_time = _as_bool(use_sim_time_str)
    autostart = _as_bool(autostart_str)

    controller_name = LaunchConfiguration('controller').perform(context)
    planner_name = LaunchConfiguration('planner').perform(context)
    controller_profile = _profile_path(package_share, 'controller', controller_name)
    planner_profile = _profile_path(package_share, 'planner', planner_name)

    common_params_file = LaunchConfiguration('common_params_file').perform(context)
    bridge_params_file = LaunchConfiguration('bridge_params_file').perform(context)
    if not os.path.isfile(common_params_file):
        raise RuntimeError(f'common_params_file does not exist: {common_params_file}')
    if not os.path.isfile(bridge_params_file):
        raise RuntimeError(f'bridge_params_file does not exist: {bridge_params_file}')

    origin_lat = _as_float('origin_lat', LaunchConfiguration('origin_lat').perform(context))
    origin_lon = _as_float('origin_lon', LaunchConfiguration('origin_lon').perform(context))
    origin_alt = _as_float('origin_alt', LaunchConfiguration('origin_alt').perform(context))

    configured_common = RewrittenYaml(
        source_file=common_params_file,
        root_key='',
        param_rewrites={'use_sim_time': use_sim_time_str},
        convert_types=True)

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    # controller_server 订阅 cmd_vel_nav 之前先发布内部速度；velocity_smoother
    # 订阅 cmd_vel_nav 并把平滑结果 remap 回 /cmd_vel，保持与 nav2_bringup 一致。
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_common, controller_profile],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[configured_common],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_common, planner_profile],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_common],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_common],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_common],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_common],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes,
            'bond_timeout': 10.0,
            'attempt_respawn_reconnection': True,
        }],
    )

    multi_goal_bridge = Node(
        package='robotdog_nav2_bringup',
        executable='multi_goal_bridge_node',
        name='multi_goal_bridge_node',
        output='screen',
        parameters=[
            bridge_params_file,
            {
                'use_sim_time': use_sim_time,
                'origin_lat': origin_lat,
                'origin_lon': origin_lon,
                'origin_alt': origin_alt,
            },
        ],
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(package_share, 'config', 'rviz', 'nav2_world.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return [
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        # 等待各节点服务完成 DDS 发现后再开始 lifecycle 配置，避免首次启动时
        # 偶发的 change_state 服务响应超时。
        TimerAction(period=2.0, actions=[lifecycle_manager]),
        multi_goal_bridge,
        rviz_node,
    ]


def generate_launch_description():
    package_share = get_package_share_directory('robotdog_nav2_bringup')

    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation/Gazebo clock'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically configure and activate Nav2 lifecycle nodes'),
        DeclareLaunchArgument(
            'controller',
            default_value='rpp',
            description='Controller profile under config/controller/ (rpp, dwb, mppi, ...)'),
        DeclareLaunchArgument(
            'planner',
            default_value='navfn',
            description='Planner profile under config/planner/ (navfn, smac, theta, ...)'),
        DeclareLaunchArgument(
            'simulation',
            default_value='true',
            description='Set false on the real robot to publish base_link -> livox static TF'),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz2 with the Nav2 world view (config/rviz/nav2_world.rviz)'),
        DeclareLaunchArgument(
            'common_params_file',
            default_value=os.path.join(package_share, 'config', 'common.yaml'),
            description='Common Nav2 parameters file'),
        DeclareLaunchArgument(
            'bridge_params_file',
            default_value=os.path.join(package_share, 'config', 'multi_goal_bridge.yaml'),
            description='MultiGoal bridge parameters file'),
        DeclareLaunchArgument(
            'origin_lat',
            default_value='0.0',
            description='GPS origin latitude, must match gps_imu_fusion'),
        DeclareLaunchArgument(
            'origin_lon',
            default_value='0.0',
            description='GPS origin longitude, must match gps_imu_fusion'),
        DeclareLaunchArgument(
            'origin_alt',
            default_value='0.0',
            description='GPS origin altitude, must match gps_imu_fusion'),
    ]

    static_tf_publisher = Node(
        condition=UnlessCondition(LaunchConfiguration('simulation')),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox_static_tf_publisher',
        output='screen',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.1',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'livox',
        ],
    )

    return LaunchDescription(
        declared_arguments + [
            OpaqueFunction(function=launch_setup),
            static_tf_publisher,
        ]
    )
