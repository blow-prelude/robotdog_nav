# Launch file for ExecuteTask Server

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Declare launch arguments
    agent_id_arg = DeclareLaunchArgument(
        'agent_id',
        default_value='uav1',
        description='Agent ID for the execute_task service'
    )

    waypoint_frame_arg = DeclareLaunchArgument(
        'waypoint_frame',
        default_value='gps',
        description='Default waypoint frame (gps or local)'
    )

    # ExecuteTask node
    execute_task_node = Node(
        package='execute_task_server',
        executable='execute_task_node',
        name='execute_task_node',
        output='screen',
        parameters=[
            {'agent_id': LaunchConfiguration('agent_id')},
            {'default_waypoint_frame': LaunchConfiguration('waypoint_frame')},
            {'default_velocity': 1.0},
            {'default_yaw': 0.0},
        ]
    )

    return LaunchDescription([
        agent_id_arg,
        waypoint_frame_arg,
        execute_task_node,
    ])
