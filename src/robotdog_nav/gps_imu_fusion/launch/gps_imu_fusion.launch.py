from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )

    config = os.path.join(
        get_package_share_directory('gps_imu_fusion'),
        'config',
        'params.yaml'
    )

    gps_imu_fusion_node = Node(
        package='gps_imu_fusion',
        executable='gps_imu_fusion_node',
        name='gps_imu_fusion_node',
        output='screen',
        parameters=[
            config,  # Load parameters from YAML file
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/imu', '/livox/imu'),  # Map imu/data to /imu/data for IMU data
            ('/gps/data', '/gps/data'),  # Map gps/data to /gps/data for GPS data
        ]
    )

    return LaunchDescription([
        declare_use_sim_time_argument,
        gps_imu_fusion_node
    ])