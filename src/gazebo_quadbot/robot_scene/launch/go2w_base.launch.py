from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
import os
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

def generate_launch_description():
    pkg_go2w = FindPackageShare("go2w_description")
    pkg_robot_scene = FindPackageShare("robot_scene")
    xacro_file = PathJoinSubstitution([pkg_robot_scene, "xacro", "go2w_description.urdf.xacro"])
    controller_yaml = PathJoinSubstitution([pkg_go2w, "config", "joint_controller.yaml"])

    # 添加 spawn 位置参数声明（与 Go2 一致）
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.75")
    declare_world_init_heading = DeclareLaunchArgument("world_init_heading", default_value="0.0")

    declare_world = DeclareLaunchArgument(
        name="world",
        default_value="default.world",
        description="World file name to load from the worlds directory"
    )

    # 阻止Gazebo从互联网自动下载模型（加速本地加载）
    os.environ['GAZEBO_MODEL_DATABASE_URI'] = ""
    # Gazebo 靠 GAZEBO_MODEL_PATH 解析 model://：sdformat 会把 URDF 里 mesh 的
    # package://<pkg>/... 自动改写成 model://<pkg>/...。在这里补上各 mesh 包
    # share 的父目录（等价于 package.xml 里 gazebo_model_path 的 ${prefix}/..），
    # 搜索路径方法与 go2_lidar_gps.launch.py 保持一致。
    # ~/.gazebo/models 与 /usr/share/gazebo-11/models 是 Gazebo 内置默认路径，无需显式加入
    gazebo_model_paths = []
    for mesh_pkg in ("go2w_description", "mid360_simulation"):
        try:
            gazebo_model_paths.append(os.path.dirname(get_package_share_directory(mesh_pkg)))
        except PackageNotFoundError:
            pass
    if os.environ.get("GAZEBO_MODEL_PATH"):
        gazebo_model_paths.append(os.environ["GAZEBO_MODEL_PATH"])
    os.environ["GAZEBO_MODEL_PATH"] = os.pathsep.join(gazebo_model_paths)

    # 添加GPU渲染优化环境变量
    mesa_adapter = SetEnvironmentVariable(
        name='MESA_D3D12_DEFAULT_ADAPTER_NAME',
        value='NVIDIA'
    )
    gazebo_gpu_rendering = SetEnvironmentVariable(
        name='GAZEBO_GPU_RENDERING',
        value='1'
    )

    gzserver_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("gazebo_ros"), "/launch", "/gzserver.launch.py"
            ]),
            launch_arguments={
                "world": LaunchConfiguration("world"),
                'extra_gazebo_args': '--disable-online-model-retrieval --disable-audio',
            }.items()
        )
    gzclient_launch = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command([FindExecutable(name="xacro"), " ", xacro_file]),
                "use_sim_time": True
            }
        ]
    )

    # 使用 LaunchConfiguration 参数而不是硬编码值
    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "go2w",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("world_init_x"),
            "-y", LaunchConfiguration("world_init_y"),
            "-z", LaunchConfiguration("world_init_z"),
            "-R", "0",
            "-P", "0",
            "-Y", LaunchConfiguration("world_init_heading"),
        ],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--param-file", controller_yaml],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    joint_group_effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_effort_controller', '--controller-manager', '/controller_manager', "--param-file", controller_yaml],
        output='screen',
        parameters=[{"use_sim_time": True}]
    )

    joint_group_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_velocity_controller', '--controller-manager', '/controller_manager', "--param-file", controller_yaml],
        output='screen',
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        mesa_adapter,
        gazebo_gpu_rendering,
        declare_world,
        declare_world_init_x,
        declare_world_init_y,
        declare_world_init_z,
        declare_world_init_heading,
        gzserver_launch,
        gzclient_launch,
        robot_state_publisher_node,
        # 先启动控制器（延迟 1 秒让它们初始化）
        TimerAction(
            period=1.0,
            actions=[
                joint_state_broadcaster_spawner,
                joint_group_effort_controller_spawner,
                joint_group_velocity_controller_spawner,
            ]
        ),
        # 再 spawn 机器人（延迟 3 秒，确保控制器完全就绪）
        TimerAction(
            period=3.0,
            actions=[
                spawn_entity_node,
            ]
        ),
    ])
