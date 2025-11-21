from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import PythonExpression, FileContent, LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    model= LaunchConfiguration('model')
    use_jsp_gui = LaunchConfiguration('use_jsp_gui')
    use_jsp = LaunchConfiguration('use_jsp')
    use_rviz = LaunchConfiguration('use_rviz')
    use_controllers = LaunchConfiguration('use_controllers')
    world_file = LaunchConfiguration('world_file')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_rsp = LaunchConfiguration('use_rsp')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable or disable usage of simulation clock (gazebo)'
    )

    declare_model = DeclareLaunchArgument(
        'model',
        default_value='alt_model',
        description='Model to load, chose either "alt_model" or "model"'
    )

    declare_use_jsp_gui = DeclareLaunchArgument(
        'use_jsp_gui',
        default_value= 'false',
        choices=['true', 'false'],
        description='Flag to enable or disable the usage of JointStatePublisher GUI'
    )

    declare_use_jsp = DeclareLaunchArgument(
        'use_jsp',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable or disable the usage of JointStatePublisher'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable or disable the usage of Rviz'
    )

    declare_use_controllers = DeclareLaunchArgument(
        'use_controllers',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable loading of ROS2_controllers'
    )

    declare_world = DeclareLaunchArgument(
        'world_file',
        default_value='empty.sdf',
        description='World file name that should be used'
    )

    declare_use_gazebo = DeclareLaunchArgument(
        'use_gazebo',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable usage of Gazebo'
    )

    declare_use_rsp = DeclareLaunchArgument(
        'use_rsp',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable or disable usage of RobotStatePublisher'
    )

    this_package = FindPackageShare('human_arm_model')

    modelpath = PathJoinSubstitution([
        this_package, 
        'urdf', 
        [LaunchConfiguration('model'), '.urdf.xacro'] 
    ])

    urdf = ParameterValue(Command(['xacro ', modelpath]), value_type=str)

    rviz= PathJoinSubstitution([
        this_package,
        'config',
        [LaunchConfiguration('model'), '.rviz']
    ])

    config_file = PathJoinSubstitution([
        this_package,
        'config',
        'config.yaml'
    ])

    start_robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                this_package,
                'launch',
                'display.launch.py'
            ])
        ]), 
        launch_arguments={
            'use_sim_time': use_sim_time,
            'model': model,
            'use_jsp_gui': use_jsp_gui,
            'use_jsp': use_jsp,
            'use_rviz': use_rviz
        }.items(),
        condition = IfCondition(use_rsp)
    )

    load_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                this_package,
                'launch',
                'ros2_controllers.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_controllers)
    )

    start_gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': world_file
        }.items(),
        condition= IfCondition(use_gazebo)
    )

    start_ros_gazebo_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': config_file,
        }],
        output='screen'
    )

    spawn_model_node = Node(
        package='ros_gz_sim',
        executable= 'create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'human_arm_model',
            '-allow_renaming', 'true'  
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_model,
        declare_use_jsp_gui,
        declare_use_jsp,
        declare_use_rviz,
        declare_use_controllers,
        declare_world,
        declare_use_gazebo,
        declare_use_rsp,
        start_robot_state_publisher,
        load_controllers,
        start_gazebo_node,
        start_ros_gazebo_bridge_node,
        spawn_model_node,
    ])