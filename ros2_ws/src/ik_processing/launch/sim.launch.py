from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable, RegisterEventHandler, TimerAction
from launch.substitutions import PythonExpression, FileContent, LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    this_package = FindPackageShare('ik_processing')
    model_package = FindPackageShare('human_arm_model')

    modelpath = PathJoinSubstitution([
        model_package,
        'urdf',
        'alt_model.urdf.xacro'
    ])
    urdf= ParameterValue(Command(['xacro ', modelpath]), value_type=str)
    
    recalculate=LaunchConfiguration('recalculate')
    use_rviz=LaunchConfiguration('use_rviz')
    config_file=LaunchConfiguration('config_file')
    grid_search=LaunchConfiguration('grid_search')

 
    declare_recalculate=DeclareLaunchArgument(
        'recalculate',
        default_value='true',
        choices=['true', 'false'],
        description='Bool indicating if previously calculated Path should be used or recalculated and be overridden'
    )

    declare_use_rviz=DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Bool indicating if calculated path should be displayed in RVIZ'
    )

    declare_config_file=DeclareLaunchArgument(
        'config_file',
        default_value= PathJoinSubstitution([
            this_package,
            'config',
            "config.yaml"
        ]),
        description='Path to the yaml config file'
    )

    declare_grid_search=DeclareLaunchArgument(
        'grid_search',
        default_value='false',
        choices=['true', 'false'],
        description='Flag to indicate grid_search and thus allowing dumping into special file'
    )

    moveit_config = MoveItConfigsBuilder("alt_human_arm_model", package_name="moveit_config") \
        .robot_description(file_path="config/alt_human_arm_model.urdf.xacro") \
        .robot_description_semantic(file_path="config/alt_human_arm_model.srdf") \
        .robot_description_kinematics(file_path="config/kinematics.yaml") \
        .joint_limits(file_path="config/joint_limits.yaml") \
        .planning_pipelines( "stomp", ["stomp"])\
        .to_moveit_configs()


    start_sim = Node(
        package='ik_processing',
        executable='fullsim',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            {'recalculate':recalculate},
            {'urdf' : urdf },
            config_file,
            {'use_rviz':use_rviz},
            {'grid_search': grid_search}
        ],
    )

    return LaunchDescription([
        declare_recalculate,
        declare_use_rviz,
        declare_config_file,
        declare_grid_search,
        start_sim,
    ])