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

def generate_launch_description():
    this_package = FindPackageShare('ik_processing')

    start_parser_node = Node(
        package='ik_processing',
        executable='parser'
    )

    start_sim = Node(
        package='ik_processing',
        executable='fullsim'
    )

    begin_sim=RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_parser_node,
            on_exit=[start_sim]
        )
    )

    delay = TimerAction(
        period=5.0,
        actions=[start_sim]
    )

    return LaunchDescription([
        start_parser_node,
        delay,
    ])