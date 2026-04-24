from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable, RegisterEventHandler, TimerAction, EmitEvent
from launch.substitutions import PythonExpression, FileContent, LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    this_package = FindPackageShare('ik_processing')
    launch_dir = PathJoinSubstitution([this_package, 'launch'])

    launch_parser = IncludeLaunchDescription(
        PathJoinSubstitution([launch_dir, 'parser.launch.py'])
    )

    launch_simulation=IncludeLaunchDescription(
        PathJoinSubstitution([launch_dir, 'sim.launch.py'])
    )

    delayed_simulation_start = TimerAction(
        period=5.0,
        actions=[launch_simulation]
    )

    launch_moveit = IncludeLaunchDescription(
        PathJoinSubstitution([launch_dir, 'moveit.launch.py'])
    )

    return LaunchDescription([
        launch_parser,
        launch_moveit,
        delayed_simulation_start,
    ])