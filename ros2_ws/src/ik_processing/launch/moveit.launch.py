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
    launch_dir = PathJoinSubstitution([this_package, 'launch'])
    moveit_package = FindPackageShare('moveit_config')
    moveit_launch_dir = PathJoinSubstitution([moveit_package, 'launch'])

    moveit_config = MoveItConfigsBuilder("alt_human_arm_model", package_name="moveit_config").to_moveit_configs()

    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        choices=['true','false'],
        description='Uncheck if no visualization is wanted'
    )

    launch_move_group = IncludeLaunchDescription(
        PathJoinSubstitution([moveit_launch_dir, 'move_group.launch.py'])
    )

    launch_rsp = IncludeLaunchDescription(
        PathJoinSubstitution([moveit_launch_dir, 'rsp.launch.py'])
    )

    launch_controllers = IncludeLaunchDescription(
        PathJoinSubstitution([moveit_launch_dir, 'spawn_controllers.launch.py'])
    )

    launch_static_tfs = IncludeLaunchDescription(
        PathJoinSubstitution([moveit_launch_dir, 'static_virtual_joint_tfs.launch.py'])
    )

    launch_rviz = IncludeLaunchDescription(
        PathJoinSubstitution([moveit_launch_dir, 'moveit_rviz.launch.py']),
        condition=IfCondition(use_rviz)
    )

    launch_manager = Node(
             package="controller_manager",
             executable="ros2_control_node",
             parameters=[
                 moveit_config.robot_description,
                 str(moveit_config.package_path / "config/ros2_controllers.yaml"),
             ],
    )

    return LaunchDescription([
        declare_use_rviz, 
        launch_move_group,
        launch_rsp,
        launch_manager,
        launch_controllers,
        launch_static_tfs,
        launch_rviz,
    ])