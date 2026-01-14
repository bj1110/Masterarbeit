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
    moveit_package = FindPackageShare("moveit_config")

    moveit_config = MoveItConfigsBuilder("alt_human_arm_model", package_name="moveit_config") \
        .robot_description(file_path="config/alt_human_arm_model.urdf.xacro") \
        .robot_description_semantic(file_path="config/alt_human_arm_model.srdf") \
        .robot_description_kinematics(file_path="config/kinematics.yaml") \
        .to_moveit_configs()

    start_parser_node = Node(
        package='ik_processing',
        executable='parser'
    )

    start_sim = Node(
        package='ik_processing',
        executable='fullsim',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
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