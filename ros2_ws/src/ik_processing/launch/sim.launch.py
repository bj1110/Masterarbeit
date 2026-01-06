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
    this_package = FindPackageShare('ik_processing')
    moveit_package = FindPackageShare('config_moveit_model3')

    start_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                moveit_package,
                'launch',
                'demo.launch.py'
            ])
        ])
    )

    start_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                moveit_package,
                'launch',
                'rsp.launch.py'
            ])
        ])
    )

    start_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                moveit_package,
                'launch',
                'spawn_controllers.launch.py'
            ])
        ])
    )

    start_joint_state_broadcaster_cmd = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
      "joint_state_broadcaster",
      "--controller-manager",
      "/controller_manager"
      ]
    )

    start_move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                moveit_package,
                'launch',
                'custom_move_group.launch.py'
            ])
        ])
    )

    start_rviz= IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                moveit_package,
                'launch',
                'moveit_rviz.launch.py'
            ])
        ])
    )

    start_parser_node = Node(
        package='ik_processing',
        executable='parser'
    )

    start_sim = Node(
        package='ik_processing',
        executable='fullsim'
    )

    return LaunchDescription([
        start_parser_node,
        start_rsp,
        start_controllers,
        start_joint_state_broadcaster_cmd,
        start_move_group,
        start_rviz,
        # start_demo, 
        start_sim,
    ])