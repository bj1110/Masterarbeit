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
    this_package = FindPackageShare('nullspace_solver_moveit_config')

    modelpath = PathJoinSubstitution([
        this_package,
        'urdf',
        'model.urdf.xacro'
    ])
    urdf= ParameterValue(Command(['xacro ', modelpath]), value_type=str)
    

    moveit_config = MoveItConfigsBuilder("model", package_name="nullspace_solver_moveit_config") \
        .robot_description(file_path="config/model.urdf.xacro") \
        .robot_description_semantic(file_path="config/model.srdf") \
        .robot_description_kinematics(file_path="config/kinematics.yaml") \
        .joint_limits(file_path="config/joint_limits.yaml") \
        .to_moveit_configs()

    start_sim = Node(
        package='nullspace_solver_moveit_config',
        executable='test_solver',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            {'urdf' : urdf }
        ],
    )


    return LaunchDescription([
        start_sim
    ])