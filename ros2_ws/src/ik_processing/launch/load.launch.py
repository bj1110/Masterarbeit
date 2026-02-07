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

    is_baseline = LaunchConfiguration('is_baseline')
    is_agent1 = LaunchConfiguration('is_agent1')
    startpos1=LaunchConfiguration('startpos1')
    goalpos1=LaunchConfiguration('goalpos1')
    startpos2=LaunchConfiguration('startpos2')
    goalpos2=LaunchConfiguration('goalpos2')
    datafile=LaunchConfiguration('datafile')
    exNum=LaunchConfiguration('exNum')

    declare_baseline=DeclareLaunchArgument(
        'is_baseline',
        default_value= 'true',
        choices=['true', 'false'],
        description='Bool indicating if baseline or interaction data should be loaded'
    )
    declare_agent= DeclareLaunchArgument(
        'is_agent1',
        default_value='true',
        choices=['true', 'false'],
        description='Bool indicating if agent1 or agent2 data should be loaded'
    )
    declare_startpos1= DeclareLaunchArgument(
        'startpos1',
        default_value='1',
        choices=['1', '3'],
        description= '(short u_)Int indicating the Startposition for agent 1'
    )
    declare_goalpos1=DeclareLaunchArgument(
        'goalpos1',
        default_value='3',
        choices=['1', '3', '4', '5', '6', '7', '8', '9'],
        description='(short u_)Int indicating the Gaolposition for agent 1'
    )
    declare_startpos2= DeclareLaunchArgument(
        'startpos2',
        default_value='5',
        choices=['5', '7'],
        description= '(short u_)Int indicating the Startposition for agent 2'
    )
    declare_goalpos2=DeclareLaunchArgument(
        'goalpos2',
        default_value='3',
        choices=['1', '2', '3', '4', '5', '7', '8', '9'],
        description='(short u_)Int indicating the Gaolposition for agent 2'
    )
    declare_datafile=DeclareLaunchArgument(
        'datafile',
        default_value='1',
        choices=[ str(i) for i in range(1,5)], 
        description='(short u_)Int indicating the experimentfile (in baseline data)'
    )
    declare_exNum=DeclareLaunchArgument(
        'exNum',
        default_value='1',
        choices= [str(i) for i in range(1, 52)],
        description='value indicating the experiment number'
    )

    moveit_config = MoveItConfigsBuilder("alt_human_arm_model", package_name="moveit_config") \
        .robot_description(file_path="config/alt_human_arm_model.urdf.xacro") \
        .robot_description_semantic(file_path="config/alt_human_arm_model.srdf") \
        .robot_description_kinematics(file_path="config/kinematics.yaml") \
        .joint_limits(file_path="config/joint_limits.yaml") \
        .to_moveit_configs()

    start_parser_node = Node(
        package='ik_processing',
        executable='parser',
        parameters=[
            {'is_baseline': is_baseline},
            {'is_agent1': is_agent1},
            {'startpos1': startpos1},
            {'goalpos1': goalpos1},
            {'startpos2': startpos2},
            {'goalpos2': goalpos2},
            {'datafile': datafile},
            {'exNum': exNum}
        ]  
    )

    start_sim = Node(
        package='ik_processing',
        executable='fullsim',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
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
        declare_baseline,
        declare_agent,
        declare_startpos1,
        declare_goalpos1,
        declare_startpos2,
        declare_goalpos2,
        declare_datafile,
        declare_exNum,
        start_parser_node,
        delay,
    ])