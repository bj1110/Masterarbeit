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
    is_baseline = LaunchConfiguration('is_baseline')
    is_agent1 = LaunchConfiguration('is_agent1')
    startpos1=LaunchConfiguration('startpos1')
    goalpos1=LaunchConfiguration('goalpos1')
    startpos2=LaunchConfiguration('startpos2')
    goalpos2=LaunchConfiguration('goalpos2')
    datafile=LaunchConfiguration('datafile')
    exNum=LaunchConfiguration('exNum')
    num_requests=LaunchConfiguration('num_requests')


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
        default_value='7',
        choices=['1', '2', '3', '4', '5', '7', '8', '9'],
        description='(short u_)Int indicating the Gaolposition for agent 2'
    )
    declare_datafile=DeclareLaunchArgument(
        'datafile',
        default_value='1',
        choices=[ str(i) for i in range(1,6)], 
        description='(short u_)Int indicating the experimentfile (in baseline data)'
    )
    declare_exNum=DeclareLaunchArgument(
        'exNum',
        default_value='1',
        choices= [str(i) for i in range(1, 52)],
        description='value indicating the experiment number'
    )


    declare_num_requests=DeclareLaunchArgument(
        'num_requests',
        default_value='1',
        description='Number of requests to the parser before it shuts down'
    )

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
            {'exNum': exNum},
            {'num_requests': num_requests}
        ]  
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
        declare_num_requests,
        start_parser_node,
    ])