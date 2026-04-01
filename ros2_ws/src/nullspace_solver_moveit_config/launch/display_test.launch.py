from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PythonExpression, FileContent, LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    this_package= FindPackageShare('nullspace_solver_moveit_config')

    use_sim_time =False 

    modelpath = PathJoinSubstitution([
        this_package, 
        'urdf', 
        'model.urdf.xacro' 
    ])
    urdf = ParameterValue(Command(['xacro ', modelpath]), value_type=str)

    rviz= PathJoinSubstitution([
        this_package,
        'config',
        'model.rviz'
    ])
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': urdf}],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz]
    )

    return LaunchDescription([  
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])