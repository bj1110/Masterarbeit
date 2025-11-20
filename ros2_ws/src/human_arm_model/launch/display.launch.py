from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PythonExpression, FileContent, LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # ''use_sim_time'' is used to have ros2 use /clock topic for the time source
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    model = LaunchConfiguration('model')

    declare_model_arg = DeclareLaunchArgument(
        'model',
        default_value='alt_model',
        description='Model to load, chose either "alt_model" or "model"'
    )

    this_package= FindPackageShare('human_arm_model')

    modelpath = PathJoinSubstitution([
        this_package, 
        'urdf', 
        [LaunchConfiguration('model'), '.urdf.xacro'] 
    ])

    urdf = ParameterValue(Command(['xacro ', modelpath]), value_type=str)

    rviz= PathJoinSubstitution([
        this_package,
        'config',
        [LaunchConfiguration('model'), '.rviz']
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
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz]
    )

    return LaunchDescription([ 
        declare_model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
