from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PythonExpression, FileContent, LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # ''use_sim_time'' is used to have ros2 use /clock topic for the time source
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    model = LaunchConfiguration('model')
    use_jsp_gui = LaunchConfiguration('use_jsp_gui')
    use_jsp = LaunchConfiguration('use_jsp')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_model_arg = DeclareLaunchArgument(
        'model',
        default_value='alt_model',
        description='Model to load, chose either "alt_model", "model3" or "model"'
    )

    declare_use_jsp_gui = DeclareLaunchArgument(
        'use_jsp_gui',
        default_value= 'true',
        choices=['true', 'false'],
        description='Flag to enable or disable the usage of JointStatePublisher GUI'
    )

    declare_use_jsp = DeclareLaunchArgument(
        'use_jsp',
        default_value='false',
        choices=['true', 'false'],
        description='Flag to enable or disable the usage of JointStatePublisher'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable or disable the usage of Rviz'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Flag to enable or disable usage of simulation clock (gazebo)'
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

    joint_state_publisher_node= Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition= IfCondition(use_jsp),
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition= IfCondition(use_jsp_gui),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([ 
        declare_model_arg,
        declare_use_jsp,
        declare_use_jsp_gui,
        declare_use_rviz,  
        declare_use_sim_time,  
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
