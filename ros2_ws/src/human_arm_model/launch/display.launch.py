from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import FileContent, LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    # ''use_sim_time'' is used to have ros2 use /clock topic for the time source
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # urdf = FileContent(PathJoinSubstitution([FindPackageShare('human_arm_model'), 'urdf', 'model.urdf.xacro']))
    urdf = Command(['xacro ', PathJoinSubstitution([FindPackageShare('human_arm_model'), 'urdf', 'model.urdf.xacro'])])
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('human_arm_model'),
            'rviz',
            'view_model.rviz'
        ]),
        description='Path to RViz config file'
    )

    return LaunchDescription([
        declare_rviz_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': urdf}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')]
        ),

    ])


    # rviz_node = Node(
    #     package='rviz2'
    #     executable='rviz2'
    #     name='rviz2'
    #     output='screen'
    #     arguments=['-d', LaunchConfiguration('rvizconfig')]
    # )

    # joint_state_publisher_node =Node(
    #     package='joifnt_state_publisher'
    #     executable='joint_state_publisher'
    #     name='joint_state_publisher'
    #     paramters
    # )



    # return LaunchDescription([
    #     IncludeLaunchDescription(
    #         PathJoinSubstitution([FindPackageShare('human_arm_model'), 'launch', 'display.launch.py']),
    #         launch_arguments={
    #             'urdf_package': 'model',
    #             'urdf_package_path': PathJoinSubstitution(['urdf', 'model.urdf'])
    #         }.items()
    #     )
    # ])