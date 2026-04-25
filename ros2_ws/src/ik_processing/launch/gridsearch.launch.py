from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    this_package = FindPackageShare('ik_processing')
    launch_dir = PathJoinSubstitution([this_package, 'launch'])

    launch_parser = IncludeLaunchDescription(
        PathJoinSubstitution([launch_dir, 'parser.launch.py'])
    )

    launch_moveit = IncludeLaunchDescription(
        PathJoinSubstitution([launch_dir, 'moveit.launch.py'])
    )

    return LaunchDescription([
        launch_parser,
        launch_moveit,
    ])