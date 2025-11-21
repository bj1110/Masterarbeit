from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # Start arm controller
    start_arm_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen')
    
    # Launch joint state broadcaster
    start_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen')

    # Add delay to joint state broadcaster (if necessary)
    delayed_start = TimerAction(
        period=10.0,
        actions=[start_joint_state_broadcaster_cmd]
    )

    # Register event handlers for sequencing
    # Launch the joint state broadcaster after spawning the robot
    load_joint_state_broadcaster_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[start_arm_controller_cmd]))


    return LaunchDescription([
        delayed_start,
        load_joint_state_broadcaster_cmd,
    ])