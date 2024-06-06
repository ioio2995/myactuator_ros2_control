from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
 
    # Spawner for the joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawner for the motor controller
    motor_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["motor_controller", "--controller-manager", "/controller_manager"],
    )

    motor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["motor_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Event handler to spawn the wheel and tail controllers after the joint state broadcaster spawner exits
    other_spawner_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[motor_controller_spawner,motor_broadcaster_spawner],
        )
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        other_spawner_event_handler,
    ])