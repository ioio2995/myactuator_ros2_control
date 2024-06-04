from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():

    # Path to the package's configuration directory
    pkg_myactuator_control = get_package_share_directory("myactuator_ros2_control")
    params_file = os.path.join(pkg_myactuator_control, "config", "controllers.yaml")


    control_spawner= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_myactuator_control, 'launch', 'control_spawner.launch.py'))
    )


    # Node to start the controller manager with the specified parameters
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[params_file],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ]
    )

    return LaunchDescription([
        control_node,
        control_spawner,
    ])