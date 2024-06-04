import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_myactuator_description = get_package_share_directory('myactuator_description')

    # Start GUI
    rviz_config_path = os.path.join(pkg_myactuator_description, "rviz/myactuator.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    return LaunchDescription(
        [
            rviz_node,
        ]
    )