import os


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_myactuator_control = get_package_share_directory("myactuator_ros2_control")
    pkg_myactuator_description = get_package_share_directory("myactuator_description")

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_myactuator_control, "launch", "robot_state_publisher.launch.py"
            )
        ),
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_myactuator_control, "launch", "control_manager.launch.py")
        )
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_myactuator_description, "launch", "rviz.launch.py")
        )
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            control,
            rviz,
        ]
    )
