import os
import xacro

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_myactuator_description = get_package_share_directory('myactuator_description')
    xacro_file = os.path.join(pkg_myactuator_description, "urdf", "myactuator.urdf.xacro")
    doc = xacro.process_file(xacro_file)
    robot_description = doc.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}])
    
    
    return LaunchDescription(
        [
        robot_state_publisher_node,
        ]
    )