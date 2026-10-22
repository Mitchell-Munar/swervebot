import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the path to the URDF file
    urdf_file = os.path.join(
        get_package_share_directory('swervebot_description'),
        'description',
        'swervebot.urdf.xacro'
    )

    # Process the URDF file using xacro
    urdf_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Create a robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_content}]
    )

    # Create a joint_state_publisher_gui node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Return the launch description with both nodes
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node
    ])
