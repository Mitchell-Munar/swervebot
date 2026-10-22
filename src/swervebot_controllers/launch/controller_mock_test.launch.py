from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Existing robot description
    xacro_file = PathJoinSubstitution([
        FindPackageShare('swervebot_description'),
        'description',
        'swervebot.urdf.xacro',
    ])

    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file,
            ' use_mock_hardware:=true',
            ' use_sim:=false',
        ]),
        value_type=str,
    )

    # Controller configuration for mock testing
    controller_config = PathJoinSubstitution([
        FindPackageShare('swervebot_controllers'),
        'config',
        'controllers_mock.yaml',
    ])

    # Publishes the robot description and TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    # ros2_control controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controller_config,
            {'use_sim_time': False},
        ],
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
        output='screen',
    )

    # Spawners wait for manager services instead of assuming a fixed startup duration.
    spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster', 'swerve_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--param-file', controller_config,
            '--activate-as-group',
        ],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        spawner,
    ])
