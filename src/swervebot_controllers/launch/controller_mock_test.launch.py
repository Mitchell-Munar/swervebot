from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


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

    # Start the joint state broadcaster after the manager starts
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager',
                    '/controller_manager',
                ],
                output='screen',
            )
        ],
    )

    # Start the custom controller after the broadcaster
    swerve_controller_spawner = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'swerve_controller',
                    '--controller-manager',
                    '/controller_manager',
                    '--param-file',
                    controller_config,
                ],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        swerve_controller_spawner,
    ])