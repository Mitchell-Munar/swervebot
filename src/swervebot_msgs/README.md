# Legacy swerve messages

Defines `RobotCommand` (Twist plus ACKERMANN/CRAB_WALK/ZERO_TURN mode). Retained so
downstream users can migrate without immediately losing the message definition.
The controller and teleop no longer publish or subscribe to this type.

New integrations use `geometry_msgs/msg/Twist` on relative `cmd_vel`; manual mode
projection lives in `swervebot_teleop`. No automatic RobotCommand bridge is supplied.

Dependencies are declared in package.xml (rosidl generators/runtime and geometry
messages). Install with rosdep, build with `colcon build --packages-select
swervebot_msgs`, and source install/setup.bash. This interfaces-only package has no
node, parameter YAML or launch demo. Inspect with `ros2 interface show
swervebot_msgs/msg/RobotCommand`.
