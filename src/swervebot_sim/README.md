# Swervebot Gazebo simulation

ROS 2 Lyrical uses Gazebo Jetty. This package runs one robot on a local ground plane,
without Fuel downloads. See the launch file for the process/clock ownership and startup sequence.

```bash
colcon build --packages-up-to swervebot_sim --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch swervebot_sim simulation.launch.py
```

Use `gui:=false` for a headless server. `paused:=true` starts paused: press Play before
controller activation can finish (within the 30-second activation timeout).
`use_sim_time:=true` is the default and is forwarded to every ROS node and both controllers.
`false` is rejected: gz_ros2_control forces simulation time internally, so allowing
other nodes to use wall time would create inconsistent clocks.
`controller_config:=/absolute/path/controllers.yaml` selects another controller configuration.
Launch creates a temporary clock-adjusted copy and removes it on shutdown.

In another sourced terminal:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.3}}'
```

`linear.x` is forward/reverse speed in m/s and `angular.z` is body yaw rate in rad/s.
The front and rear modules counter-steer around a common turning center. Positive
wheel rotation drives forward; positive yaw turns left. Wheel speeds compensate
for the lateral wheel offsets and measured steering motion. Odometry solves the
same rolling constraints; `/simulation/ground_truth` provides independent Gazebo
model poses for comparison and never replaces `/odom` or publishes onto `/tf`.

## Commands and configuration

All planar Twist components are supported: linear x/y and angular z. Lateral and
turn-in-place commands work directly. Manual mode restrictions are supplied by
`swervebot_teleop`. The live `unsupported_command_policy` (`stop` or `ignore`)
applies only to vertical/roll/pitch fields. Changes require a fresh Twist;
nonfinite fields always stop. Commands expire after 0.5 monotonic seconds by
default, including during a simulation pause; resume with fresh commands.

Controller geometry uses per-module arrays, with the same four-module geometry
and ±90-degree steering as the robot Xacro. `steering_velocity_limits` bounds
position steps and is narrowed by URDF velocity limits. See the controller README
for the complete schema. Geometry and other controller settings remain read-only
and require reloading the controller.

Dependencies (in package.xml) include ros_gz_sim, ros_gz_bridge, gz_ros2_control,
launch, xacro, PyYAML, the robot description and controller. Install with
`rosdep install --from-paths src --ignore-src -r -y`. The integration entrypoint
`ros2 launch swervebot_bringup simulation.launch.py` forwards these settings.
Joint limits and contact/servo dynamics are simulation design approximations,
not measured physical specifications. Wheel odometry is not ground truth.

## Verification

```bash
colcon test --packages-select swervebot_description swervebot_controllers swervebot_sim
colcon test-result --verbose
```

Tests cover the model/world contract, numerical kinematics, controller safety,
allocation-free updates, mock interfaces, and headless physical motion. ROS DDS
and Gazebo transport sockets must be available. GUI rendering requires a working
display and graphics driver; headless tests do not require either.

## Observed dependency shutdown issue

On this machine the installed `ros_gz_bridge` 3.0.10 delivers clock/pose data during
simulation but exits with SIGSEGV when launch shuts it down. Gazebo may then require
launch's SIGTERM escalation. The motion tests validate runtime behavior; they do
not certify clean dependency teardown. A related upstream ownership/teardown defect
is tracked in [ros_gz issue 951](https://github.com/gazebosim/ros_gz/issues/951).
This package does not patch system ROS/Gazebo libraries.
