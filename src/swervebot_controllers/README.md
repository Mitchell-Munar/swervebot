# Swerve controller connection check

This controller accepts one `geometry_msgs/msg/Twist` on relative topic `cmd_vel`.
With the supplied launch file, this is `/cmd_vel`. It claims four wheel velocity
command interfaces (rad/s), four steering position command interfaces (radians),
and the corresponding velocity/position feedback interfaces. Module order is
front-left, front-right, back-left, back-right in both joint-name arrays.

## Run with mock hardware

From the workspace root, after sourcing ROS 2 Lyrical:

```bash
colcon build --packages-up-to swervebot_controllers
source install/setup.bash
ros2 launch swervebot_controllers controller_mock_test.launch.py
```

In another sourced terminal:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.075}}'
```

Both controllers should stay active. The swerve controller should claim eight
command interfaces. `/joint_states` should report all four wheel velocities near
1 rad/s and all four steering positions near zero. Stop the publisher: within
`cmd_vel_timeout` (default 0.5 seconds), wheel velocities should become zero while
steering holds its measured position. Odometry and optional odom TF are published
by an executor timer at 50 Hz; they are not published from the update thread.

Commands received while inactive are ignored. Every activation requires a fresh
command. Zero, expired, or nonfinite commands stop the wheels. Missing/nonfinite
feedback and failed writes return an update error after attempting to stop every
wheel. A failed hardware write cannot guarantee that the affected actuator stops;
the hardware driver must provide its own watchdog.

Generated parameters are read-only after declaration. Edit the parameter YAML and
reload the controller to apply changes. Only wheel velocity and steering position
command interfaces are supported. The state-interface lists may include position
and velocity, but the controller only claims the feedback it actually reads.

## Tests

```bash
colcon test --packages-select swervebot_description swervebot_controllers
colcon test-result --verbose
```

The C++ tests cover interface order, invalid configurations, lifecycle transitions,
command expiry, fault handling, and allocations in the warmed-up update thread.
The Python launch test verifies the actual `/cmd_vel` → controller → GenericSystem
→ `/joint_states` path, including claimed interfaces, odometry, stops, and restart.
ROS/DDS networking must be available to run these tests. Use the system Python
interpreter that has the ROS test dependencies. If CMake selects an unrelated local
Python, rebuild with `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`.

This verifies the connection using mock hardware. Kinematics accuracy, additional
drive modes, physical hardware communication, Gazebo, and bringup remain deferred.
Steering command bounds are ±90°; the existing continuous URDF joint model still
needs finite-travel modeling when the physical servo specifications are available.
