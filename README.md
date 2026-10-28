# swerve_bot

ROS 2 Lyrical packages for a steerable-wheel mobile robot. The controller supports
three or more modules; the supplied robot has four. Commands use SI units and body
coordinates: **x forward, y left, z up**, with positive yaw counterclockwise.

## Architecture

| Package | Responsibility |
| --- | --- |
| `swervebot_controllers` | Reusable `ros2_control` plugin, ROS-independent kinematics, standalone mock demo |
| `swervebot_teleop` | Joystick profiles, manual mode state, projection into chassis `Twist` |
| `swervebot_description` | This robot's geometry, joint conventions, and hardware selection |
| `swervebot_sim` | Gazebo world, bridge, spawning, and embedded controller manager |
| `swervebot_bringup` | Robot-specific mock and simulation entrypoints |
| `swervebot_msgs` | Legacy `RobotCommand` message retained for downstream migration |

`Joy → teleop → cmd_vel (Twist) → controller → hardware command interfaces`.
The controller also estimates measured chassis motion and publishes odometry/TF.
Manual modes belong to teleop; autonomous callers can publish any planar Twist.
Source arbitration is the integrating project's responsibility.

## Dependencies and build

Use ROS 2 Lyrical and its system Python. Package manifests declare dependencies;
Gazebo integration additionally requires `ros_gz_sim`, `ros_gz_bridge`, and
`gz_ros2_control` compatible with the installed ROS distribution.

```bash
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

For an isolated build without touching existing workspace build artifacts:

```bash
colcon --log-base /tmp/swerve-log build --base-paths src \
  --build-base /tmp/swerve-build --install-base /tmp/swerve-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source /tmp/swerve-install/setup.bash
```

To reuse just the controller, copy `src/swervebot_controllers` into another
workspace and build it against the ROS underlay. Neither the robot description nor
custom messages, teleop, or Gazebo is required. Its own mock demo also works there.

## Run

```bash
# Controller-only GenericSystem demo
ros2 launch swervebot_controllers controller_mock_test.launch.py
# Or the complete robot description with mock hardware
ros2 launch swervebot_bringup mock.launch.py
# Or Gazebo (one embedded controller manager)
ros2 launch swervebot_bringup simulation.launch.py gui:=false
# In another terminal, optionally start manual joystick control
ros2 launch swervebot_teleop teleop.launch.py
```

Run one of the three hardware/manager launches at a time. Verify with
`ros2 control list_controllers` and `ros2 control list_hardware_interfaces`.

```bash
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15}}'
```

See each package README for its configuration and launch interfaces. The default
physical-hardware plugin is a placeholder; this project does not provide a real
device driver.

## Verification

```bash
colcon test --return-code-on-test-failure
colcon test-result --verbose
```

For isolated builds, pass matching `--build-base`, `--install-base`, and
`--log-base` options. ROS tests need working DDS/network sockets and a writable ROS
log directory; set `ROS_LOG_DIR=/tmp/swerve-ros-log` when appropriate. Mock tests
exercise root and namespaced controllers. Gazebo tests compare ground truth with
odometry and require an available physics engine.

## Migration and naming audit

The controller now consumes `geometry_msgs/msg/Twist` on relative `cmd_vel`,
replacing `/swervebot/cmd_raw` and `RobotCommand`. Move mode selection/projection to
your command source; update publishers, remappings, and topic type together.
`unsupported_command_policy` now concerns only nonplanar fields; lateral motion and
yaw-only motion are valid controller commands.

Geometry now uses per-module arrays instead of `wheel_base`, `track_width`,
`wheel_radius`, `wheel_offset`, and `max_steering_velocity`. See the controller
README for the migration table and example.

Naming audit: robot-specific description, simulation, and bringup names describe
their scope. The controller and teleop could eventually use generic package names,
but `swervebot_controllers`, its C++ namespace, and plugin ID are retained to
preserve discovery and downstream imports. `actuator_names` means steering joint
names. No package, namespace, plugin, or existing launch file was renamed.
