# Reusable swerve controller

`swervebot_controllers/SwerveController` implements ROS 2 Lyrical's
`controller_interface::ControllerInterface` for N independently steered drive
modules (N ≥ 3). It has no dependency on this robot's description or custom messages.
The public `swerve_kinematics.hpp` contains ROS-independent geometry, steering
selection, inverse kinematics, and measured chassis velocity estimation.

## Interfaces and conventions

- Input: relative `cmd_vel`, `geometry_msgs/msg/Twist`, queue depth 1. Body-frame
  `linear.x`, `linear.y` (m/s), and `angular.z` (rad/s) are supported together.
- Output: relative `odom`, `nav_msgs/msg/Odometry`; optional odom-to-base `/tf`.
- Each module claims drive `velocity` commands and steering `position` commands.
  Required feedback is drive `velocity` and steering `position` plus `velocity`.
- Drive rates are rad/s; steering is radians. Module array order is arbitrary but
  must be identical across all arrays. Hardware loan order is irrelevant.

The base origin is the reference point for the commanded Twist and reported
odometry. Use x forward, y left, z up; positive steering/yaw rotate about +z.
Steering zero rolls along +x, positive wheel rotation drives along the current
rolling direction. Drivers must supply consistent signs and steering calibration.
Continuous steering feedback must be unwrapped; its position commands remain
unwrapped too.

For pivot `(x,y)`, its velocity is `(vx - omega*y, vy + omega*x)`. A wheel center is
at `pivot + offset*(-sin(angle), cos(angle))`; `offset` is signed and can differ
between modules. Its rolling speed is the pivot velocity projected onto measured
wheel direction, minus `offset*(omega + measured_steering_rate)`, divided by radius.
The same offset and measured steering rate enter forward velocity estimation.
Longitudinal caster offsets and nonvertical steering axes are outside this model.

## Configuration

The generated parameter schema is `config/swerve_controller_parameters.yaml`;
`controllers_mock.yaml` and `controllers.yaml` are runnable defaults, not schemas.
All module arrays must match `wheel_names` length. The supplied order is
front-left, front-right, back-left, back-right.

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    swerve_controller:
      type: swervebot_controllers/SwerveController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
swerve_controller:
  ros__parameters:
    wheel_names: [wheel_front_left_joint, wheel_front_right_joint, wheel_back_left_joint, wheel_back_right_joint]
    actuator_names: [actuator_front_left_joint, actuator_front_right_joint, actuator_back_left_joint, actuator_back_right_joint]
    module_x: [0.13, 0.13, -0.13, -0.13]
    module_y: [0.12, -0.12, 0.12, -0.12]
    wheel_radii: [0.075, 0.075, 0.075, 0.075]
    wheel_offsets: [0.045, -0.045, 0.045, -0.045]
    steering_continuous: [false, false, false, false]
    steering_min: [-1.5707963267948966, -1.5707963267948966, -1.5707963267948966, -1.5707963267948966]
    steering_max: [1.5707963267948966, 1.5707963267948966, 1.5707963267948966, 1.5707963267948966]
    steering_velocity_limits: [10.0, 10.0, 10.0, 10.0]
```

| Parameter | Meaning/default |
| --- | --- |
| `wheel_names`, `actuator_names` | Required nonempty, globally unique drive/steering joint names |
| `module_x`, `module_y` | Required pivot coordinates relative to base origin (m) |
| `wheel_radii` | Required positive per-module radii (m) |
| `wheel_offsets` | Required signed local lateral offsets (m) |
| `steering_continuous` | Required per-module bounded/continuous flags |
| `steering_min`, `steering_max` | Required finite bounds (rad); ignored for continuous modules |
| `steering_velocity_limits` | Required positive slew rates (rad/s) |
| `cmd_vel_timeout` | Monotonic command watchdog, 0.5 s; range (0, 3600] |
| `unsupported_command_policy` | `stop` rejects nonplanar components; `ignore` discards them |
| `command_epsilon` | Components smaller than 1e-6 become zero; nonnegative |
| `steering_feedback_tolerance` | Allowed bounded feedback overshoot, 0.001 rad; commands stay within limits |
| `odom_publish_rate` | Executor publication rate, 50 Hz; range (0, 1000] |
| `odom_frame_id`, `base_frame_id` | `odom`, `base_footprint`; distinct nonempty frame names |
| `enable_odom_tf` | Publish odom-to-base transform, true |
| `wheel_command_interfaces` | Must be `[velocity]` |
| `actuator_command_interfaces` | Must be `[position]` |
| `wheel_state_interfaces` | Default `[velocity]`; may also list position |
| `actuator_state_interfaces` | Must include position and velocity, default both |

Geometry, joint/interface lists and tuning are read-only after declaration.
Only `unsupported_command_policy` is live; an actual policy change invalidates
previous commands and requires a fresh Twist. Configure again after reloading the
controller to apply read-only parameter changes. Available URDF steering limits
narrow configured bounds/rates; incompatible definitions fail configuration.
Coincident pivots, singular geometry, invalid lengths, and nonfinite values fail
configuration rather than entering the update loop.

### Migration from rectangular geometry

| Old value | New arrays |
| --- | --- |
| wheel_base L, track_width W | x `[L/2,L/2,-L/2,-L/2]`; y `[W/2,-W/2,W/2,-W/2]` |
| wheel_radius R | `wheel_radii: [R,R,R,R]` |
| wheel_offset D | `wheel_offsets: [D,-D,D,-D]` |
| max_steering_velocity S | `steering_velocity_limits: [S,S,S,S]` |
| Fixed ±90° steering | Explicit bounded flags and `steering_min`/`steering_max` |

There is no rectangular fallback. Update YAML and command publishers together;
`RobotCommand` and `/swervebot/cmd_raw` are no longer subscribed. Manual mode
projection is implemented by `swervebot_teleop`.

## Build and run independently

Dependencies include `controller_interface`, `hardware_interface`, `rclcpp`,
`realtime_tools`, `generate_parameter_library`, `geometry_msgs`, `nav_msgs`, TF,
and pluginlib. The mock demo additionally uses standard controller manager,
GenericSystem, joint-state broadcaster, robot-state publisher, launch, and PyYAML.
`package.xml` declares the complete dependency set for `rosdep`.

```bash
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to swervebot_controllers --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch swervebot_controllers controller_mock_test.launch.py
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.075}}'
```

Expect eight claimed command interfaces, four wheel rates near 1 rad/s and steering
near zero. Stopping publication stops wheel commands after the timeout. The demo
constructs a minimal kinematic URDF from the chosen YAML, supports different module
counts, and requires no robot-specific package:

```bash
ros2 launch swervebot_controllers controller_mock_test.launch.py \
  namespace:=robot_a controller_config:=/absolute/path/controllers.yaml use_sim_time:=false
ros2 topic pub --rate 20 /robot_a/cmd_vel geometry_msgs/msg/Twist '{linear: {y: 0.1}}'
```

`use_sim_time:=true` requires an external `/clock`. Namespaces separate ROS topics
and manager services; set unique odom/base frame IDs for multiple robots because TF
frame names do not inherit ROS namespaces. Other integrations can remap relative
`cmd_vel`/`odom` normally. Link the exported `swerve_controller` target when using
controller headers, or include the standalone kinematics header without ROS types.

## Lifecycle and real-time behavior

Activation clears old loan references, binds by name, validates measured feedback,
stops wheels and holds steering before accepting fresh commands. Deactivation
stops all wheels and clears references. Commands received while inactive cannot
be replayed after activation. Odometry pose persists across deactivation/reactivation.

Steering selects the closest feasible heading modulo pi; signed wheel speed allows
reverse rolling. Continuous modules choose an unwrapped nearby heading. Bounded
modules stay inside effective travel; if any desired heading is unreachable, all
modules stop and hold until a feasible command arrives. During steering changes,
projecting onto measured headings reduces scrub; motion can differ from the target.

Zero/expired/nonfinite commands stop wheels and hold steering. Any nonfinite field
stops even under `ignore`. Missing, nonfinite, or out-of-range feedback, failed
writes, and solver failures attempt to stop all wheels and return an update error.
A failed write cannot guarantee that the actuator stops: drivers need watchdogs.

All module storage is allocated during configuration. The update path uses bounded
mailbox operations and preallocated buffers; it performs no ROS parameter fetching,
logging or publication. An executor timer publishes fixed-size telemetry and fault
reports. Hardware-interface limiter callbacks must themselves be real-time safe;
a throwing driver/limiter may allocate while creating its own exception.

## Tests

```bash
colcon test --packages-select swervebot_controllers --return-code-on-test-failure
colcon test-result --verbose
```

Gtests cover kinematics, module counts/layouts, travel selection, continuous steering,
lifecycle rebinding, watchdogs, policies, hardware faults and update allocations.
Launch tests exercise the installed plugin through GenericSystem at root and under
`/reuse`, including odometry, commands, stops and restart. Gazebo motion/geometry
checks live in `swervebot_sim`.
