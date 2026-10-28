# Joystick teleop

Converts `sensor_msgs/msg/Joy` on relative `joy` into `geometry_msgs/msg/Twist` on
relative `cmd_vel`. Mode state belongs here rather than in the hardware controller:
Ackermann uses longitudinal motion/yaw and suppresses yaw while throttle is zero;
crab walk uses longitudinal/lateral motion; held zero-turn uses yaw alone. Mode
buttons trigger on rising edges. Releasing zero-turn restores the toggled mode.

Dependencies are rclpy, geometry/sensor messages, joy, ament_index_python, launch,
and PyYAML. Install with `rosdep`, build with `colcon build --packages-select
swervebot_teleop`, then source the install setup file.

```bash
ros2 launch swervebot_teleop teleop.launch.py
ros2 launch swervebot_teleop teleop.launch.py namespace:=robot_a \
  config_file:=/absolute/path/profile.yaml cmd_vel_topic:=cmd_vel
```

Launch arguments: `driver_type` (`game_controller` or `joy`), `config_file`,
`namespace`, `cmd_vel_topic`, `device_id` (0), `driver_deadzone` (0.05), and
`autorepeat_rate` (50 Hz). The node itself accepts `config_file` and supports normal
ROS remappings. Empty config_file selects the bundled profile. An invalid supplied
profile fails explicitly.

`config/game_controller.yaml` is a joystick profile, not a ROS parameter file:
`axes` maps throttle/steering indices; `buttons` maps toggle_ackermann,
toggle_crab_walk and hold_zero_turn indices; `settings` supplies deadzone,
linear_scale_max and angular_scale_max. An index of -1 disables an input. Missing
Joy entries produce zero inputs. The existing emergency_stop/invert_steering profile
fields are reserved and are not implemented; no emergency-stop mechanism is claimed.
Crab lateral input retains the existing steering-axis scale. Profile deadzone must
be in [0,1); scales must be finite and nonnegative.

Autonomous/manual arbitration is not provided. The downstream controller's watchdog
stops when command updates cease. Test with `colcon test --packages-select
swervebot_teleop` (pytest includes mode projection and input handling tests).
