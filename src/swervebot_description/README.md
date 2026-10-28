# Robot description

Robot-specific Xacro geometry and ros2_control hardware wiring. This package is
not required to reuse the controller. Dependencies: xacro, launch/launch_ros,
robot-state publisher, joint-state publisher GUI, and the controller package for
the default simulation configuration path. Install dependencies with `rosdep`,
build with `colcon build --packages-up-to swervebot_description`, and source setup.

```bash
ros2 launch swervebot_description swervebot_description.launch.py
ros2 launch swervebot_bringup mock.launch.py
```

The description-only launch visualizes joints; it does not load a controller.
Its arguments are `gui` (true) and `use_sim_time` (false).
Xacro arguments select `use_mock_hardware`, `use_sim`, and `controller_config`.
Default false/false refers to an unimplemented physical hardware plugin. Use bringup
mock or simulation for functioning hardware. Robot dimensions and limits live in
`description/swervebot_properties.xacro`; use the standard Xacro namespace
`http://www.ros.org/wiki/xacro`.

The robot has wheelbase 0.26 m, pivot track width 0.24 m, wheel radius 0.075 m,
and outward lateral wheel offsets ±0.045 m. Pivot coordinates are ±0.13/±0.12 m
relative to base_footprint in x/y. Steering is ±pi/2 with a 10 rad/s slew limit.
Axes are x forward, y left, z up. Wheel joint rotation maps positive rate onto
forward rolling; steering rotates about +z. `base_footprint` is at ground height,
`base_link` and chassis are above it; their planar origins coincide. Odometry's
base frame defaults to base_footprint.

Keep controller module arrays consistent with expanded Xacro joint positions,
wheel radii, signed offsets, and steering limits. The simulation package's geometry
test checks that contract. Existing hardware configuration edits require approval
per AGENTS.md. Sensor additions are independent of the controller refactor.
