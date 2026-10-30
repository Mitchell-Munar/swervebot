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
per AGENTS.md.

## Adapting the four-module model

Edit `description/swervebot_properties.xacro` for geometry, masses, colors, limits,
module names, module placement, and sensor settings. The entrypoint loads this file
once; component files do not reload or reset it. The four `module_N_x/y/z` values
locate steering pivots relative to the chassis; `module_N_wheel_x/y/z` locate each
wheel relative to its steering link. Defaults derive from wheelbase, track width,
and signed wheel offsets. All four contact points must remain on the ground plane;
if vertical offsets change, adjust `chassis_ground_clearance` accordingly.

Match controller YAML arrays to altered module positions, names, wheel radii,
lateral offsets, and steering limits before driving. The current controller does
not support longitudinal caster offset: keep module wheel x offsets zero.
Changing the description does not automatically change controller parameters.

Component macros accept explicit shape and mounting inputs for use in other
assemblies. Include properties and inertia helpers before component macros.
The default joint names and eight hardware command interfaces are retained.
Simulation and mock hardware modes are mutually exclusive; the physical hardware
plugin remains a placeholder.

## Lidar

Set `lidar_enabled` to false to omit the sensor model and simulation bridge.
The default mount uses `base_link`, whose origin is the chassis center, so
`lidar_z = chassis_height / 2 + lidar_height / 2` places the sensor bottom on the
chassis top. `lidar_x/y/z` and `lidar_rpy` are relative to `lidar_parent`; adjust
these together when selecting a different parent. The frame is `laser_frame`.

Simulation produces a 500-sample, 10 Hz planar scan from -pi to pi with range
0.05–12 m. These are configurable simulation values, not calibrated hardware
specifications. `lidar_topic` selects both the Gazebo and ROS topic (default
`/scan`). The simulation launch reads the expanded sensor to create a one-way
LaserScan bridge. Mock and description-only launches retain the physical sensor
model without creating simulated scans. No physical lidar driver is started.

The bundled simulation world loads the Sensors system with Ogre2. GUI-disabled
simulation uses headless rendering, which still requires an EGL-capable renderer.
Custom worlds must also load a Sensors system. Gazebo may warn that `gz_frame_id`
is an extension to SDF; it is retained to give ROS scans the correct TF frame.
