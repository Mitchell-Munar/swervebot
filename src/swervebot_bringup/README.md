# Robot bringup

Top-level composition for this robot; reusable controller behavior remains in
`swerve_drive_controller` and Gazebo orchestration in `swervebot_sim`. Dependencies
are declared in `package.xml`; install using `rosdep`, build this workspace with
`colcon build`, and source `install/setup.bash`.

```bash
ros2 launch swervebot_bringup mock.launch.py
ros2 launch swervebot_bringup simulation.launch.py gui:=false
```

Mock uses the complete `swervebot_description` URDF and GenericSystem. Arguments:
`controller_config` (controller package's controllers_mock.yaml), `use_sim_time`
(false; true needs an external clock). Simulation forwards `gui`, `paused`,
`use_sim_time` and `controller_config` to `swervebot_sim`; Gazebo requires sim time.
Choose one manager/hardware entrypoint at a time. Teleop is optional and started
separately so joystick hardware is not required for either demo.

Verify controller activation with `ros2 control list_controllers` and publish
`geometry_msgs/msg/Twist` on `/cmd_vel`. Physical hardware bringup is deferred:
the description's real hardware plugin is a placeholder, not an implemented driver.

`colcon test --packages-select swervebot_bringup` checks the installed robot mock
entrypoint, controller activation, wheel feedback and command expiry.

## Mapping and navigation

Install runtime dependencies with `rosdep install --from-paths src --ignore-src -y`.
Build with `colcon build` and source `install/setup.bash`.

```
ros2 launch swervebot_bringup autonomy.launch.py mode:=slam enable_sensor_fusion:=true use_imu:=true
ros2 launch swervebot_teleop teleop.launch.py cmd_vel_topic:=cmd_vel_teleop
mkdir -p "$HOME/swervebot_maps"
ros2 run nav2_map_server map_saver_cli -f "$HOME/swervebot_maps/imu_on" \
  --ros-args -p use_sim_time:=true
```

Stop mapping and teleop, then restart with the saved map:

```
ros2 launch swervebot_bringup autonomy.launch.py mode:=nav \
  map:="$HOME/swervebot_maps/imu_on.yaml" \
  enable_sensor_fusion:=true use_imu:=true
```

Set **2D Pose Estimate** in RViz, then send a navigation goal. Manual commands on
`/cmd_vel_teleop` override navigation. Stop the teleop publisher to release control
after 0.5 seconds: centered joystick messages still hold manual priority.
Collision monitoring applies to autonomous commands; manual driving requires operator care.

`params_file` selects Nav2/localization configuration; `slam_params_file` selects
SLAM configuration. `gui:=false rviz:=false` supports headless runs.
Simulation requires `use_sim_time:=true`.
The TF chain is `map -> odom -> base_footprint -> base_link -> laser_frame`;
verify with `ros2 run tf2_tools view_frames`.
Navigation servers launch independently of localization so future exploration can
combine `mapping.launch.py` and `navigation.launch.py` without AMCL/map_server.

## Foxglove application bridge

Mock, simulation, and autonomy bringup start one Foxglove bridge by default.
Connect your application to `ws://<robot-host>:8765`. Set
`enable_foxglove_bridge:=false` to disable it, or override `foxglove_address`
(default `0.0.0.0`) and `foxglove_port` (default `8765`). The bridge inherits
`use_sim_time` from the selected bringup entrypoint.

To run the bridge independently:

```bash
ros2 launch swervebot_bringup foxglove_bridge.launch.py
```

Disable the integrated bridge when running a standalone instance on the same port.

## Comparing IMU input

Keep `enable_sensor_fusion:=true` for both trials and change only `use_imu`
(`true` by default). With `use_imu:=false`, the same EKF and wheel covariance
conditioning run without IMU subscriptions. With it enabled, conditioned gyro yaw
rate is fused; magnetic orientation is not selected by the current EKF mask.
SLAM consumes the resulting odometry transform rather than IMU messages directly.
`enable_sensor_fusion:=false` bypasses the EKF and uses controller odometry;
`use_imu` has no effect in that mode. Restart launch between trials.

Save the second trial as `$HOME/swervebot_maps/imu_off`. Repeat the same route,
world, and starting pose for map comparisons. To isolate navigation performance,
use the same saved map, initial pose, and goals for both IMU settings. Compare
completion, travel time, drift against `/simulation/ground_truth`, and map alignment.
Record bags if repeatable offline analysis is needed.

Run map_saver while SLAM is active. Its image and YAML are occupancy maps for
AMCL navigation; resuming SLAM requires a separately serialized pose graph.
