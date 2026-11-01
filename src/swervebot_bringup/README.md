# Robot bringup

Top-level composition for this robot; reusable controller behavior remains in
`swervebot_controllers` and Gazebo orchestration in `swervebot_sim`. Dependencies
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
ros2 launch swervebot_bringup autonomy.launch.py mode:=slam
ros2 launch swervebot_teleop teleop.launch.py cmd_vel_topic:=cmd_vel_teleop
ros2 run nav2_map_server map_saver_cli -f ~/Code/ROS2/swerve_bot/src/swervebot_navigation/maps/room
```

Stop mapping and teleop, then restart with the saved map:

```
ros2 launch swervebot_bringup autonomy.launch.py mode:=nav \
  map:=$HOME/Code/ROS2/swerve_bot/src/swervebot_navigation/maps/room.yaml
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
