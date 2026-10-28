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
