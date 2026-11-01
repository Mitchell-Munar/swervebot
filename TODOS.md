## Goal: Implement SLAM and Navigation

## Long term goal is to make a controller interface that can switch between manual and autonomous drive with different autonomous drive behaviors like waypoint follower, coverage path planning, and explorer. 

** The custom controller interface and different autonomous drive behavior is currently out of scope. Use the context only for architectural decisions when implementing SLAM and Nav2 to make it easy to add drive behavior later.

* Robot is a swerve drive (holonomic). Configure Nav2 for omidirectional motion

* Setup and use twist_mux for switching source behavior

* Packages `swervebot_navigation` and `swervebot_mapping` are made for Nav2 and SLAM. Setup those 2 packages.

* `swervebot_mapping`: slam_toolbox (online_async) config + launch, map saving instructions, maps/ directory

* `swervebot_navigation`: Nav2 params (costmaps, planner, controller, AMCL, velocity smoother, collision_monitor), localization launch (map_server + AMCL), navigation launch, RViz config

* twist_mux (in swervebot_navigation, config/twist_mux.yaml):
   - inputs: teleop (priority 100), nav2 (priority 50)
   - output feeds the swerve controller's cmd_vel input
   - Nav2 must NOT publish directly to the controller

* Create a launch file in `swervebot_bringup`: launch file that starts the sim with `test_room_1.sdf` from `swervebot_sim`, robot_state_publisher, controllers, bridge, twist_mux, and RViz, with launch args: `mode:=slam|nav`, `use_sim_time`, `map`, `params_file`

## Constraints
* Use Python launch files, FindPackageShare/PathJoinSubstitution
* No parameters in bringup; params live in the navigation/mapping packages.
* Do not modify description/control/sim packages unless required; if so, list the change and why.
* Single TF publisher per transform (odom->base_link from one source only).
* Update package.xml dependencies and CMake install rules.

### Done when
* `colcon build` succeeds with no errors.
* `ros2 launch swervebot_bringup <file> mode:=slam` starts the sim; driving via teleop builds a map visible in RViz; map saves successfully.
* `mode:=nav` with the saved map: AMCL localizes and a Nav2 goal from RViz moves the robot to the target without collision.
* Publishing to the teleop topic overrides Nav2 via twist_mux.
* tf tree is map->odom->base_link->lidar frame, verified with view_frames.