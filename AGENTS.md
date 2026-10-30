# Project Name & Overview
ROS 2 swerve drive mobile robot project (`swerve_bot`) for a 4-module platform (4 motorized drive wheels and 4 steering servos). Driven by a custom `ros2_control` hardware interface and controller.

## Build
* **Build:** `colcon build` (Packages use `ament_cmake` for C++ and `ament_python` for Python)
* **Testing:** Use `gtest` for C++ packages and `pytest` for Python packages.

## Code Style & Architecture
* **Language/Framework:** ROS 2 (Lyrical Luth)
* **Architecture Pattern:** Standard `ros2_control` real-time controller layout inheriting from `controller_interface::ControllerInterface`. Packages are to be designed for modularity and reuse. They should be parameterized and driven by configuration files (YAML) and launch files.
* **Swerve Kinematics:** A single `geometry_msgs::msg::Twist` (`cmd_vel`) subscription handles chassis velocity, which is resolved via inverse kinematics inside the controller loop directly to individual wheel velocities and servo positions. Do not create separate subscriptions for individual servos.
* **Real-Time Safety:** Strictly no memory allocations, `rclcpp` logging, or node parameters fetching inside the `update()` loop.
* **Lifecycle Hooks:** `on_init`, `on_configure`, `on_activate`, and `on_deactivate` must safely manage hardware loans. Clear local reference vectors inside `on_activate` to prevent stale tracking data.
* **Parameters:** Use the `generate_parameter_library` package for managing, declaring, and validating controller parameter YAML files.
* **State Toggling:** Drive state toggles or controller mode shifts are handled by the custom teleop node. Used for manual operations.
* **Imports:** Group standard library first, third-party second, local modules third.
* **Error Handling:** Always catch specific exceptions; never use bare `except:`.
* **Documentation & Review:** Add clear, concise comments to logical blocks of code explaining the functionality, mathematical formulas, and purpose of the lines to make the logic easily reviewable during pull requests. Focus especially on explaining math transitions inside the swerve kinematics and lifecycle management stages.

## Key Directories
* `src/swervebot_controllers/` - Custom C++ `ros2_control` swerve controller classes and real-time logic.
* `src/swervebot_description/` - Robot URDF and Xacro configuration files. Use standard standard Xacro URL namespaces (`http://www.ros.org/wiki/xacro`).
* `src/swervebot_sim/` - Latest version of Gazebo simulation launch and world configurations. Ensure `use_sim_time` configuration is passed correctly through launch arguments.
* `src/swervebot_bringup` - Top-level launch files combining the robot description, controller manager configurations, and simulation nodes.

## Common Workflows / Constraints
* Request permission before modifying any existing parameter files (`.yaml`) or robot hardware configuration formats.
* Real-time safety inside the update() loop (no smart pointers creation, no logging, no dynamic allocations).