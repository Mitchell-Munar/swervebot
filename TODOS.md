## Package Refactoring & Standardization Task
* Goal: Update packages to be modular and easy to reuse and configure in other project workspaces
* Audit the kinematic dimensions and ensure the coordinate frames are standard and well-documented.
* Critical goal is to make sure the swerve controller is decoupled, adaptable, and structured to be used in other projects.

### Phase 1: Structure & Configuration Check
- [x] Verify package builds cleanly in an isolated workspace.
- [x] Move all hardcoded constants into declared ROS 2 parameters.
- [x] Create or verify a pre-structured default YAML configuration file in `config/`.
- [x] Add a clean Python test demo launch file in `launch/` that loads the default YAML.

### Phase 2: Documentation & Readability
- [x] Clean up source code formatting and add descriptive inline comments for forkability.
- [x] Write a comprehensive `README.md` covering:
  - Overview & architecture placement
  - Dependencies (`package.xml` / `rosdep`)
  - Configurable parameters and YAML structure
  - Step-by-step build, setup, and launch instructions

### Phase 3: Naming Audit (Approval Required)
- [x] Review current package and namespace names against generic/descriptive conventions.
- [x] **STOP & PROPOSE:** Present proposed renames to the user for explicit approval before executing any file path or `CMakeLists.txt`/`package.xml` changes.

### Implementation decisions and verification
- Retained package names, C++ namespaces, plugin ID, and existing launch filenames;
  the naming audit is complete and no renames are proposed for this pass.
- Controller commands now use standard planar Twist on relative `cmd_vel`; manual
  Ackermann/crab/zero-turn projection belongs to teleop. Legacy messages remain available.
- Approved YAML migration supports N >= 3 modules, arbitrary planar pivot positions,
  individual radii/signed lateral offsets, and bounded or continuous steering.
  Mathematical constants and solver tolerances remain implementation constants.
- Robot hardware Xacro and joystick YAML were preserved, including existing user
  property/lidar work. Controller geometry is checked against expanded robot Xacro.
- All six packages build in isolated temporary build/install directories. The
  controller also builds/tests alone against the ROS Lyrical underlay, and a
  separate CMake consumer compiles and links against its installed public API.
- Workspace results: 101 checks, zero errors/failures, five skips (four cppcheck
  checks skipped by ament for cppcheck 2.19.0 performance issues, plus the existing
  teleop copyright placeholder). Standalone controller: 58 checks, zero errors/
  failures, the same four cppcheck skips. All 35 controller gtests pass.
- Integration checks cover root/namespaced generic mock launches, robot-specific
  mock bringup, teleop mode projection, geometry consistency and Gazebo motion.
- The existing ros_gz_bridge SIGSEGV during shutdown remains documented in the
  simulation README; runtime motion checks pass. Physical drivers remain deferred.
