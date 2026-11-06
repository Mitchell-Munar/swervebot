# Holonomic state estimation

The estimation graph is:

`wheel velocity -> covariance conditioning -> EKF -> /odom`

`BMI323 raw accel/gyro + BMM350 magnetic field -> Madgwick -> covariance conditioning -> EKF`

The EKF preserves wheel vx, vy and yaw rate. It fuses Madgwick yaw and gyro yaw
rate, excludes wheel absolute pose and linear acceleration, and uses planar mode
only to constrain z, roll and pitch. No standalone laser odometry is launched,
required, or fused. slam_toolbox consumes the scan and odometry transform downstream
and owns map -> odom; its pose is not fed back into this local EKF.

## Launch

Dependencies: robot_localization and imu_filter_madgwick. Madgwick 2.2.2 is
installed in this environment. There is no physical BMI323/BMM350 driver here;
a hardware driver must publish SI units in a consistent board frame with valid
acquisition timestamps. The simulation supplies those two sensor streams.

```bash
colcon build --packages-select swervebot_localization swervebot_description swervebot_sim swervebot_bringup
source install/setup.bash
ros2 launch swervebot_bringup simulation.launch.py enable_sensor_fusion:=true
```

For mapping, use `ros2 launch swervebot_bringup autonomy.launch.py mode:=slam
enable_sensor_fusion:=true` as one command. It starts the same estimation stack.
Do not launch a second simulation or estimator alongside it.

For existing publishers and static robot TF:

```bash
ros2 launch swervebot_localization localization.launch.py use_sim_time:=false
```

Before standalone launch, disable the controller's odometry TF and remap its
`odom` output to `/wheel/odometry`. The integrated simulation does this automatically.
The controller must not publish /odom while the EKF owns that topic.
Fusion remains opt-in until full Gazebo and physical hardware validation is complete.

The standalone orientation filter can also run independently:

```bash
ros2 launch swervebot_localization imu_filter.launch.py use_sim_time:=false
```

It accepts `imu_topic`, `mag_topic`, `output_topic`, and `filter_config` arguments.
Combined launch uses /imu/data_filtered internally and publishes conditioned
/imu/data. Standalone Madgwick publishes /imu/data directly and retains driver gyro
covariance. No filter broadcasts an orientation TF.

## Interfaces and tuning

Inputs are /wheel/odometry (Odometry), /imu/data_raw (Imu), and /mag
(MagneticField). The bridge maps Gazebo /imu/mag to ROS /mag. Both board messages
must use imu_link; the board mounting transform comes from the robot description.
The raw IMU orientation is unused; Madgwick estimates it from accel/gyro/magnetometer.
Madgwick uses magnetic ENU, timestamp-derived dt, gain 0.1, zeta 0, and a 0.05 rad
orientation standard deviation. These require hardware calibration and tuning.

Output /odom and odom -> base_footprint belong exclusively to EKF. Static robot
TF provides base_footprint -> base_link -> imu_link. All simulated nodes use
simulation time. The synthetic magnetic north is world +Y, consistent with ENU
and yaw=0 along world +X. Physical declination, mounting and hard/soft iron errors
must be calibrated before trusting magnetic heading.

`config/conditioning.yaml` provides variance floors, preserving larger driver
uncertainty: wheel vx/vy 0.04 (m/s)^2, wheel yaw rate 0.01 (rad/s)^2 and filtered
gyro 0.0001 (rad/s)^2. Symmetric x/y floors avoid privileging forward motion.
`config/madgwick.yaml` sets orientation uncertainty. `config/ekf.yaml` selects the
measurements and frames; its `ekf_config` launch override accepts a replacement.
The conditioner does not estimate sensor noise or detect wheel slip.

Tune with recorded stationary and moving data, comparing wheel and gyro yaw
rates, magnetic heading and independent ground truth in simulation. Heading and
gyro share correlated errors; do not mistake their fusion for two independent
heading references. Do not add a zero lateral-velocity measurement or lower vy
uncertainty to enforce straight travel. An IMU and wheel encoders cannot reliably
observe steady translational slip against a wall. This architecture cannot promise
its rejection; slam_toolbox handles downstream map alignment, not wheel-slip gating.

## Validation

```bash
colcon test --packages-select swervebot_localization --event-handlers console_direct+
colcon test-result --verbose
```

The launch test runs actual Madgwick, conditioning and EKF processes with synthetic
board and wheel streams. It checks stationary magnetic heading, signed lateral
motion, rotation, IMU dropout/recovery, covariance, quaternion normalization and
single EKF TF authority. It uses wall time and an isolated ROS domain should be used
when other ROS nodes are active. The synthetic runtime test and two focused tests passed in this workspace.
It does not validate the physical board, Gazebo
noise fidelity, wall collisions or map quality. Test forward, reverse, crab,
rotation and wall contact against Gazebo ground truth before production use.
