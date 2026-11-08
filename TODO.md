# Holonomic state estimation and simulated board

## Architecture

- Wheel velocity (vx, vy, yaw rate) and BMI323 IMU data feed the local EKF.
- The BMM350 magnetometer remains enabled in simulation as a passive topic but its values are ignored by the active filtering pipeline.
- Madgwick pre-filter acts strictly as a 6-DOF relative orientation filter, relying on gravity alignment for pitch/roll and bypassing magnetic North alignment for yaw.
- No standalone laser odometry or lidar odometry input is permitted.
- slam_toolbox owns downstream map alignment and map -> odom.
- EKF owns odom -> base_footprint; static TF connects base_link and imu_link.
- Configuration edits for this work were authorized in the implementation session.

## Implemented

- Centered configurable board link, native Gazebo IMU/magnetometer sensors and bridges.
- Madgwick 6-DOF relative filtering (ignoring magnetic vectors) and covariance conditioning outside the controller.
- Holonomic EKF masks with wheel lateral velocity enabled and wheel pose excluded.
- Optional integrated fusion in simulation and autonomy launches.
- Runtime launch validation for Madgwick -> conditioning -> EKF with synthetic inputs.

## Remaining validation

- Verify Gazebo sensor rates, noise profiles, and frame IDs to ensure the 6-DOF IMU plugin matches the expected BMI323 mathematical specs.
- Inject synthetic biases, drift, and noise into the Gazebo IMU and wheel plugins to stress-test the EKF's resilience to imperfect simulation data.
- Verify that the magnetometer topic is streaming but isolated from the Madgwick filter inputs.
- Run full simulated forward/reverse/crab/rotation and wall-contact comparisons.
- Quantify trajectory errors and map wall thickness with repeated recorded routes to verify SLAM scan-matching performance.
- Test simulation pause/resume, time reset and sensor dropout end to end.
- Do not claim robust translational slip rejection: wheel + IMU lacks an independent translation reference during sustained slipping. Covariance floors are not slip detection.
- Production readiness remains unverified until these checks pass.

- need to add launch arguement for `use_sim` so it implements gazebo simulated noise for wheel odom and steering actuators
