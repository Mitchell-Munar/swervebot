# Holonomic state estimation and simulated board

## Architecture

- Wheel velocity (vx, vy, yaw rate) and BMI323/BMM350 magnetic IMU feed the local EKF.
- No standalone laser odometry or lidar odometry input is permitted.
- slam_toolbox owns downstream map alignment and map -> odom.
- EKF owns odom -> base_footprint; static TF connects base_link and imu_link.
- Configuration edits for this work were authorized in the implementation session.

## Implemented

- Centered configurable board link, native Gazebo IMU/magnetometer sensors and bridges.
- Madgwick magnetic ENU filtering and covariance conditioning outside the controller.
- Holonomic EKF masks with wheel lateral velocity enabled and wheel pose excluded.
- Optional integrated fusion in simulation and autonomy launches.
- Runtime launch validation for Madgwick -> conditioning -> EKF with synthetic inputs.

## Remaining validation

- Calibrate hardware sensor noise, bias, mounting, magnetic declination and hard/soft iron.
- Verify Gazebo sensor rates/noise/covariances and frame IDs against the expanded model.
- Run full simulated forward/reverse/crab/rotation and wall-contact comparisons.
- Quantify trajectory errors and map wall thickness with repeated recorded routes.
- Test simulation pause/resume, time reset and sensor dropout end to end.
- Do not claim robust translational slip rejection: wheel + IMU lacks an independent
  translation reference during sustained slipping. Covariance floors are not slip detection.
- Production readiness remains unverified until these checks pass.
