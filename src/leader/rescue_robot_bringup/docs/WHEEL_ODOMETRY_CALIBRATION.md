# Wheel odometry calibration — 2026-10-04

Local EKF configuration now fuses wheel odometry only. IMU publication and rosbag recording remain enabled for independent comparison. This YAML change takes effect when the local EKF is restarted; the currently running mapping session still uses its startup configuration. Avoid restarting the local EKF alone without preserving its pose, since an odom discontinuity would corrupt the existing map.

The current nvblox map is integrated in `odom`, using local EKF TF. VSLAM/global EKF correction is not directly applied to that map. Wheel-only localization therefore does not eliminate wheel geometry errors or provide global drift correction.

## Prior recorded rotation

Source: `data/vslam_mapping_20261004_142456/vslam_mapping_20261004_142456_0.db3`.
Analysis: `log/vslam_mapping_20261004_151348/previous_spin_pose_diagnosis.json`.

Wheel signed yaw was 361.15 degrees, local EKF 361.15 degrees, and integrated IMU gyro 352.58 degrees over the recorded interval. Local-minus-wheel yaw absolute residual p95 was 0.027 degrees. These measurements establish disagreement, not which sensor is correct. VSLAM signed yaw was 166.84 degrees, with maximum message header gap 834 ms; tracking/reset/frame interpretation needs separate investigation.

## Measurement

Use a visible heading reference to independently measure physical rotation. Start and finish stationary. Do not move the robot by hand during the capture. A command requesting 360 degrees is not an independent measurement of actual rotation.

```bash
cd /home/maze/damgc_robot
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
/usr/bin/python3 scripts/measure_wheel_turn.py --duration 60
```

The tool publishes no commands. Turn manually with the existing teleop during the capture. It saves raw timestamped samples as NPZ and a JSON summary, integrating all three sensors over a common time interval. Supply `--measured-deg 360` only if physical rotation was independently confirmed as positive 360 degrees; use -360 for the opposite direction. `--track-width` must equal the active bridge wheel separation (default 0.23 m).

Suggested width = active width × wheel angle / physical angle. The tool never applies this value automatically. Repeat in both directions before choosing a value. Straight-distance and left/right wheel radius calibration are separate measurements. Do not calibrate track width against IMU or VSLAM without independently validating them.

The bridge currently stamps wheel/IMU messages with host receipt time rather than synchronized MCU capture time. Timestamp alignment remains an additional candidate for map distortion during motion.

## Applied calibration and manual verification

User confirmed one negative full turn ending at the starting heading. Bag recovery measured wheel -383.986 degrees and gyro -373.569 degrees. Effective track width was changed from 0.23 m to 0.2453246 m in the repository bridge defaults, launch defaults, deployed `/home/maze/stm32_bridge_build` equivalents, measurement tool default, and live `/leader/stm32_bridge` parameter. The same parameter affects angular command conversion and wheel yaw integration. Existing integrated map geometry is not corrected retroactively.

A second manually driven full turn after calibration recorded wheel/local -359.728 degrees and gyro -371.717 degrees. Wheel difference from the manually indicated -360-degree turn is +0.272 degrees; this is limited by the user's visual heading reference and is not instrument-verified accuracy. No further width adjustment was applied. Results: `log/vslam_mapping_20261004_154509/calibrated_manual_turn.json` and `.npz`. Remaining IMU discrepancy needs independent calibration/timestamp investigation. The local EKF remains wheel-only. User operates all subsequent physical rotations; do not automatically send spin goals.

The original manual-capture process failed during SIGINT shutdown; its samples were recovered from the companion rosbag into `manual_full_turn_recovered.json/.npz`. Capture now handles termination through a flag while leaving the ROS context alive until samples are saved. Rosbag recording was resumed to `data/vslam_mapping_20261004_154509_continued`.

## IMU yaw-rate scale calibration

The opposite manual turn recorded wheel +364.885 degrees and raw gyro +375.552 degrees. Stationary gyro median was zero in both directions. At user request, use the first-turn gyro magnitude 371.717 degrees as the reference: gyro_z_scale = 360 / 371.717 = 0.9684782203835076, bias = 0. The `imu_gyro_calibration.py` node preserves `/leader/imu/data_raw` and publishes `/leader/imu/data_calibrated`, retaining capture headers and scaling yaw-rate covariance. This corrects z angular velocity only; orientation/quaternion is unchanged and no claim is made of full IMU calibration or MCU/host time synchronization. The local EKF stays wheel-only. The node is launched in both mapping modes and the calibrated topic is recorded in future runs. The current live run has the correction node launched separately.

For manual verification use `scripts/measure_wheel_turn.py --imu-topic /leader/imu/data_calibrated`; wait for the user's start/end signals and never command rotation automatically.

Manual verification after IMU scale correction recorded calibrated gyro -360.543 degrees and wheel/local -358.384 degrees for a user-indicated negative full turn. Difference from the visual -360-degree reference: gyro -0.543 degrees, wheel +1.616 degrees. This is a single trial with a visual reference; it does not establish accuracy across directions, speeds, temperatures, or timestamps. Keep the current scale. Data: `log/vslam_mapping_20261004_154509/imu_calibrated_manual_turn.json/.npz`.

## Current mapping configuration: wheel + calibrated gyro

At user request, new session `20261004_160628` starts a fresh nvblox map with wheel velocity and calibrated IMU z angular velocity fused in the local EKF. Wheel absolute x/y/yaw are no longer fused, avoiding domination of integrated gyro yaw by wheel absolute pose. Wheel lateral zero velocity supplies the differential-drive constraint. No IMU orientation or accelerometer axes are fused. Live subscriptions to `/leader/odom/raw` and `/leader/imu/data_calibrated` were confirmed; active bridge width 0.2453246 m and gyro scale 0.9684782203835076 were confirmed. RViz uses the 2D projection profile. Initial automatic spin is disabled; user controls motion. VSLAM remains separate from the odom-frame mapping pose. Map stability with this fusion still requires observation during user driving.

Previous run and continued rosbag were finalized, including metadata. Previous main bag analysis is saved under `data/vslam_mapping_20261004_154509/analysis.md`.

## Saved development result

User reported substantially improved mapping after enabling calibrated wheel/gyro fusion in session `20261004_160628`. This is qualitative user observation, not a quantified map accuracy evaluation. The five compact measurement/calibration JSON reports are versioned in `docs/calibration_results/20261004/`; original rosbag/NPZ recordings remain on the robot in ignored `data/` and `log/` directories. Current operating mode is 3D nvblox mapping with a 2D RViz projection, manual keyboard mapping and RViz Nav2 goals. Automatic spin remains disabled.
