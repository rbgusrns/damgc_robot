# Leader signal starts both grasp tasks

The leader-side `scripts/paired_grasp.py` calls the verified leader and follower
`std_srvs/Trigger` grasp tasks. Both start and cancel services must be discovered
before either robot is started. Each robot uses its own tag alignment and grasp
calibration. This synchronizes the start request, not the moment of jaw closure.
The coordinator waits for each current-cycle STARTED followed by DONE; one side's
DONE or an old DONE is insufficient. Rejected/missing start replies, FAULT or
CANCELLED statuses, or a 135-second timeout request cancellation on both robots.
A late acceptance after cancellation is cancelled again. Cancellation holds the
last gripper target and does not open the jaws. DONE is latched in the coordinator;
restart it to permit another cycle.

Both machines use ROS_DOMAIN_ID=0 and ROS_LOCALHOST_ONLY=0. Follower preparation:

```bash
cd ~/damgc_robot_mission
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ros2 launch follower_supply_perception follower_apriltag_drive.launch.py
```

On the leader, use the validated `leader_grasp_demo.launch.py` with a single owner
for camera, TF, command selector, STM32 bridge and Dynamixel port. The physical
trial shut down the previous mapping stack first, then started the standalone
leader launch. Both robots remain stationary until the start request.

Once both standalone task launches are ready:

```bash
cd ~/damgc_robot
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 python3 scripts/paired_grasp.py --start
```

Omit `--start` to expose `/leader/grasp_pair/start` and
`/leader/grasp_pair/cancel` without starting motion. Progress is published on
`/leader/grasp_pair/status`. This paired trial does not invoke the older
cooperative mission's lift or transport stages.

Physical result on 2026-10-27: both tasks accepted the leader's requests, opened,
aligned and reached DONE. Follower closed first, leader later; total about 48.5s.
Latest sampled camera distances were leader 0.2093m and follower 0.2181m. Both
sampled drive commands were zero. The leader used close raw477, follower raw331,
and lift was disabled. DONE confirms motor ACK plus close wait; no contact sensor
was available to verify physical grip force. Both motor processes remain running
to retain their grip targets. Seven coordinator unit tests passed.
