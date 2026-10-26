# Arrival-only person detection

Run from the manually restored starting pose:

```bash
bash scripts/run_survivor_exploration.sh
```

This uses the existing fresh-map, initial scan, 3m forward by 3m right exploration,
short navigation steps and return-to-start mission. At each exploration arrival,
including its heading alignment, the explorer waits for the command selector to
acknowledge STOP, requests `/leader/person_detector/checkpoint` (std_srvs/Trigger),
and waits for three newly captured, distinct RGB frames to finish inference.
Navigation resumes after the reply. Return transit does not run detection.

The detector loads YOLO once, retains it on the GPU, and runs no inference outside
requests. Frames from before a request are discarded. The existing 2Hz cap also
applies inside a request. A failed inference or a 12-second request timeout returns
failure; the explorer stops on failure or after 15 seconds without a response.
An empty successful detection is valid and permits exploration to continue.

The separate checkpoint launch shares the existing RealSense instance, rectifies
RGB, transforms depth-derived person positions into `odom`, and runs the persistent
registry/2D RViz markers. Its confirmation settings allow three observations to
confirm a track (the existing close-person immediate-confirm rule also applies).
Confirmed coordinates survive gaps while driving; matching nearby detections reuse
an existing ID. Valid aligned depth and camera-to-odom TF are necessary for stored
3D coordinates; RGB detection alone cannot supply them. Detection does not trigger
an early return: exploration continues, followed by the normal return home.

`run_map_only_exploration.sh` still defaults to mapping without the detector.
Camera streams used for mapping remain active; this reduces YOLO inference load,
not camera acquisition load. I2C and rosbag settings are unchanged.

Validation on 2026-10-27: real CUDA YOLO in an isolated Docker ROS graph received a
cached sample image with synthetic aligned depth/intrinsics. No inference ran while
idle; two service requests produced exactly 3 + 3 inference calls, with four person
positions per frame. These are sample-image detections, not people detected by the
robot. Navigation gates, worker idle/budget handling, timeout rejection and registry
persistence are covered by focused tests. The first physical trial is recorded below; whole-system CPU savings are unmeasured.


## Physical trial, 2026-10-27 02:29 KST

Logs: `log/vslam_mapping_20261027_022919/`. Fresh map and initial full scan
completed. Seven exploration arrivals each requested three frames (21 total).
The persistent registry recorded person ID 1 from six observations, at odom
(x, y, z) approximately (2.488, -2.059, 0.256) metres. This is a depth/TF estimate.

The main mission began returning, then stopped after filtered depth had not been
received for 10 seconds. Its main coordinator remained in HOLD. A separate
return-only recovery checked all live inputs for freshness and stability,
retained the existing map and original home, and completed the remaining return
and final in-place alignment. `return_recovery.log` contains the COMPLETE event;
the selector confirmed STOP. This was not an uninterrupted automatic round trip.
No timeout thresholds or sensor rates were changed during this trial.

`result.json` contains the final odometry estimate and projected-map observation
fraction. Mission completion indicates eligible reachable goals were exhausted,
not that every cell of the square was observed. People markers and 2D RViz
remain available with the robot stopped.


## Automatic input recovery and medication handoff heading

The next mission run now pauses automatically on a depth-video gap over 10 seconds,
or when active navigation loses a fresh map/robot transform. It selects STOP,
cancels pending/active navigation, planning, spin and backup actions, and waits for
all old action results and selector acknowledgements. Late accepted actions are
cancelled and drained too. It also waits for any in-flight person check to finish.
No action from before the pause can carry over into resumed motion.

The coordinator requires three new samples on each monitored stream, depth image
ages below two seconds, fresh odometry/map/TF, and two continuous seconds of stable
inputs before resuming. It preserves the map, original starting pose and area,
visited/failed positions, completed goal count and persistent people registry.
It replans the interrupted exploration waypoint, or continues returning home if
already returning. It does not perform a new initial scan or reset coordinates.
Recovery waits at most 60 seconds and allows at most three recoveries per mission;
a timeout, action/selector failure or explicit operator stop remains stopped.
An operator STOP while paused permanently cancels that automatic recovery.

At home, the final orientation is now the starting yaw plus pi radians, normalized
to [-pi, pi]. `return_heading_offset_rad` defaults to pi; zero restores the old
orientation. Both direct and short-step final return paths use this heading,
while the green square still uses the original heading. The robot therefore
finishes at the original start looking backward, ready for the medication task.

These changes are exercised by deterministic recovery/action tests and isolated
ROS tests. The 02:29 physical trial above used the previous manual recovery and
original final heading; automatic recovery plus the reverse final heading have
not yet been tested in physical motion. An already running coordinator keeps its
old loaded code until the next mission launch.
