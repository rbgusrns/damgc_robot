# Mapping recovery — 2026-10-27

## Subsequent physical test and fixes

Session `log/vslam_mapping_20261027_014507`: the initial full scan completed,
then eight exploration waypoints were reached. Navigation used path-length
steps up to 0.60m, a 12cm position checker and separate in-place Spin actions
for final heading. Global paths replan at 0.5Hz. Manual RViz goals retain the
original general checker. BackUp is available for recovery.

The last return initially stopped because an intermediate waypoint crossed
the exploration boundary. The waypoint boundary rejection was removed for
both exploration and return: the square restricts observation targets, not
Nav2's collision-checked transit paths. Connectivity search can also use known free cells outside the square to reach an interior observation target. Return translation also aims along
the approach direction, with the original start yaw applied by a separate
Spin. Running the return-only recovery confirmed return success and a final
162-degree in-place heading correction. The complete mission was not rerun
from a fresh start after this last fix.

Post-return fused-odometry residuals: position 0.135m, heading 0.95 degrees.
These are pose-estimate residuals, not external physical measurements.
The 3m × 3m area contained 3,600 projected cells: 2,670 known, 576 occupied,
and 930 unknown (74.2% observed). Exhausting eligible reachable viewpoints
does not prove full-area observation; coverage improvement remains pending.
Detailed local evidence: `return_only.log` and `result.json` in that session.

Two additional runtime fixes were physically verified: `setsid --wait`
keeps the foreground supervisor attached to its child, and a latest-only
best-effort dynamic TF subscription avoids the explorer's multi-second
queue backlog (probe: final age 0.092s, maximum 0.676s).

The sections below record the recovery baseline and the validation before
the physical test; they are historical, not the current robot state.

GitHub `origin/main` was fetched before comparison. Its latest commit is
`c411ca2` (autonomous exploration and persistent survivor markers). A separate
commit for the 3m × 3m setup does not exist. The mapping launch path was
restored from the preceding `f7255ad` tree, following the mapping/odometry
stabilization in `f342249`.

The three launch files `nvblox_realsense.launch.py`,
`nvblox_vslam_realsense.launch.py`, and `visual_slam_realsense.launch.py` are
byte-for-byte identical to `f7255ad`. The source and installed copies match.
This restores stereo RealSense/VSLAM, dual EKF and nvblox with 2D RViz.

Nav2 geometry/cost settings were also restored: inflation radius 0.30m,
padding 0.01m, cost scaling factor 10, and planner cost penalty 2. Requested
delay allowances remain: TF tolerance 1 second, action acknowledgement
5 seconds and progress checker allowance 20 seconds. The bounded exploration
retains the 3m forward × 3m right rectangle, rear-left start, green boundary,
obstacle recovery and return to the saved start. I²C settings were untouched.

RGB-D odometry and custom DDS/shared-memory profiles were removed from the
active path. Their experiment files are archived in the backup. Camera FPS
returns to 30 by default. YOLO is not started by the mapping wrapper; rosbag
is off by default there, with the reduced topic list available on request.

## Runtime faults fixed

An observed process snapshot contained two local EKFs (PIDs 16798 and 39556)
and leftover rosbag recorders for sessions `003126` and `004020`. Multiple
publishers and old sessions can mix TF/odometry; this is stronger evidence
than attributing the problem to the square boundary itself. Actual map
quality still requires another physical run.

The original `trap cleanup EXIT INT TERM` could return from an interrupt
handler and continue launching stages after cleanup. INT/TERM now exit and
EXIT performs cleanup. A file lock rejects a second launcher before cleanup
traps and PID ownership are installed. Cleanup also checks ownership.
Remaining robot nodes cause preflight refusal. Docker ROS launches use their
own process groups, and cleanup targets the group even if its leader has
already exited. A stopped matching mapping container can be restarted
without a Docker name collision; mismatched image/workspace reuse is refused.

## Validation and state

- 48 process, navigation configuration, frontier lifecycle and search tests passed.
- One real ROS TF reception test passed in a container with no network or
  devices: TF kept updating while the explorer executor was not spun.
- `rescue_robot_bringup` rebuilt successfully; shell syntax and diff checks passed.
- The final installed mapping files were compared to the recovered source.
- The robot stack, cameras, detector and RViz remain stopped. No physical
  navigation or map-quality retest was performed after this recovery.

Backup: `/home/maze/backups/damgc_mapping_recovery_20261027_010105/` contains
the original dirty source snapshot, binary patch, Git status, manifest and
archived experiments. Validation logs are `log/mapping_recovery_validation.log`
and `log/mapping_recovery_build.log`.

From the robot's newly placed start pose, the prepared entry point is:

```bash
bash scripts/run_map_only_exploration.sh
```

## Final pre-push validation

74 focused mapping/runtime/detector tests passed, plus the isolated ROS TF reception test. The bringup package rebuilt successfully. Added regressions cover paths and connectivity outside the exploration rectangle, short path length, reverse-body orientation, separate terminal Spin, intermediate home steps, and rate-limited YOLO bursts. No generated build products or raw trial recordings are included in the commits.
