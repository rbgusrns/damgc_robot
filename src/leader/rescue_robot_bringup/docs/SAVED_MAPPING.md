# Saved nvblox map and physical restart anchor

Saved on 2026-10-04 at 16:27:45 KST while stationary. Directory on the host:

`/home/maze/damgc_robot/data/maps/home_anchor_20261004_162000`

`data/maps/latest` points to this directory. Contents:

- `map.nvblx`: 16,379,904-byte nvblox layer archive, save service returned success.
- `manifest.json`: odom-frame physical restart pose, zero-velocity 15-element EKF initial state, calibration metadata.
- `projection.pgm` / `projection.yaml`: 2D display projection export; 3D layers are the authoritative saved mapping data.

The stored base_link anchor is x=-0.616429 m, y=-0.067749 m, yaw=10.359079 degrees in the existing odom frame. We retain the existing map coordinates; the physical anchor need not be coordinate (0,0). Mark the robot's current physical position and heading on the floor. Each resume requires placing the robot at that position and heading before launching.

## Resume

From an interactive terminal:

```bash
cd /home/maze/damgc_robot
./scripts/run_saved_mapping.sh
```

The wrapper selects the latest saved map, disables automatic spin, and runs the calibrated 3D mapping/Nav2 stack with the 2D RViz profile. The local EKF initializes at the manifest pose; nvblox then loads the saved layers before teleop and recording start. The initial command selector is STOP. Normal keyboard controls and RViz Nav2 Goal remain available.

A different saved directory can be selected with `MAPPING_SNAPSHOT=/home/maze/damgc_robot/data/maps/<directory>`. Plain `run_vslam_mapping.sh` with no MAPPING_SNAPSHOT starts a new map. `MAPPING_SNAPSHOT` is supported only with the full 3D nvblox stack and must be a directory under the shared repository.

This is manual pose initialization at a known physical anchor, not visual landmark detection or automatic relocalization. VSLAM tracking starts a new visual session and its global frame correction is not used by the odom-frame nvblox/Nav2 map. Placement errors will shift subsequent mapping; odometry can still drift during driving. No physical resume/reboot trial has been performed yet. The current live map was saved without resetting or reloading it.

## Save another anchor/map

Stop the robot and leave it at the desired future start pose. Run in the current Docker ROS environment:

```bash
docker exec -u admin isaac_ros_dev-aarch64-container bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install_docker/setup.bash
  /usr/bin/python3 /workspaces/isaac_ros-dev/scripts/mapping_snapshot.py save /workspaces/isaac_ros-dev/data/maps/NEW_UNIQUE_NAME
'
```

The save tool checks stationary local odometry, saves the 3D map through `/nvblox_node/save_map`, and writes the manifest only after a successful nonempty save. It publishes no motion commands. It does not overwrite existing directories. Update `data/maps/latest` after saving a new anchor if desired. Saved data are local and ignored by Git; back up the full directory to preserve the map outside this robot.

The mode-enabled keyboard also supports **L** to reload the active saved map archive in place. It preserves the current estimated pose, pauses depth during loading, and leaves the command selector stopped afterward. Use `run_saved_mapping.sh` for a restart at the marked physical anchor; L is intended to restore map layers within an already aligned running session.
