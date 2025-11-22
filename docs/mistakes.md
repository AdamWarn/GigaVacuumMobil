# Avoid These Build Pitfalls

This repo has already hit the issues below—documenting them keeps us from stepping on the same rake twice.

## 1. `setup.cfg` dash-separated keys (patterns package)

- **Mistake:** Used legacy keys `script-dir` / `install-scripts` in `src/patterns/setup.cfg`.
- **Symptom:** `setuptools` printed `SetuptoolsDeprecationWarning: Invalid dash-separated options` during `colcon build`.
- **Fix:** Switch to the underscore variants `script_dir` and `install_scripts`.
- **Prevention:** When copying `setup.cfg` templates, double-check that option names match the latest [`setuptools` docs](https://setuptools.pypa.io) before committing.

## 2. Package naming warning

- **Mistake:** Core package name `GigaVacuumMobil` uses uppercase letters.
- **Symptom:** `colcon` emits `WARNING: Package name "GigaVacuumMobil" does not follow the naming conventions` on every build.
- **Fix (pending):** Rename or add a TODO to align with ROS naming rules (lowercase + underscores/dashes only).
- **Prevention:** Pick ROS package names that already comply so CI stays clean.

## 3. Non-executable helper scripts

- **Mistake:** Added new helper nodes (e.g., `scan_timestamp_sync.py`, `odom_to_pose.py`) without setting the executable bit before installing.
- **Symptom:** Launch files fail with `executable 'scan_timestamp_sync.py' not found on the libexec directory ...` even though the script exists.
- **Fix:** Run `chmod +x src/GigaVacuumMobil/scripts/<script>.py` (or commit with executable permissions) before rebuilding so `colcon` copies it into `install/<pkg>/lib/<pkg>/`.
- **Prevention:** Always `ls -l` new scripts to confirm `rwx` flags, or set the mode in your editor/git attributes immediately after creation.

## 4. Feeding exploration the wrong pose message

- **Mistake:** Pointed `exploration.pose_topic` at `/odom`, which publishes `nav_msgs/Odometry` instead of the expected `PoseWithCovarianceStamped`.
- **Symptom:** Exploration logged `Waiting for pose data on /odom...` despite the robot moving, because the message type never matched.
- **Fix:** Bridge `/odom` into a pose-with-covariance topic (e.g., `odom_to_pose.py` -> `/odom_pose`) and set `exploration.pose_topic` accordingly.
- **Prevention:** Double-check message types before repointing critical topics; if a consumer expects `PoseWithCovarianceStamped`, provide that or add a converter.

## 5. QoS mismatch on synthetic topics

- **Mistake:** Bridged `/odom` to `/odom_pose` using `qos_profile_sensor_data` (best effort) while downstream nodes expected reliable pose data.
- **Symptom:** ROS warnings about QoS incompatibility and no subscribers receiving the pose feed.
- **Fix:** Use a `QoSProfile` with `reliability=RELIABLE`, `history=KEEP_LAST`, `depth=10` so publishers match Nav2/exploration expectations.
- **Prevention:** Before creating synthetic topics, inspect subscriber QoS requirements (`ros2 topic info -v`) and match them explicitly.

_Add more entries whenever we trip over something non-obvious; incremental lessons beat tribal knowledge._
