# Mapping Reliability Plan

_Audience:_ simulation-first operators preparing to migrate improvements to hardware.

_Goal:_ make the exploration + Cartographer stack fully autonomous, reliable, and reasonably fast inside Gazebo before mirroring the exact recipe on the physical robot.

## 0. Pre-flight Checklist (Gazebo)
- Launch `ros2 launch GigaVacuumMobil preflight_gazebo.launch.py use_sim_time:=true` (or run `full_cleaning.launch.py` manually) and confirm `/scan_synced`, `/odom`, `/tf`, `/map`, `/stable_map` all exist (the included `mapping_health_monitor` publishes a latched ready flag).
- Ensure `patterns` auto mode is **off** while tuning; run patterns manually via `/pattern_manager/command`.
- Record rosbag2 captures for every test run: `/scan`, `/scan_synced`, `/odom`, `/tf`, `/map`, `/stable_map`, `/pattern_manager/status`.

## 1. Baseline + Instrumentation
1. Create a `mapping_health` node (or simple launch_ros lifecycle node) that checks:
   - `/scan_synced` rate ≥ 8 Hz in sim.
   - `/odom` age < 0.2 s and TF tree `map→odom→base_link` is valid.
   - Cartographer topic `/trajectory_node_list` still updating.
2. Gate PatternManager auto-start on a latched `/mapping_health/ready` Bool to avoid exploring with missing data.
3. Add a `ros2 launch` arg to `full_cleaning.launch.py` to enable extra diagnostics (topic hz, TF latency logging) when running Gazebo tests.

## 2. Sensor & Preprocessing Hygiene (Sim Focus)
1. Extend `scan_timestamp_sync.py`:
   - Optionally drop scans with skew > `max_skew_sec` instead of re-stamping.
   - Add a moving median intensity/range filter to cut Gazebo glitches.
2. Add a lightweight `LaserScan` downsampler (e.g., take every 2nd beam) and make it param-controlled to match Cartographer load.
3. Inflate odom covariance in `odom_to_pose.py` when commanded angular velocity > threshold to emulate wheel slip alarms; exploration can treat this as “pose uncertain” and pause.
4. Add a near-field collision watchdog subscribing to `/scan_synced`; if any obstacle < `frontier_clearance + 0.1`, preempt the current frontier goal.

## 3. Cartographer Tuning Loop (Rosbag-driven)
1. Set `use_odometry = true`, `max_range = 8.0`, `TRAJECTORY_BUILDER_2D.missing_data_ray_length = 2.5`, `resolution = 0.05` to mirror Nav2 costmap.
2. Increase `num_range_data` from 70 → 120 for crisper walls; raise `POSE_GRAPH.optimize_every_n_nodes` to 120 so optimization cadence matches.
3. Script: replay each rosbag through `slam.launch.py` (with `use_rviz:=false`) and run `cartographer_rosbag_validate` for metrics.
4. Iterate on `ceres_scan_matcher` weights until translational RMS < 3 cm and rotational RMS < 0.5° across your Gazebo apartment world.

## 4. Stable Map Publishing & Saving
1. Break out a dedicated `stable_map.launch.py` (Cartographer optional) that launches `map_updater_node` with:
   - Persistence (`wall_persistence_observations` 12+), dilation (`obstacle_dilation_radius` 0.08), optional workspace bounds.
   - Tunable `wall_gap_fill` and `free_decay_step` exposed as launch args.
2. Add Prometheus-like stats: percentage difference between `/map` and `/stable_map`, save latency, last save timestamp.
3. Ensure every `map_updater` save event writes to `/log/latest/maps/` inside sim for deterministic playback.

## 5. Frontier Planner Overhaul
1. Replace “farthest frontier wins” in `frontier_supervisor.py` with a score = `alpha * (nav_path_length) + beta * (1 / clearance) + gamma * failure_count`.
   - Query Nav2 `ComputePathThroughPoses` (or `is_path_valid`) to confirm reachability before dispatching goals.
2. Sample headings around each candidate cell and reject those that would face unknown space with low clearance.
3. Track per-frontier failure counts; decay `_frontier_clearance` only after repeated failures so exploration keeps pushing but does not oscillate.
4. Add map-inset checks: ensure goal sits inside `/stable_map` eroded by e.g. 10 cm; otherwise drop the candidate.

## 6. Map-Completion Criteria & Automation
1. Extend `MapMetrics` to analyze interior unknown blobs (use morphological erosion to ignore boundary bands) and require both:
   - Edge occupancy ≥ configured fraction.
   - Interior unknown area < configurable square meters.
2. Publish a structured status (reason, percentages) on `/map_ready`; latch with `DurabilityPolicy.TRANSIENT_LOCAL`.
3. When exploration finishes:
   - Trigger `map_updater` immediate save.
   - Switch Nav2 into localization-only mode (`use_slam:=false`) if desired.
   - Enable `coverage_executor` via existing `/coverage_enable` wiring.

## 7. Regression + Performance
1. Add a CI job or `pytest`-style test that replays a canned Gazebo rosbag through the full stack and asserts KPI thresholds:
   - Map seal time < X minutes.
   - Unknown ratio < Y.
   - Frontier failures < Z.
2. Profile CPU usage in simulation; enable LaserScan downsampling or voxel filtering if Cartographer saturates a core.
3. Store KPI logs under `log/build_*` and summarize after each run for quick regress detection.

## 8. Migration Notes (after Gazebo is solid)
- Re-run the exact rosbag-driven validation using hardware bags before touching the robot.
- Keep all tunables in launch files; hardware bring-up should only require parameter flips, not code edits.
- When deploying to the robot, turn off Gazebo-only helpers (e.g., scan dropper if hardware LiDAR already timestamps cleanly) but keep the diagnostics gate in place.

_Follow this playbook during Gazebo bring-up, document results per section, then mirror the configuration on hardware once KPIs stay green._
