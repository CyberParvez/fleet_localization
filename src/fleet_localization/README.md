# fleet_localization

This package starts one independently selected robot's ROS 2 Jazzy localization
session. Start `turtlebot_fleet_sim` first, then launch one invocation:

```bash
ros2 launch fleet_localization localization.launch.py \
  fleet_config:=/absolute/path/to/fleet.yaml robot:=robot1
```

The launch validates the manifest-relative immutable map, holds a shared map-store
lock, waits for the selected robot's wheel odometry, IMU, lidar, and body TF, and
then starts its EKF, map server, AMCL, lifecycle manager, health publisher, and
robot-scoped RViz. Use RViz's **2D Pose Estimate** once for manual initialization.
Set `rviz:=false` for a background session or pass a safe `map_id:=...` override.

For two robots, start the same launch independently in two terminals. Both
processes read the same immutable map while retaining separate ROS lifecycle and
failure boundaries:

```bash
ros2 launch fleet_localization localization.launch.py \
  fleet_config:=/absolute/path/to/fleet.yaml robot:=robot1 rviz:=false
```

```bash
ros2 launch fleet_localization localization.launch.py \
  fleet_config:=/absolute/path/to/fleet.yaml robot:=robot2 rviz:=false
```

Stopping either command does not stop Gazebo or the other localization command.
Restart that same command to restore only the selected robot. There is
intentionally no fleet-level localization launcher and no shared global map
frame; the sessions publish `/robot1/map` in `robot1/map` and `/robot2/map` in
`robot2/map`, respectively.

Health is published on `/<robot>/localization/health`. It reports `awaiting initial
pose`, `localizing`, `localized`, or `degraded`; an active process alone is not
treated as successful localization.

At startup, readiness observes the shared `/tf` transport for a bounded window
and rejects any preexisting publisher of this robot's owned `odom ->
base_footprint` or `map -> odom` edge. ROS 2 Jazzy's Python `MessageInfo` does not
expose a publisher GID, so a duplicate broadcaster introduced only after
readiness cannot be attributed reliably at runtime. After changing any TF
publisher, restart the affected localization invocation so the startup ownership
check runs again.

Mapping is an explicit, exclusive session. Stop every localization invocation,
then run the same one-robot launcher in fresh asynchronous mapping mode:

```bash
ros2 launch fleet_localization localization.launch.py \
  fleet_config:=/absolute/path/to/fleet.yaml robot:=robot1 mode:=mapping rviz:=false
```

This takes the map store's non-blocking exclusive lock before readiness and then
starts only the EKF, asynchronous SLAM Toolbox, and mapping health graph. Other
localization or mapping launches against that store fail with a lock conflict.

Saving is a separate action and requires a new map ID:

```bash
ros2 run fleet_localization map_save --fleet-config /absolute/path/to/fleet.yaml \
  --robot robot1 --map-id turtlebot3_world_mapped_v2
```

The command verifies the matching mapping session, stages and validates the map,
adds active-world metadata, and atomically publishes the immutable ID. It never
overwrites an existing ID and shutdown never saves automatically. Stop mapping
before launching normal localization with the new `map_id`. Autonomous navigation
is not provided.

## Supported end-to-end workflow

Choose exactly one matched pair in the fleet manifest:
`turtlebot3_world` / `turtlebot3_world_v1`, or
`asymmetric_indoor` / `asymmetric_indoor_v1`. Start the one fleet simulator,
then start this localization launch independently once per robot. Mapping uses
the same launch with `mode:=mapping` only after every localization process has
released its shared map-store lock. Use the separate simulation-package teleop
launch for one selected robot and the explicit `map_save` command for a new ID.
Always pass the same manifest to simulation, localization, teleop, and map-save.
The simulator's optional `world:=...` argument is simulator-only and does not
change localization/map provenance. For a custom-world workflow, change both the
world and map ID in the manifest itself.

Automated verification is headless. GUI Gazebo, robot-scoped RViz manual pose,
visual scan/map alignment, and physical keyboard release are manual host checks;
they do not introduce alternate launch paths. Verification and performance
claims cover two robots. Physical adapters, automatic initial pose, a shared
fleet frame, and concurrent mapping remain outside this demo.

For repeatable public-process headless checks, use
`scripts/run_headless_localization_demo.sh ABSOLUTE_FLEET_YAML robot1 robot2`.
It launches the canonical simulator and one canonical localization process per
robot, then requires health, map, and filtered-odometry output from each. After
stopping localization, `scripts/run_headless_mapping_smoke.sh
ABSOLUTE_FLEET_YAML robot1 NEW_MAP_ID` exercises the canonical mapping,
explicit-save, and load-the-new-map boundaries. The scripts deliberately accept
no world override: the manifest is the single provenance source.
