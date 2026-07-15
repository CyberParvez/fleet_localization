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
