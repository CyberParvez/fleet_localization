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

Health is published on `/<robot>/localization/health`. It reports `awaiting initial
pose`, `localizing`, `localized`, or `degraded`; an active process alone is not
treated as successful localization. Mapping and autonomous navigation are not part
of this slice.
