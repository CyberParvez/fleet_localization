# Fleet Localization Demo

ROS 2 Jazzy demonstration of one or more TurtleBot3 Burger robots in Gazebo, with separately launched per-robot EKF and AMCL localization. The same localization package also supports exclusive, on-demand SLAM mapping.

## Build

Run from the workspace root:

```bash
cd /home/syncrobot/localization
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The commands below use the example fleet manifest:

```text
/home/syncrobot/localization/src/turtlebot_fleet_sim/config/fleet.example.yaml
```

Source ROS and the workspace in every new terminal:

```bash
cd /home/syncrobot/localization
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Start Gazebo

Start the configured two-robot fleet with the Gazebo GUI:

```bash
ros2 launch turtlebot_fleet_sim fleet_sim.launch.py \
  fleet_config:=/home/syncrobot/localization/src/turtlebot_fleet_sim/config/fleet.example.yaml
```

For headless operation:

```bash
ros2 launch turtlebot_fleet_sim fleet_sim.launch.py \
  fleet_config:=/home/syncrobot/localization/src/turtlebot_fleet_sim/config/fleet.example.yaml \
  gui:=false
```

Wait until the simulator reports:

```text
FLEET_READY
```

## Localize Robot 1

In a new terminal:

```bash
ros2 launch fleet_localization localization.launch.py \
  fleet_config:=/home/syncrobot/localization/src/turtlebot_fleet_sim/config/fleet.example.yaml \
  robot:=robot1
```

RViz opens by default. Use its **2D Pose Estimate** tool to set robot1's approximate initial pose.

To run without RViz:

```bash
ros2 launch fleet_localization localization.launch.py \
  fleet_config:=/home/syncrobot/localization/src/turtlebot_fleet_sim/config/fleet.example.yaml \
  robot:=robot1 \
  rviz:=false
```

## Localize Robot 2

Start a separate localization invocation in another terminal:

```bash
ros2 launch fleet_localization localization.launch.py \
  fleet_config:=/home/syncrobot/localization/src/turtlebot_fleet_sim/config/fleet.example.yaml \
  robot:=robot2
```

Each robot has independent namespaced interfaces, including:

- `/<robot>/map`
- `/<robot>/initialpose`
- `/<robot>/amcl_pose`
- `/<robot>/odometry/filtered`
- `/<robot>/localization/health`

## Move a Robot

Start selected-robot keyboard teleoperation in another terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p stamped:=true \
  -p frame_id:=robot1/base_footprint \
  -p use_sim_time:=true \
  --remap cmd_vel:=/robot1/cmd_vel
```

## Automated Headless Localization Demo

The checked-in harness starts Gazebo and independently localizes both robots:

```bash
./scripts/run_headless_localization_demo.sh \
  /home/syncrobot/localization/src/turtlebot_fleet_sim/config/fleet.example.yaml \
  robot1 robot2
```

The harness also checks robot isolation, shutdown, and restart behavior. It uses process-group cleanup and retains diagnostic logs under `/tmp` if a check fails.

## Mapping Mode

Mapping holds an exclusive lock on the configured map store. Stop all localization invocations before starting it.

Start fresh asynchronous SLAM for robot1:

```bash
ros2 launch fleet_localization localization.launch.py \
  fleet_config:=/home/syncrobot/localization/src/turtlebot_fleet_sim/config/fleet.example.yaml \
  robot:=robot1 \
  mode:=mapping
```

Move robot1 using the teleop command above. Then save a new immutable map ID:

```bash
ros2 run fleet_localization map_save \
  --fleet-config /home/syncrobot/localization/src/turtlebot_fleet_sim/config/fleet.example.yaml \
  --robot robot1 \
  --map-id my_new_map
```

Maps are saved under the manifest's configured `map_store`. Existing map IDs are never overwritten.

To exercise mapping, immutable save, duplicate rejection, and saved-map localization using a disposable map store:

```bash
./scripts/run_headless_mapping_smoke.sh \
  /home/syncrobot/localization/src/turtlebot_fleet_sim/config/fleet.example.yaml \
  robot1 \
  smoke_map
```

## Run Tests

```bash
colcon test --packages-select turtlebot_fleet_sim fleet_localization \
  --event-handlers console_direct+
colcon test-result --verbose
```

