# TurtleBot Fleet Simulation

`turtlebot_fleet_sim` is the project's only supported multi-robot simulation entry point.
It validates the complete fleet manifest before starting one Gazebo Harmonic world and
spawns every configured TurtleBot3 Burger into that shared world.

After installing the ROS 2 Jazzy TurtleBot3 simulation dependencies and building the
workspace:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch turtlebot_fleet_sim fleet_sim.launch.py \
  fleet_config:=$(ros2 pkg prefix --share turtlebot_fleet_sim)/config/fleet.example.yaml \
  gui:=false
```

Simulation exposes only the canonical public source boundary for each robot:
`wheel/odometry`, `imu`, `scan`, and stamped `cmd_vel` beneath its namespace.
The internal `_sim` topics are an implementation detail. Simulation publishes body and
sensor TF only; localization is responsible for `odom -> base_footprint`.

The standard `turtlebot3_world` remains the manifest default. For a complete
custom-world localization or mapping workflow, create a fleet manifest that sets
both `simulation.world: asymmetric_indoor` and `fleet.map_id:
asymmetric_indoor_v1`, then pass that same manifest to every command. Map
provenance is always taken from the manifest and mismatches are rejected before
AMCL activation.

The simulator alone accepts `world:=asymmetric_indoor` as a temporary,
simulator-process-only override. It does **not** alter the manifest read by
localization, mapping, or map saving. Use it only for simulation-only inspection,
not for an end-to-end localization workflow.

Teleoperation is a separate, explicitly selected process:

```bash
ros2 launch turtlebot_fleet_sim teleop.launch.py \
  fleet_config:=/absolute/path/to/fleet.yaml robot:=robot1
```

It publishes only stamped commands to `/robot1/cmd_vel`. `w/a/s/d` issue bounded
motion pulses, key-release timeout or space sends zero, and `q`, EOF, or handled
interrupt sends a final zero. The robot name must exist before a publisher is
created. No plain `Twist` interface is provided. A base/controller command
timeout remains the final protection for uncatchable process or host failure.

For automation use `gui:=false`; it changes only the Gazebo rendering client.
On a display-capable host the default `gui:=true` uses the identical world,
robots, bridges, readiness, clock, and ROS interfaces.
