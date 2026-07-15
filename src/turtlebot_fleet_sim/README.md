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

