# Host-Native TurtleBot Localization Design

## Status
- Phase: approved replacement feature design
- Approved by: user in the current conversation
- Supersedes: the rejected Docker-based simulation design and fleet-level localization launcher concept
- Recommended branch when a Git repository exists: `feature/host-localization`
- Next phase: `2_Align`

## Goal
Run Gazebo Harmonic and localization processes natively on an Ubuntu 24.04 host with ROS 2 Jazzy. Support one or more simulated TurtleBot3 Burger robots while keeping every robot's localization process, topics, frames, lifecycle, and failures independent. Keep the localization package driven by a sensor and TF contract rather than by Gazebo internals.

## Target User And Operator
- A robotics developer validating wheel-odometry and IMU fusion, lidar localization, and optional 2D mapping.
- An operator who starts simulation once, launches localization separately for each desired robot, initializes each robot manually in RViz, and teleoperates a selected robot.

## Success Criteria
- One host-native Gazebo process spawns at least two independently namespaced TurtleBot3 Burgers in one world.
- Each robot publishes isolated wheel-odometry, IMU, lidar, and velocity-command topics with prefixed TF frames.
- Two separately launched localization instances run without node, topic, lifecycle, or TF collisions.
- Each EKF fuses only its robot's wheel odometry and IMU.
- Each AMCL uses the common saved map data through its own namespaced map topic and map frame.
- Each robot can be initialized and localized independently in RViz.
- One selected robot can run mapping and explicitly save a replacement map with SLAM Toolbox.
- The shared YAML can describe more than two robots; initial functional and performance verification covers two.
- No Docker runtime or image is required.

## Non-Goals
- Docker support.
- Autonomous navigation or Nav2 motion control.
- A fleet-level localization launcher that starts all robots automatically.
- A shared global TF frame joining all robots.
- Concurrent or collaborative multi-robot mapping.
- Automatic initial-pose estimation.
- Collision avoidance, task allocation, or coordinated motion.
- Guaranteed performance beyond two verified robots.
- Custom reimplementation of official TurtleBot3 models or sensor plugins.

## Deferred Scope And Follow-Up Candidates
| ID | Item | Why Deferred | Follow-Up Trigger |
| --- | --- | --- | --- |
| FUP-001 | Performance beyond two robots | Capacity depends on host hardware, Gazebo physics, lidar rates, ROS traffic, and visualization load. | A required fleet size or performance envelope is established. |
| FUP-002 | Real non-TurtleBot sensor profiles | The physical platform and its actual topic and frame contracts are not yet available. | A real robot interface is supplied for integration. |
| FUP-003 | Automatic initial pose | Manual RViz initialization is sufficient for the initial demonstration. | Unattended or repeatable automatic startup becomes necessary. |
| FUP-004 | Shared global multi-robot frame | Independent robot map frames satisfy the current localization objective. | Robots must be visualized, compared, or coordinated in one coordinate system. |
| FUP-005 | Concurrent or collaborative mapping | One authoritative map is created by one robot at a time. | Multiple robots must contribute to map construction. |

## Constraints And Assumptions
- Ubuntu 24.04 and ROS 2 Jazzy are installed natively on the host.
- Gazebo Harmonic runs natively on the same host.
- TurtleBot3 Burger is the simulated robot and provides encoder-derived wheel odometry, IMU, and 360-degree 2D lidar data.
- Official TurtleBot3 Jazzy descriptions, Gazebo assets, and sensor plugins are reused and configured rather than copied or recreated.
- One Gazebo world contains every configured simulated robot.
- The standard TurtleBot3 World is the default; a small asymmetric indoor world is an alternative.
- Gazebo GUI is enabled by default, with headless operation supported.
- A single host-side YAML manifest describes the fleet and is passed to both packages.
- All robots use the same saved occupancy-map data identified by one fleet-wide `map_id`.
- Each robot has an independent namespaced copy of that map topic and an independent map frame.
- Localization is launched separately for each robot.
- Mapping is performed by one robot while fleet localization is stopped.
- Initial poses are provided manually per robot in RViz.

## Canonical Mental Model
Two ROS 2 packages share a fleet manifest but have separate ownership:

```text
turtlebot_fleet_sim                    fleet_localization invocation
one Gazebo world                       exactly one selected robot
  + robot1 sensors/body TF               + EKF
  + robot2 sensors/body TF               + map server + AMCL
  + robotN sensors/body TF               or EKF + SLAM Toolbox
```

The localization package is invoked once per robot rather than once per fleet:

```text
invocation for robot1 -> robot1 EKF + map server + AMCL
invocation for robot2 -> robot2 EKF + map server + AMCL
```

Every robot's transform tree is independent even though all robots load identical occupancy-map data:

```text
robot1/map -> robot1/odom -> robot1/base_footprint -> robot1/base_link
robot2/map -> robot2/odom -> robot2/base_footprint -> robot2/base_link
```

The local EKF fuses wheel odometry and IMU. Lidar is consumed by AMCL against an existing map or by SLAM Toolbox during an explicit mapping session.

## Superseded Concepts To Remove
- Remove Docker images, Compose files, mounts, host-networking behavior, and container startup from the feature model.
- Remove the idea of one Docker container per robot and the later replacement of one container holding the entire simulation.
- Remove the fleet-level localization launcher; localization is invoked separately for exactly one selected robot.
- Remove the shared global `map` frame; each robot owns a prefixed map frame and namespaced map topic.
- Remove the assumption that the eventual physical platform is TurtleBot3; real-platform profiles are deferred until an actual contract exists.
- Retain EKF as local state estimation, AMCL as default existing-map localization, and SLAM Toolbox as explicit mapping.

## Approaches Considered
### Selected: Two Packages With Per-Robot Localization Launches
- `turtlebot_fleet_sim` owns host-native Gazebo simulation and manual movement support.
- `fleet_localization` owns a single selected robot's estimation, localization, or mapping per invocation.
- Both accept one shared fleet YAML path.
- This preserves clear ownership and independent robot deployment without adding orchestration infrastructure.

### Rejected: Monolithic Simulation And Localization Package
Rejected because it couples estimation and map behavior to Gazebo, weakens the sensor-interface boundary, and makes future non-Gazebo sources harder to integrate.

### Rejected: Three Packages With Fleet Orchestration
Rejected because a common orchestration layer conflicts with the explicit requirement to run each robot's localization separately and adds no present value.

## Package Responsibilities
### `turtlebot_fleet_sim`
- Reuse official TurtleBot3 Burger description and Gazebo Harmonic assets.
- Start one selected world and spawn every robot listed in the fleet manifest.
- Apply unique robot names, namespaces, frame prefixes, and spawn poses.
- Publish simulation clock, encoder-derived wheel odometry, IMU, lidar, body TF, and sensor TF for each robot.
- Accept namespaced velocity commands.
- Provide separately launched keyboard teleoperation for one explicitly selected robot.
- Own no EKF, AMCL, map-server, SLAM Toolbox, or localization transform behavior.

### `fleet_localization`
- Select exactly one robot entry from the shared manifest per invocation.
- Validate readiness of that robot's sensor topics and required body/sensor transforms.
- In localization mode, run one EKF, one namespaced map server, and one AMCL instance.
- In mapping mode, run one EKF and one SLAM Toolbox instance instead of map server and AMCL.
- Publish per-robot health state and support a per-robot RViz view.
- Save maps only through explicit operator action under a new map ID by default.
- Depend only on the declared sensor, command, map, and TF contract rather than on Gazebo APIs.

## Fleet Configuration Model
One YAML manifest is passed to both packages. At design level it contains:
- Fleet-wide map ID.
- Default simulation world and GUI/headless preference.
- A robot list.
- Per robot: stable name, ROS namespace, frame prefix, simulated model, and Gazebo spawn pose.

The simulation package reads all configured robots and simulation fields. Each localization invocation selects one robot by name, uses identity and frame data, and ignores spawn and world fields. Exact schema spelling and validation belong to alignment.

## Topic And Transform Contract
Per robot, the simulation source publishes:

```text
/<robot>/wheel/odometry    nav_msgs/msg/Odometry
/<robot>/imu               sensor_msgs/msg/Imu
/<robot>/scan              sensor_msgs/msg/LaserScan
/<robot>/cmd_vel           geometry_msgs/msg/Twist
```

All frames are robot-prefixed. Ownership is exclusive:
- Simulation-side robot state publishing owns robot body, wheel, and sensor transforms.
- The selected robot's EKF owns `<robot>/odom -> <robot>/base_footprint`.
- The selected robot's AMCL or SLAM Toolbox owns `<robot>/map -> <robot>/odom`.
- No simulator component publishes the odometry transform.
- No two components publish the same transform edge.

Exact topic names, child frames, body-transform details, QoS, and covariance semantics remain for alignment.

## Map Model
- One fleet-wide `map_id` identifies a single set of saved occupancy-map files.
- Each localization invocation starts its own map-server instance from those files.
- Each instance publishes a namespaced map topic whose occupancy-grid frame is that robot's prefixed map frame.
- Identical map geometry does not imply a shared TF coordinate frame.
- Robots are initialized independently and are not overlaid in one RViz fixed frame.
- A mapping result is saved explicitly under a new map ID; existing map IDs are not overwritten by default.

## Operating Modes
### Localization Mode
1. The operator starts one `fleet_localization` invocation for a selected robot.
2. The invocation waits for that robot's inputs and transforms.
3. EKF fuses wheel odometry and IMU and owns the robot's odometry transform.
4. The namespaced map server publishes a copy of the selected common map data in the robot's map frame.
5. AMCL consumes that robot's lidar, map, and odometry TF and owns its map-to-odometry transform.
6. The operator supplies an initial pose in that robot's RViz map frame.

### Mapping Mode
1. Existing fleet localization invocations are stopped operationally.
2. The operator starts one `fleet_localization` invocation for a selected robot in mapping mode.
3. EKF provides local odometry; SLAM Toolbox consumes that robot's lidar and odometry.
4. The robot is moved with separately launched teleoperation.
5. The operator explicitly saves a valid map using a new map ID.
6. The mapping invocation stops before per-robot localization resumes with the new map ID.

## User-Visible Behavior And Error Handling
- Each localization invocation waits up to a configurable timeout, defaulting to 60 seconds, for wheel odometry, IMU, lidar, and required TF.
- Missing readiness inputs fail only the selected robot's invocation and name every missing item.
- Runtime sensor or transform loss degrades only that robot's instance.
- Health distinguishes waiting for inputs, ready, awaiting initial pose, localized, mapping, and degraded/error states.
- Duplicate robot identities, invalid namespaces or frame prefixes, unknown robot selections, invalid modes, missing maps, and duplicate TF ownership are startup errors.
- Teleoperation requires an explicit target and commands only that robot.
- Map shutdown does not implicitly save partial mapping output.
- Existing map IDs are protected from accidental replacement.

## Testing Strategy At Design Level
- Validate fleet manifest identity, namespace, frame-prefix, spawn-pose, world, and map fields without starting Gazebo.
- Verify official TurtleBot3 assets work through the selected wrapper configuration.
- Verify at least two robots share one world while sensor topics and TF remain isolated.
- Verify separate localization invocations can start, stop, and fail independently.
- Verify every EKF consumes only its selected robot's wheel odometry and IMU and is the sole odometry-TF owner.
- Verify every AMCL consumes its selected robot's map copy and lidar and is the sole map-TF owner.
- Verify manual initial-pose behavior independently for two robot map frames.
- Verify mapping excludes AMCL/map-server ownership, saves only explicitly, and protects existing map IDs.
- Verify standard and custom worlds and GUI/headless modes.
- Treat more-than-two robot manifests as structurally supported but outside the initial performance guarantee.

## Rough Vertical-Slice Roadmap
1. Host-native shared-world simulation and shared fleet-manifest contract.
2. Single-robot EKF localization invocation and readiness behavior.
3. Independent AMCL/map instances for multiple separately launched robots.
4. Explicit single-robot mapping and protected map persistence.
5. Alternate world, teleoperation, health evidence, and integration hardening.

These are roadmap-level intents only. Implementation-ready slice scope belongs to `3_Plan`.

## Open Questions For Alignment
- What exact fleet YAML schema and defaults are accepted by both packages?
- Which official TurtleBot3 Jazzy simulation packages and launch behavior are wrapped?
- What are the exact frame names and body/sensor TF ownership details exposed by the stock model?
- How is simulator odometry TF disabled while preserving raw odometry messages?
- What QoS, timestamp, covariance, and `use_sim_time` contracts apply to each sensor and estimator?
- How does one localization invocation select its robot, mode, map ID, and RViz behavior?
- How are namespaced map-server and AMCL lifecycle nodes managed without Nav2 navigation bring-up?
- What constitutes localized versus degraded health at measurable thresholds?
- Where are common map assets stored and how are IDs validated and protected?
- How is fleet-wide mapping downtime enforced or clearly guarded when localization is launched independently?

## Issue-Log Overlap
No targeted issue-log overlap found. `docs/issues/README.md` and `docs/issues/register.md` do not exist in the current workspace.

## Source Notes
- Initial backlog: [0_backlog.md](0_backlog.md); its Docker constraint was explicitly superseded by the user's revised request.
- User-approved replacement architecture and decisions in the current conversation.
- ROBOTIS TurtleBot3 Jazzy, Ubuntu 24.04, and Gazebo Harmonic setup: <https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/>.
- ROBOTIS TurtleBot3 Burger sensor specifications: <https://emanual.robotis.com/docs/en/platform/turtlebot3/features/>.
- ROBOTIS TurtleBot3 Gazebo simulation: <https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/>.
- SLAM Toolbox Jazzy mapping and map-saving behavior: <https://docs.ros.org/en/ros2_packages/jazzy/api/slam_toolbox/>.

## Repository Rules
- Consulted: Existing pipeline artifacts were read; `docs/rules/README.md`, relevant rule files, `docs/about/01_project_goals.md`, and `docs/issues/README.md` were sought but do not exist.
- Constraints applied: Replaced the rejected design instead of maintaining dual architectural paths, preserved stable deferred IDs where still applicable, kept package and slice discussion at feature level, and created no implementation files.
- Rule updates: None
