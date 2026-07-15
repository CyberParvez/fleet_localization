# Host-Native TurtleBot Localization Alignment

## Status And Routing
- Short feature name: Host Localization
- Pipeline folder: `docs/pipeline/turtlebot-gazebo-host-localization`
- Approved design: [1_design.md](1_design.md)
- Alignment status: approved by the user in the current conversation
- Next phase after UX catalogue checkpoint: `3_Plan`
- Implementation status: no implementation source exists yet

## Shared Understanding
The feature consists of two host-native ROS 2 Jazzy packages. `turtlebot_fleet_sim` starts one Gazebo Harmonic world and atomically spawns the complete configured TurtleBot3 Burger fleet. `fleet_localization` is invoked separately for exactly one selected robot and runs either that robot's EKF plus AMCL/map-server localization stack or its EKF plus SLAM Toolbox mapping stack.

There is no Docker runtime, no fleet-level localization launcher, and no shared global TF frame. All robots use identical saved occupancy-map data by fleet map ID, but each robot publishes that data under its own topic and prefixed map frame.

## Final Decisions
### Runtime And Dependency Baseline
- Host: Ubuntu 24.04 with ROS 2 Jazzy installed natively.
- Simulator: Gazebo Harmonic running natively.
- Use ROS 2 Jazzy binary packages through `rosdep` when available.
- Reuse official Jazzy TurtleBot3, Nav2 AMCL/map-server, `robot_localization`, and SLAM Toolbox dependencies.
- Do not vendor or copy upstream source into the repository.
- Project-owned behavior is limited to configuration validation, launch orchestration, minimal simulator plugin/model overrides, health behavior, map assets, and the custom world.
- The project multi-robot launcher is the only documented multi-robot simulation entry point. The upstream fixed multi-robot launch is not retained as a parallel supported path.

### Package Ownership
#### `turtlebot_fleet_sim`
- Wrap official TurtleBot3 Burger geometry, descriptions, Gazebo assets, and sensor plugins.
- Start one selected Gazebo world.
- Read the complete fleet YAML and spawn every configured robot.
- Publish `/clock`, encoder-derived wheel odometry, IMU, lidar, body TF, and sensor TF.
- Accept namespaced stamped velocity commands.
- Provide a separately launched keyboard teleoperation utility for one explicitly selected robot.
- Use a minimal local model/plugin override when required to suppress simulator odometry TF while preserving raw odometry messages.
- Own no EKF, AMCL, map-server, SLAM Toolbox, map-to-odometry TF, or odometry-to-base TF.

#### `fleet_localization`
- Start exactly one selected robot's stack per invocation.
- Consume the common fleet YAML and ignore simulation-only fields.
- Validate selected-robot inputs and TF before estimator lifecycle activation.
- In localization mode, own one EKF, one namespaced map server, one AMCL instance, one robot-specific RViz session by default, and one health state.
- In mapping mode, own one EKF, one asynchronous fresh-map SLAM Toolbox instance, one robot-specific RViz session by default, and one health state.
- Save maps only through an explicit operator request using a new map ID.
- Depend on the sensor, topic, frame, and map contracts rather than Gazebo APIs.

### Configuration Precedence
The shared YAML contains durable fleet and simulation defaults. Explicit launch arguments override matching YAML defaults.

Simulation launch inputs:
- `fleet_config`: required.
- `world`: optional override.
- `gui`: optional override.

Localization launch inputs:
- `fleet_config`: required.
- `robot`: required.
- `mode`: optional, default `localization`; valid values are `localization` and `mapping`.
- `map_id`: optional override of `fleet.map_id` in localization mode.
- `rviz`: optional, default `true`.
- `timeout`: optional, default 60 seconds.

Mapping mode loads no input map. A new map ID is provided only during the explicit save action.

### Canonical Fleet YAML
The accepted conceptual schema is:

```yaml
fleet:
  map_store: ./maps
  map_id: turtlebot3_world_v1

simulation:
  world: turtlebot3_world
  gui: true

robots:
  - name: robot1
    namespace: /robot1
    frame_prefix: robot1
    model: burger
    spawn:
      x: -1.0
      y: 0.0
      z: 0.01
      yaw: 0.0

  - name: robot2
    namespace: /robot2
    frame_prefix: robot2
    model: burger
    spawn:
      x: 1.0
      y: 0.0
      z: 0.01
      yaw: 3.14159
```

Schema rules:
- Reject unknown keys.
- Require fleet map store, fleet map ID, robot name, namespace, frame prefix, model, and planar spawn coordinates.
- Resolve `map_store` relative to the fleet YAML location.
- Allow defaults only for optional fields such as GUI, spawn `z`, and yaw.
- Reject duplicate names, namespaces, or frame prefixes.
- Require absolute ROS namespaces.
- Reject leading slashes in frame prefixes and all frame IDs.
- Reject unsupported robot models or worlds, unsafe map IDs, invalid numeric poses, and colliding spawn positions.
- The schema supports N robots; acceptance and performance verification cover two.

### Topic Contract
Per robot source interfaces are:

```text
/<robot>/wheel/odometry    nav_msgs/msg/Odometry
/<robot>/imu               sensor_msgs/msg/Imu
/<robot>/scan              sensor_msgs/msg/LaserScan
/<robot>/cmd_vel           geometry_msgs/msg/TwistStamped
```

`TwistStamped` is the canonical Jazzy command type. Plain `Twist` is not a supported parallel interface.

### Frame Contract And Ownership
Topics use absolute ROS namespaces. TF frame IDs use a prefix without a leading slash:

```text
robot1/map
robot1/odom
robot1/base_footprint
robot1/base_link
robot1/base_scan
robot1/imu_link
robot1/wheel_left_link
robot1/wheel_right_link
```

Ownership is exclusive:
- Simulation-side robot state publication owns body, wheel, and sensor transforms.
- EKF exclusively owns `<robot>/odom -> <robot>/base_footprint`.
- AMCL owns `<robot>/map -> <robot>/odom` in localization mode.
- SLAM Toolbox owns `<robot>/map -> <robot>/odom` in mapping mode.
- Simulator odometry TF must be disabled while retaining the raw wheel-odometry message.
- Duplicate transform publishers are configuration/startup failures.

### Time, QoS, And Covariance
- Every simulation, EKF, map-server, AMCL, SLAM Toolbox, RViz, and health node sets `use_sim_time: true`.
- Sensor messages and stamped commands use Gazebo simulation time, not host wall time.
- IMU and lidar use best-effort, volatile sensor-data QoS with small queues.
- Wheel odometry uses reliable, volatile QoS.
- Maps use reliable, transient-local QoS so late-starting localization receives the map.
- Static TF uses transient-local behavior; dynamic TF remains volatile.
- Zero, future, or significantly out-of-order timestamps degrade the selected robot's instance.
- Simulated wheel odometry and IMU provide explicit, nonzero covariance suitable for estimator weighting.

### EKF Policy
- Use planar `two_d_mode`.
- Wheel odometry contributes forward velocity and yaw rate.
- IMU contributes yaw orientation and yaw angular velocity.
- Do not initially fuse IMU linear acceleration.
- Do not simultaneously fuse encoder-derived pose and encoder-derived velocity, avoiding correlated duplicate measurements.
- EKF publishes planar pose, twist, covariance, filtered odometry, and the robot odometry transform.
- All simulated Burgers share one EKF tuning profile.

### AMCL Policy
- All simulated Burgers share one AMCL tuning profile.
- Use a differential-drive motion model and likelihood-field lidar model.
- Inject selected-robot map, odometry, base, and scan frames at launch.
- Each invocation owns an independent map-server and AMCL lifecycle.
- Lifecycle configuration and activation occur automatically after readiness succeeds.
- Manual initial pose is required for every robot.

### Independent Map Model
- One fleet-wide map ID identifies one set of occupancy-map files.
- Every localization invocation loads those same files into its own map-server instance.
- Each map server publishes a namespaced topic with the selected robot's prefixed map frame.
- Identical map geometry does not join robot TF trees.
- There is no shared global map frame and no combined fleet visualization.
- Every supported simulation world has a corresponding map and world-association metadata.
- An obvious world/map metadata mismatch fails before AMCL activation.

Canonical storage is relative to the fleet YAML:

```text
maps/<map_id>/
  map.yaml
  map.pgm
  world metadata
```

- The map store is the only lookup and save location; no hidden fallback exists.
- Map IDs are immutable through the package interface.
- Saving an existing map ID fails; there is no force-overwrite operation.
- Map deletion and archival are manual operations outside the running package.

### Localization Mode
1. The operator starts `fleet_localization` for one robot.
2. The invocation resolves configuration and acquires a shared map-store read lock.
3. It waits for selected-robot sensor topics and required body/sensor TF.
4. It activates EKF, map server, and AMCL automatically.
5. Robot-specific RViz opens by default.
6. The operator sets an initial pose in that robot's map frame.
7. Health progresses from waiting to ready, awaiting initial pose, and localized or degraded.

Multiple separate robot-localization invocations may hold shared read locks concurrently.

### Mapping Mode
1. Existing localization invocations must be stopped.
2. One selected-robot invocation requests the exclusive map-store write lock.
3. Mapping refuses to start while any localization read lock exists.
4. EKF and asynchronous fresh-map SLAM Toolbox activate after readiness.
5. No AMCL or map-server instance runs in mapping mode.
6. Teleoperation moves the selected robot.
7. The operator explicitly saves a new immutable map ID with occupancy files and world metadata.
8. Mapping stops before localization resumes against the new map ID.

No lifelong mapping, pose-graph localization, continuation mode, map merging, or concurrent mapping is included.

### Map-Store Concurrency
- Localization holds an operating-system shared/read lock for the session.
- Mapping requires an exclusive/write lock for the session.
- Operating-system lock release on process termination prevents stale boolean lock flags.
- Localization refuses to start during mapping; mapping refuses to start while any localization session exists.

### Simulation Readiness And Failure
- Validate the entire YAML before Gazebo starts.
- Start the selected world and spawn every configured robot.
- Declare simulation ready only when all configured entities, sensor topics, and required TF exist.
- If any robot fails to spawn or become ready, terminate the complete simulation launch.
- Runtime sensor failure remains observable without automatically terminating Gazebo, supporting diagnosis.

### Per-Robot Localization Readiness And Failure
- Wait up to the configured timeout, default 60 seconds, for wheel odometry, IMU, lidar, and required body/sensor TF.
- Missing inputs fail only the selected robot's invocation and list every missing item.
- Lifecycle transition failure fails only the selected invocation.
- Runtime input or TF loss degrades only the affected robot.

Initial health defaults:
- Required sensor data is fresh when received within 1 second of simulation time.
- The complete robot map-to-sensor TF chain must resolve at the current simulation timestamp.
- EKF output must be current and contain finite state and covariance values.
- AMCL is `awaiting initial pose` until a post-initialization estimate exists.
- `localized` requires a current estimate, planar position covariance no greater than `0.25 m²`, and yaw covariance no greater than `0.10 rad²`.
- A violated health condition persisting for more than 2 seconds yields `degraded`.
- Recovery requires all health conditions to remain satisfied for 2 consecutive seconds.
- Thresholds are configurable through the shared Burger profile.

### RViz Operator Experience
Every localization invocation opens a selected-robot RViz session by default:
- Fixed frame: `<robot>/map`.
- Map: `/<robot>/map`.
- Scan: `/<robot>/scan`.
- Displays: selected robot model, filtered odometry, AMCL pose, and relevant TF only.
- RViz 2D Pose Estimate publishes to `/<robot>/initialpose`.
- One RViz session never shows or initializes another robot.
- Operators may set `rviz:=false` for background robot instances.

### Teleoperation Experience
- Robot selection is required before a velocity publisher starts.
- Publish only to the selected robot's namespaced `TwistStamped` topic.
- Use simulation timestamps.
- Releasing movement keys stops the robot.
- Normal exit or interruption publishes a final zero-velocity command.
- Unknown robot names fail before publication begins.

## Rejected Alternatives
- Docker-based simulation: explicitly removed in favor of a host-native runtime.
- Monolithic simulation and localization package: rejected because it couples estimation to Gazebo.
- Third fleet-orchestration package: rejected because localization is deliberately invoked independently per robot.
- Fleet-level localization launcher: rejected by explicit user requirement.
- Shared global map frame: rejected; robot map frames remain independent.
- Independent map content per robot: rejected; all robots use the same fleet map ID and files.
- Plain `geometry_msgs/msg/Twist`: rejected in favor of Jazzy's stamped command interface.
- Simulator-owned odometry TF: rejected; EKF is the sole odometry-transform owner.
- Identical simulator and future physical tuning: not retained; real profiles wait for an actual interface.
- Automatic map save or force overwrite: rejected to protect map assets.
- Automatic or hidden configuration fallback: rejected in favor of strict validation and explicit inputs.

## Explicit Non-Goals
- Docker files or runtime support.
- Autonomous navigation or Nav2 control behavior beyond AMCL/map-server dependencies.
- A command that starts localization for the whole fleet.
- Global comparison or coordination of robot poses.
- Collaborative, concurrent, continuing, or lifelong mapping.
- Map merging.
- Automatic initial pose.
- Inter-robot collision avoidance or task coordination.
- Physical robot integration without a concrete interface contract.
- Guaranteed performance beyond two robots.
- Custom copies of upstream robot models, plugins, or localization implementations.

## Deferred Scope And Follow-Up Candidates
| ID | Item | Why Deferred | Follow-Up Trigger |
| --- | --- | --- | --- |
| FUP-001 | Performance beyond two robots | Capacity depends on host hardware, Gazebo physics, sensor rates, ROS traffic, and visualization. | A required fleet size or performance envelope is established. |
| FUP-002 | Real non-TurtleBot sensor profiles | The physical platform's topics, frames, QoS, covariance, and timing are unknown. | A real robot interface contract and sample data are supplied. |
| FUP-003 | Automatic initial pose | Manual robot-specific RViz initialization meets the current objective. | Unattended or repeatable startup becomes necessary. |
| FUP-004 | Shared global multi-robot frame | Independent map frames satisfy the current localization requirement. | Robots must be overlaid, compared, or coordinated globally. |
| FUP-005 | Concurrent or collaborative mapping | One authoritative immutable map is created by one robot at a time. | Multiple robots must contribute to a mapping session. |

No central issue-register rows were created because the repository has no `docs/issues/README.md` or register schema. Preserve these feature-local IDs through planning and archive decisions.

## Canonical Mental Model And Required Removals
The canonical unit is one independently launched robot-localization process group. Fleet configuration is shared data, not shared lifecycle orchestration. Simulation publishes raw robot data and body structure; localization owns estimated coordinate relationships.

Planning and implementation must remove or avoid:
- All Docker terminology and artifacts.
- Any one-command fleet localization behavior.
- Unprefixed TF frames.
- A shared `map` frame.
- Simulator publication of the odometry transform.
- Plain `Twist` command aliases.
- Map-path fallbacks and mutable map IDs.
- Parallel supported launch paths that bypass strict fleet validation.

## Likely Vertical Slices And Recommended Order
1. Strict fleet contract and atomic host-native shared-world simulation.
2. One selected robot's readiness, planar EKF, and exclusive odometry TF.
3. One selected robot's immutable map loading, AMCL lifecycle, health, and RViz initialization.
4. Multiple independently launched localization instances using the same map data and isolated map frames.
5. Exclusive single-robot asynchronous mapping and explicit immutable map saving.
6. Alternate world, stamped teleoperation safety, fault behavior, and integration hardening.

These remain roadmap-level boundaries; detailed tasks, file lists, and verification commands belong to `3_Plan`.

## Acceptance Criteria In Plain Language
- A user can provide one valid YAML and start a complete one-or-more-Burger Gazebo fleet on the host.
- Invalid fleet configuration prevents Gazebo from partially starting.
- Each robot exposes the agreed namespaced sensor and stamped-command topics and unique prefixed frames.
- Gazebo does not publish the odometry transform.
- A user can independently launch localization for robot 1 and robot 2 from separate terminals.
- Each launch observes only its selected robot, owns only its selected robot's estimator and localization transforms, and can fail without stopping the other.
- Each robot displays its own copy of the same map in a robot-specific RViz session and accepts manual initial pose without affecting another robot.
- EKF and AMCL health reaches localized only when data, TF, estimates, and covariance meet the agreed thresholds.
- A user cannot begin mapping while any localization invocation holds the map store.
- A mapping session can explicitly create a new map ID but cannot overwrite an existing map ID.
- The standard TurtleBot3 World and custom indoor world each have matching map metadata and reject obvious mismatches.
- Two robots are verified end to end; larger lists parse and spawn structurally without a performance guarantee.

## Risks That Must Shape Planning And Testing
- The official TurtleBot3 Jazzy/Gazebo Harmonic model may couple raw odometry and TF publication, requiring a carefully bounded override.
- Upstream multi-robot examples may carry Gazebo Classic or fixed-count assumptions; only verified Harmonic behavior should be reused.
- ROS namespaces do not namespace frame IDs, so every SDF, URDF, plugin, message header, and transform must apply the prefix consistently.
- Namespaced Nav2 lifecycle nodes and independent prefixed map frames may expose assumptions about the conventional global `map` frame.
- Sensor QoS compatibility must be verified between Gazebo publishers and estimator subscribers.
- `use_sim_time` omissions can cause apparently healthy topics with unusable TF timestamps.
- AMCL covariance thresholds may require tuning against the shipped worlds while retaining their aligned semantics.
- Filesystem locking must cover process crashes and all save paths without introducing stale lock files.
- World-to-map metadata validates identity, not geometric accuracy; scan alignment remains part of verification.
- Multiple GUI and RViz processes may affect two-robot performance on constrained hosts.

## Remaining Open Questions
None material for planning. Exact upstream package versions, launch names, plugin parameters, and tuned numeric estimator values should be discovered and proven during planning and implementation without changing this contract.

## UX Catalogue Updates
Added reusable decisions to `docs/about/05_ux_decision_catalogue.md` for robot-scoped operator sessions, safe manual robot motion, and observable localization health. No conflicts existed because the catalogue was new.

## Source Notes
- Approved design: [1_design.md](1_design.md).
- User answers and approval in the current conversation.
- ROBOTIS TurtleBot3 Jazzy setup and Gazebo Harmonic dependency: <https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/>.
- ROBOTIS TurtleBot3 Burger sensors: <https://emanual.robotis.com/docs/en/platform/turtlebot3/features/>.
- ROBOTIS TurtleBot3 multi-robot topic/frame behavior: <https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/>.
- ROBOTIS Jazzy stamped command behavior: <https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/>.
- SLAM Toolbox Jazzy behavior: <https://docs.ros.org/en/ros2_packages/jazzy/api/slam_toolbox/>.

## Repository Rules
- Consulted: [1_design.md](1_design.md) and existing routing artifacts; `docs/rules/README.md`, relevant rule files, `docs/about/05_ux_decision_catalogue.md`, and `docs/issues/README.md` were sought but do not exist.
- Constraints applied: Maintained one canonical architecture, recorded rejected and deferred work separately, preserved `FUP-###` IDs, kept slices at roadmap level, and avoided implementation files, detailed tasks, file lists, and verification commands.
- Rule updates: None
