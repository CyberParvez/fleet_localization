# Slice 01 Plan: Fleet Contract And Shared-World Simulation

## Implementation Status
Status: planned
Commit:
Verification:
Deviations:
Follow-ups:

## Context
- Repository: `/home/syncrobot/localization` (not initialized as a Git repository at planning time)
- Required implementation branch: `feature/host-localization`, created from `dev` after repository initialization
- Feature folder: `docs/pipeline/turtlebot-gazebo-host-localization`
- Selected vertical slice: Fleet Contract And Shared-World Simulation
- Sequence: 01 of 06
- Design source: [1_design.md](1_design.md)
- Alignment source: [2_alignment.md](2_alignment.md)
- Roadmap source: [3_plan.md](3_plan.md)
- Routing source: [CURRENT.md](CURRENT.md)
- UX source: [../../about/05_ux_decision_catalogue.md](../../about/05_ux_decision_catalogue.md)

## Slice Goal
Create a buildable ROS 2 Jazzy workspace and the `turtlebot_fleet_sim` package so one project-owned launch validates a strict shared fleet manifest, starts one Gazebo Harmonic world, and atomically exposes the complete configured TurtleBot3 Burger fleet through the canonical namespaced sensor, command, clock, and prefixed-body-TF boundary. This slice must prove two robots in the standard TurtleBot3 world while retaining schema support for N robots.

This slice establishes the source contract consumed by every later localization slice. It does not implement localization, mapping, RViz localization sessions, or teleoperation.

## Repository Rules
- Consulted: [1_design.md](1_design.md), [2_alignment.md](2_alignment.md), [3_plan.md](3_plan.md), [CURRENT.md](CURRENT.md), and [../../about/05_ux_decision_catalogue.md](../../about/05_ux_decision_catalogue.md). `docs/rules/README.md` and repository rule files do not exist.
- Constraints applied: planning only; preserve two-package ownership; use one project-supported fleet launch; reuse upstream Jazzy assets rather than vendoring them; keep localization per robot and out of this package; retain one canonical stamped command interface; validate the whole manifest before side effects; fail incomplete fleet startup atomically; preserve unrelated user work.
- Rule updates: None

## Follow-Up Scope
- Included deferred items: None.
- Still deferred:
  - `FUP-001`: performance guarantees beyond two robots. The schema and launch remain N-robot capable, but verification covers two.
  - `FUP-002`: real non-TurtleBot sensor profiles.
  - `FUP-003`: automatic initial pose.
  - `FUP-004`: shared global multi-robot frame.
  - `FUP-005`: concurrent or collaborative mapping.

## Central Issue Scope
- Included register IDs: None.
- Deferred register IDs: None.
- Newly created register IDs: None. No central issue register exists, and this slice introduces no durable issue beyond the approved `FUP-###` set.

## Explicit Non-Goals
- Creating `fleet_localization`, EKF, AMCL, map-server, SLAM Toolbox, localization health, lifecycle management, map locking, or map saving.
- Creating a fleet-level localization launcher or a shared global map frame.
- Adding Docker, Compose, ROS bridge gateways, autonomous navigation, collision avoidance, or coordinated control.
- Implementing the custom asymmetric world, matching map assets, keyboard teleoperation, or final GUI/headless hardening; those belong to slice 06.
- Guaranteeing simulator performance beyond the two-robot acceptance case.
- Vendoring or copying TurtleBot3 or `ros_gz` source/assets into this repository.
- Supporting plain `geometry_msgs/msg/Twist` alongside the canonical `TwistStamped` command.
- Publishing `odom -> base_footprint` or any map transform from simulation.

## Canonical Mental Model And Required Cleanup
One validated manifest drives one Gazebo process and the complete simulated fleet:

```text
fleet.yaml -> strict validation -> one Gazebo Harmonic world
                                  + robot1 Burger source boundary
                                  + robot2 Burger source boundary
                                  + robotN Burger source boundary
```

For each robot, simulation owns body, wheel, and sensor transforms and publishes `/clock`, `/<robot>/wheel/odometry`, `/<robot>/imu`, and `/<robot>/scan`; it consumes only `/<robot>/cmd_vel` as `TwistStamped`. Simulation must not own `<robot>/odom -> <robot>/base_footprint`.

Required removals and exclusions:
- Do not introduce Docker artifacts or documentation.
- Do not retain or document the upstream fixed-count multi-robot launcher as a second supported entry point.
- Do not add a legacy `Twist` bridge, alias topics, unprefixed frame fallback, implicit fleet file, hidden world fallback, or partial-fleet mode.
- If the official Burger Gazebo model publishes odometry TF, override only the relevant plugin/model setting while continuing to consume official geometry, meshes, descriptions, and sensor conventions. Do not fork the complete upstream model.

## Reconnaissance And Existing Patterns
- The workspace contains pipeline documents only; there is no `src/`, package manifest, build configuration, test tree, or Git metadata to extend.
- `/opt/ros/jazzy/share/ros_gz_sim` is installed. Its supported launch assets include `gz_sim.launch.py` and ROS/Gazebo spawn launch files, providing the local pattern for starting Gazebo Harmonic and creating entities.
- No TurtleBot3 package was found under `/opt/ros/jazzy/share` during planning. Implementation must install/resolve the official Jazzy binary dependencies with `rosdep` and inspect their installed Burger description, Gazebo model/plugins, bridge configuration, and package exports before writing the minimal override.
- Follow standard `ament_python` patterns for Python launch orchestration, validation/readiness nodes, package resource indexing, and `pytest`/`launch_testing`; use `ament_cmake` only if the inspected upstream integration or a compiled plugin makes it necessary. A new compiled simulator plugin is not expected.
- Follow `ros_gz_sim` launch inclusion and entity-creation interfaces as installed for Jazzy. Do not assume Classic Gazebo launch APIs or filenames from older TurtleBot releases.
- Preserve Gazebo sensor QoS and explicit simulated covariances required by alignment: IMU/lidar best-effort volatile sensor-data QoS, wheel odometry reliable volatile, and nonzero odometry/IMU covariance.

## Likely Files, Modules, Components, And Commands
The exact upstream model reference must follow installed Jazzy package inspection, but the project-owned surface is expected to be:

```text
src/turtlebot_fleet_sim/
  package.xml
  setup.py
  setup.cfg
  resource/turtlebot_fleet_sim
  turtlebot_fleet_sim/__init__.py
  turtlebot_fleet_sim/fleet_config.py
  turtlebot_fleet_sim/fleet_readiness.py
  launch/fleet_sim.launch.py
  config/fleet.example.yaml
  config/bridge.yaml              # only if the installed ros_gz/TurtleBot integration requires a generated/static bridge template
  models/                         # only the minimal Burger plugin/model override required to suppress odometry TF
  test/test_fleet_config.py
  test/test_fleet_launch.py
  test/test_fleet_interfaces.py
```

Component responsibilities:
- `fleet_config.py`: load YAML, resolve `fleet.map_store` relative to the manifest, apply optional defaults, reject unknown/unsafe/duplicate/unsupported values, detect spawn collisions, and return one canonical typed configuration to launch code.
- `fleet_sim.launch.py`: expose required `fleet_config` and optional `world`/`gui` overrides; complete validation before starting Gazebo; launch the selected standard world; create one uniquely named, namespaced, frame-prefixed Burger per robot; arrange required ROS/Gazebo bridges/remaps; and supervise readiness atomically.
- `fleet_readiness.py`: observe all configured Gazebo entities, source topics, `/clock`, and required body/sensor TF; signal success only for the complete fleet; terminate the launch with an itemized failure after the configured startup bound. Runtime sensor loss remains observable and must not terminate Gazebo automatically.
- Minimal model/plugin override: disable simulator odometry TF while retaining raw odometry and the official Burger geometry/sensors. Its exact format and location depend on the installed Jazzy TurtleBot3 model.
- `fleet.example.yaml`: strict two-Burger default manifest using `turtlebot3_world`, absolute `/robot1` and `/robot2` namespaces, slash-free `robot1`/`robot2` TF prefixes, noncolliding spawn poses, required map fields, and GUI enabled.

Expected package dependencies include `ament_index_python`, `launch`, `launch_ros`, `rclpy`, `ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, `tf2_ros`, message packages for clock/odometry/IMU/scan/stamped velocity, YAML/schema parsing support, and the official Jazzy TurtleBot3 description/simulation packages confirmed during implementation. Declare only dependencies actually used after inspection.

## Acceptance Criteria
1. `colcon build` succeeds from a clean ROS 2 Jazzy environment, and `rosdep` can account for every declared dependency.
2. The strict example manifest validates and resolves `map_store` relative to its own location; launch arguments override only `world` and `gui` defaults as aligned.
3. Validation fails before Gazebo starts for unknown keys, missing required fields, unsafe map IDs, unsupported models/worlds, nonabsolute namespaces, leading-slash frame prefixes, duplicate names/namespaces/prefixes, nonfinite poses, and colliding spawn positions. Diagnostics identify the offending field/robot.
4. The project fleet launcher is the only documented/supported multi-robot entry point and starts one Gazebo Harmonic `turtlebot3_world` containing the complete two-Burger example fleet.
5. Startup becomes ready only after both entities, `/clock`, each robot's wheel odometry, IMU, scan, and required body/sensor transforms exist. A failed or incomplete robot startup terminates the whole launch with every missing item listed.
6. Robot topics are isolated exactly as `/<robot>/wheel/odometry`, `/<robot>/imu`, `/<robot>/scan`, and `/<robot>/cmd_vel`; the command topic type is `geometry_msgs/msg/TwistStamped`.
7. Frame IDs are slash-free and robot-prefixed, including `base_footprint`, `base_link`, lidar, IMU, and wheel links. No unprefixed duplicate body/sensor frames appear.
8. Simulation publishes body/sensor TF but never publishes `<robot>/odom -> <robot>/base_footprint`; raw wheel odometry remains available with robot-prefixed frame IDs.
9. Sensor messages use Gazebo simulation timestamps. IMU and wheel odometry expose finite, explicit, nonzero covariance values; the required QoS policies allow subscribers using the aligned contracts to connect.
10. A stamped zero and nonzero command addressed to one robot affects only that robot; there is no plain-`Twist` compatibility topic.
11. The manifest/launcher structurally iterates over N robot entries without hard-coded `robot1`/`robot2` branches, while automated functional verification proves the two-robot case only.
12. No Docker, localization, mapping, custom-world, teleoperation utility, or upstream-vendored source is introduced by this slice.

## Ordered Implementation Tasks
1. Initialize the ROS workspace/package skeleton on the implementation branch without modifying pipeline artifacts or unrelated user files.
2. Resolve official Jazzy binary dependencies with `rosdep`; inspect installed TurtleBot3 Burger URDF/Xacro/SDF, Gazebo Harmonic plugin and bridge configuration, command message type, frame-prefix mechanism, odometry output, and odometry-TF control. Record any upstream contract discrepancy before proceeding.
3. Implement the strict manifest loader and typed resolved configuration, including schema-key rejection, defaults, relative map-store resolution, identifiers, supported values, finite spawn validation, uniqueness, and spawn-collision validation.
4. Add focused unit tests covering the valid two-robot manifest and every required rejection class before simulator orchestration.
5. Implement the project-owned Gazebo fleet launch around installed `ros_gz_sim` and official TurtleBot3 assets, generating per-robot entity names, namespaces, frame prefixes, spawn poses, remaps/bridges, and state publishers from the resolved robot list.
6. Add the smallest project-owned Burger plugin/model override needed to suppress simulator odometry TF while preserving raw odometry and official sensors. If upstream already provides a supported switch, configure that switch and omit a local model override.
7. Implement complete-fleet startup readiness and atomic failure supervision for entities, `/clock`, topics, and body/sensor TF. Keep post-readiness sensor failures diagnostic rather than process-fatal.
8. Add launch/integration tests for topic types and isolation, frame prefixes and TF ownership, clock/timestamps, covariances, QoS compatibility, command isolation, complete readiness, and one deliberately incomplete fleet failure.
9. Build and run the two-Burger standard-world smoke test in GUI-disabled mode for automation; manually confirm GUI startup only if the environment supports it. Do not add the slice-06 custom world or teleoperation utility.
10. Update package-facing usage documentation only if a README is created with the package skeleton; document the project launcher and canonical manifest, not upstream multi-robot alternatives.

## Verification Commands And Expected Outcomes
Run from `/home/syncrobot/localization` after sourcing `/opt/ros/jazzy/setup.bash`:

```bash
rosdep check --from-paths src --ignore-src
```

Expected: all declared dependencies are available. If dependencies are missing, `rosdep install --from-paths src --ignore-src -r -y` is the normal remediation when network/package installation is authorized.

```bash
colcon build --symlink-install --packages-select turtlebot_fleet_sim
```

Expected: the package builds and installs launch, config, resource, optional override, and test assets without warnings caused by this package.

```bash
source install/setup.bash
colcon test --packages-select turtlebot_fleet_sim --event-handlers console_direct+
colcon test-result --verbose
```

Expected: manifest unit tests and non-GUI launch/interface tests pass with zero failures.

```bash
source install/setup.bash
ros2 launch turtlebot_fleet_sim fleet_sim.launch.py fleet_config:=$(ros2 pkg prefix --share turtlebot_fleet_sim)/config/fleet.example.yaml gui:=false
```

Expected: one Gazebo world starts, two entities become ready atomically, and the launch remains active. Run the following inspections in separately sourced shells while it is active:

```bash
ros2 topic list -t
ros2 topic info -v /robot1/cmd_vel
ros2 topic info -v /robot1/wheel/odometry
ros2 topic echo /robot1/imu --once
ros2 topic echo /robot2/scan --once
ros2 run tf2_tools view_frames
```

Expected: canonical per-robot topics and types are present; stamps follow `/clock`; covariances are finite/nonzero where required; body/sensor frames are prefixed; no simulator publisher owns either robot's `odom -> base_footprint` edge.

```bash
ros2 topic pub --once /robot1/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'robot1/base_link'}, twist: {linear: {x: 0.1}, angular: {z: 0.0}}}"
```

Expected: only `robot1` responds. Publish a stamped zero command immediately afterward; `robot2` remains unaffected. Exact motion assertions should live in the integration test rather than depend on visual inspection.

Also run a launch test with an invalid manifest and one with an intentionally unavailable/failed robot model. Expected: validation failure starts no Gazebo process; incomplete-fleet readiness terminates the launch and lists all missing entities/topics/frames.

## Risks And Dependencies
- Official TurtleBot3 Jazzy simulation assets are absent locally at planning time. Their actual package names, model format, bridge conventions, and odometry-TF switch must be verified after dependency installation; older Gazebo Classic examples are not authoritative.
- Upstream may still consume plain `Twist` internally. If so, a project bridge may translate from the one public `TwistStamped` contract internally, but it must not expose a second public command API; stop for review if this requires broad custom control code.
- Some Gazebo plugins cannot parameterize frame prefixes or odometry-TF publication per entity. Prefer supported Xacro/SDF/plugin parameters; a narrow local model override is allowed, while copying the complete upstream model is not.
- Frame prefixing and ROS namespaces solve different problems. Tests must inspect message `frame_id` fields and TF, not infer correctness from topic names.
- Gazebo startup and lidar tests may be resource-sensitive in CI. Use headless/non-GUI execution and bounded readiness, but do not weaken the complete-fleet requirement.
- `map_store` and `map_id` are validated in this slice because the manifest is shared, but no map is read or written until later slices.
- Git branch/commit verification cannot proceed until the workspace is initialized as a repository; implementation must not invent repository history.

## Stop Conditions And Blocked Procedure
Stop this slice and report back before broadening scope when:
- The installed Jazzy TurtleBot3 assets conflict materially with the aligned `TwistStamped`, frame-prefix, raw odometry, covariance, or odometry-TF contracts.
- Disabling simulator odometry TF requires forking upstream TurtleBot3 geometry/sensors or writing a substantial custom drive plugin rather than a minimal override.
- The official assets cannot support arbitrary-N entities in one Gazebo world without upstream modification.
- Required packages cannot be installed, Gazebo cannot run, or the two-robot test cannot execute in the available environment.
- Atomic readiness cannot distinguish entity/topic/TF failure using supported ROS/Gazebo interfaces.
- Work would require changes to `fleet_localization`, map behavior, custom-world/teleoperation scope, pipeline contracts, or unrelated files.
- Existing user changes overlap likely files or the slice becomes materially larger than this contract.

When blocked, preserve all completed nonconflicting work, capture the exact command/error and discovered upstream interface, leave implementation status as `blocked`, and request a design/alignment decision rather than adding compatibility fallbacks.

## Preserve Unrelated Changes
- Inspect status before every edit and before any commit.
- Treat all pre-existing and concurrently created files as user-owned unless this slice explicitly names them.
- Do not delete, reset, reformat, stage, or commit unrelated work.
- Coordinate if another slice plan or implementation appears concurrently in the shared workspace.

## Expected Final Summary
The implementing agent must report:
- The `turtlebot_fleet_sim` package and canonical launch/manifest behavior created.
- The installed upstream TurtleBot3/`ros_gz` interfaces reused and whether a minimal odometry-TF override was required.
- Validation, atomic readiness, topic/type, frame/TF ownership, timestamp/covariance/QoS, and command-isolation evidence.
- Exact build/test/smoke commands run and their outcomes.
- Any deviations, blocked checks, or retained risks.
- The implementation commit hash and paths of unrelated uncommitted changes, if Git exists.
- Confirmation that deferred `FUP-001` through `FUP-005` remain deferred and that slices 02-06 were not implemented.
