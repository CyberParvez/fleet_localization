# Slice 02 Plan: Single-Robot EKF And Readiness

## Implementation Status
Status: implemented
Commit: `f741ef3`
Verification: Both ROS packages build; `colcon test` passes 33/33 with zero failures; Python compilation and `git diff --check` pass. An escalated same-shell live Gazebo workflow completed selected-robot readiness, started the robot1 EKF/health processes, emitted finite `/robot1/odometry/filtered` with `robot1/odom` and `robot1/base_footprint`, and resolved the EKF-owned transform with `tf2_echo`. Shutdown regression coverage verifies clean SIGINT/external shutdown handling.
Deviations: The environment's `rosdep` database reports no definition for installed `ament_python`; builds and tests resolve it successfully. Gazebo occasionally logs simulation-time jump warnings under accelerated headless execution, but the live filtered output and transform remain current and valid. No approved behavior deviation.
Follow-ups: None in slice scope. `FUP-001` through `FUP-005` remain deferred.

## Context
- Repository: `/home/syncrobot/localization` (the workspace is not initialized as a Git repository at planning time)
- Branch: unavailable; phase publication ultimately requires `dev`
- Feature folder: `docs/pipeline/turtlebot-gazebo-host-localization`
- Selected vertical slice: **02 — Single-Robot EKF And Readiness**
- Sequence: 02 of 06
- Design source: [1_design.md](1_design.md)
- Alignment source: [2_alignment.md](2_alignment.md)
- Roadmap source: [3_plan.md](3_plan.md)
- Routing source: [CURRENT.md](CURRENT.md)
- UX source: [../../about/05_ux_decision_catalogue.md](../../about/05_ux_decision_catalogue.md)
- Required predecessor: slice 01 must provide the `fleet_localization` package skeleton, strict fleet-manifest parser, selected-robot resolution, simulated sensor contract, prefixed body/sensor TF, and simulator suppression of odometry TF.

## Exact Slice
Deliver one separately invoked, selected-robot local-estimation workflow. The invocation validates its robot from the shared fleet manifest, waits independently for wheel odometry, IMU, lidar, simulation time, and required body/sensor transforms, then starts one planar `robot_localization` EKF. The EKF publishes namespaced filtered odometry and is the only publisher of `<robot>/odom -> <robot>/base_footprint`. A robot-scoped health publisher makes readiness, estimator activity, timestamp/covariance faults, and recovery observable. This slice stops at local odometry; it does not load a map or establish a map transform.

## Repository Rules
- Consulted: [1_design.md](1_design.md), [2_alignment.md](2_alignment.md), [3_plan.md](3_plan.md), [CURRENT.md](CURRENT.md), and [../../about/05_ux_decision_catalogue.md](../../about/05_ux_decision_catalogue.md); `docs/rules/README.md` and rule files do not exist.
- Constraints applied: Preserve exactly one robot per localization invocation, strict configuration with no fallback interface, Jazzy simulation time and QoS policy, exclusive TF ownership, observable robot-scoped health, and external use of the installed `robot_localization` package.
- Rule updates: None

## Follow-Up Scope
- Included deferred items: None.
- Still deferred: `FUP-001` performance guarantees beyond two robots; `FUP-002` real non-TurtleBot sensor profiles; `FUP-003` automatic initial pose; `FUP-004` shared global multi-robot frame; `FUP-005` concurrent or collaborative mapping.
- Do not add generalized physical-platform profiles or fleet orchestration while implementing this slice.

## Central Issue Scope
- Included register IDs: None.
- Still deferred register IDs: None.
- Newly created register IDs: None. The repository has no `docs/issues/README.md` or issue-register schema; do not invent one in this slice.

## Canonical Mental Model
One command selects exactly one manifest robot and creates exactly one local estimator boundary:

```text
/<robot>/wheel/odometry -- forward velocity + yaw rate --+
                                                         +--> EKF --> /<robot>/odometry/filtered
/<robot>/imu ------------ yaw + yaw angular velocity ----+             |
                                                                       +--> <robot>/odom -> <robot>/base_footprint

/<robot>/scan + source body/sensor TF + /clock --> readiness/health only in this slice
```

- The source publishes raw measurements and body/sensor transforms.
- The EKF alone publishes the odometry transform.
- The readiness gate completes before the EKF process starts; it is not merely a warning emitted alongside an already-running filter.
- Readiness failure terminates only this invocation and reports every missing or invalid input.
- Runtime faults degrade this robot's health without inventing a fleet-wide controller.
- `map`, map server, AMCL, SLAM Toolbox, and RViz map-frame behavior do not exist in this slice.

## Required Cleanup And Removals
- Do not retain or introduce an unprefixed `odom`, `base_footprint`, or `base_link` frame.
- Do not support `/<robot>/odom` as an alternate raw source beside the canonical `/<robot>/wheel/odometry`; remapping belongs at the source boundary established by slice 01.
- Remove any slice-01 placeholder odometry-transform publisher before enabling EKF `publish_tf`; there must be one owner, not a compatibility fallback.
- Remove any simulator/plugin odometry TF still observed at runtime rather than disabling EKF TF publication.
- Do not fuse wheel pose, IMU acceleration, or lidar into the EKF. Do not retain example `robot_localization` inputs or its unprefixed example frames.
- Do not copy or vendor upstream `robot_localization` source or its sample configuration.

## Likely Files, Modules, And Components
The workspace currently contains documentation only. Exact paths may follow the package skeleton created by slice 01; if that skeleton materially differs, stop and reconcile this plan rather than creating a second package layout.

- `src/fleet_localization/package.xml` — declare runtime dependencies including `ament_index_python`, `launch`, `launch_ros`, `rclpy`, `robot_localization`, `nav_msgs`, `sensor_msgs`, `diagnostic_msgs`, `tf2_ros`, and test dependencies used below.
- `src/fleet_localization/setup.py` and `setup.cfg` (or slice-01 equivalent) — install Python executables, launch files, profile YAML, and tests according to the established `ament_python` layout.
- `src/fleet_localization/launch/localization.launch.py` — canonical one-robot entry point with required `fleet_config` and `robot`, optional `mode`, `map_id`, `rviz`, and `timeout`; in this slice accept the aligned interface but reject modes other than the implemented local-estimation path with an explicit message. Reuse the manifest parser from slice 01, derive namespace/topic/frame substitutions once, run the readiness process first, and start EKF plus runtime health only after success.
- `src/fleet_localization/config/burger_sim.yaml` — single shared Burger estimator/health profile; set `use_sim_time`, `two_d_mode`, prefixed frame placeholders, `world_frame == odom_frame`, `base_link_frame == <robot>/base_footprint`, `publish_tf: true`, raw input topics, fusion vectors, queue/timeout settings, process covariance, and health thresholds.
- `src/fleet_localization/fleet_localization/readiness.py` — bounded one-shot selected-robot gate using aligned QoS; wait for nonzero `/clock`, wheel odometry, IMU, scan, and the selected robot's body/sensor transforms; validate message frame IDs, finite values, nonzero covariance, and sane timestamps; accumulate all missing/invalid reasons; exit zero only on complete readiness and nonzero otherwise.
- `src/fleet_localization/fleet_localization/health.py` — persistent selected-robot health monitor publishing `diagnostic_msgs/msg/DiagnosticArray` on `/<robot>/localization/health`; report stable states and per-condition key/value evidence for input freshness, TF, EKF output, finite values, and covariance. This slice needs `waiting for inputs`, `ready`, `ekf active`, and `degraded/error`; later slices extend the same interface with `awaiting initial pose`, `localized`, and `mapping`.
- `src/fleet_localization/fleet_localization/topic_qos.py` — shared QoS constructors only if slice 01 has not already established them: sensor-data QoS for IMU/scan, reliable volatile QoS for wheel and filtered odometry, and standard TF QoS through `tf2_ros`.
- `src/fleet_localization/test/test_ekf_profile.py` — static parameter-contract checks, including exact 15-element fusion vectors and absence of forbidden measurements.
- `src/fleet_localization/test/test_readiness.py` — unit tests for requirement aggregation, timeout, timestamp ordering, frame checks, finite-value checks, covariance validation, and successful readiness.
- `src/fleet_localization/test/test_health.py` — deterministic health state/persistence tests using injected time or pure state evaluation; verify 1-second freshness, 2-second degradation, and 2-second recovery windows without wall-clock sleeps.
- `src/fleet_localization/test/test_ekf_launch.py` — `launch_testing` integration with synthetic `/clock`, wheel odometry, IMU, scan, and static body/sensor TF; assert delayed EKF start, topic/frame isolation, filtered output, transform ownership, and selected-invocation shutdown on timeout.
- `src/fleet_localization/test/fixtures/` — minimal valid fleet and synthetic-message fixtures reused from slice 01 rather than duplicated when those fixtures already exist.

Installed Jazzy patterns to follow:
- `/opt/ros/jazzy/share/robot_localization/launch/ekf.launch.py` launches executable `ekf_node` with a YAML parameter file; project launch should instantiate that external executable directly with resolved namespace and parameters.
- `/opt/ros/jazzy/share/robot_localization/params/ekf.yaml` documents the 15-state input-vector order (`x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az`), `two_d_mode`, `publish_tf`, frame parameters, timeouts, and diagnostics.
- Installed `robot_localization` version is 3.8.3 in this Jazzy environment. Treat package APIs as external dependencies; do not modify `/opt/ros`.

## Parameter And Interface Contract
- Invocation: `ros2 launch fleet_localization localization.launch.py fleet_config:=<absolute-or-resolvable-path> robot:=<manifest-name> mode:=localization timeout:=60.0 rviz:=false` for this slice's executable proof. The final aligned default remains `rviz:=true`, but RViz/map behavior is delivered in slice 03; tests disable it explicitly.
- Raw inputs: `/<robot>/wheel/odometry` (`nav_msgs/msg/Odometry`, reliable/volatile), `/<robot>/imu` (`sensor_msgs/msg/Imu`, best-effort/volatile), and `/<robot>/scan` (`sensor_msgs/msg/LaserScan`, best-effort/volatile).
- EKF output: `/<robot>/odometry/filtered` (`nav_msgs/msg/Odometry`, reliable/volatile).
- Health output: `/<robot>/localization/health` (`diagnostic_msgs/msg/DiagnosticArray`). The status name includes the selected robot and component; status message is the stable operator state; key/value pairs list all observed failures.
- Frames: `<robot>/odom`, `<robot>/base_footprint`, `<robot>/base_link`, `<robot>/imu_link`, and `<robot>/base_scan`, all without a leading slash.
- EKF: `two_d_mode: true`, `world_frame: <robot>/odom`, `odom_frame: <robot>/odom`, `base_link_frame: <robot>/base_footprint`, `publish_tf: true`, `publish_acceleration: false`, and `use_sim_time: true`.
- Wheel fusion vector: enable only `vx` and `vyaw`; do not enable wheel pose. In the documented 15-state order: `[false, false, false, false, false, false, true, false, false, false, false, true, false, false, false]`.
- IMU fusion vector: enable only `yaw` and `vyaw`; do not enable roll, pitch, or linear acceleration. In the documented order: `[false, false, false, false, false, true, false, false, false, false, false, true, false, false, false]`.
- Use one `burger_sim` profile for every simulated robot; inject topics and frame IDs at launch rather than copying a profile per robot.
- Required messages are fresh within 1 second of simulation time. Reject zero timestamps after `/clock` is active, future timestamps beyond a small configurable tolerance, out-of-order timestamps beyond the configured tolerance, non-finite measurements/covariances, and missing/zero required covariance entries.
- A fault persisting more than 2 seconds yields degraded health; recovery requires 2 consecutive healthy seconds. Keep tolerances and thresholds in the shared profile.

## Existing Behavior To Preserve
- Slice 01 strict manifest validation, relative path rules, selected-robot lookup, namespace/frame uniqueness, and no unknown-key fallback.
- Atomic fleet simulation behavior and the canonical raw topic/frame contract.
- Simulation ownership of body and sensor TF, including the `base_footprint -> base_link`, IMU, and lidar chains.
- Gazebo simulation timestamps, explicit simulated covariance, and source QoS.
- Failure isolation: this invocation must not terminate Gazebo or another separately started robot invocation.
- No fleet-level localization launcher and no shared global frame.

## Acceptance Criteria
1. A valid invocation for one manifest robot waits for nonzero simulation time and all three canonical sensor topics plus required body/sensor TF before starting `ekf_node`.
2. Readiness succeeds only when topic types, QoS compatibility, frame IDs, timestamps, finite values, and required nonzero covariance satisfy the contract. Timeout defaults to 60 seconds and reports every missing or invalid requirement before this invocation exits nonzero.
3. After readiness, exactly one namespaced EKF consumes only that robot's wheel odometry and IMU and publishes `/<robot>/odometry/filtered` with `header.frame_id == <robot>/odom` and `child_frame_id == <robot>/base_footprint`.
4. The EKF publishes the sole `<robot>/odom -> <robot>/base_footprint` dynamic transform. A test detects and fails duplicate authority or a simulator-owned copy rather than suppressing EKF TF.
5. Profile tests prove wheel odometry enables only forward velocity and yaw rate, IMU enables only yaw and yaw angular velocity, `two_d_mode` is enabled, IMU acceleration and wheel pose are disabled, and frames are selected-robot-prefixed.
6. The filter output contains finite planar pose, twist, and covariance and updates under simulated forward and rotational measurements; no unprefixed frames or cross-robot subscriptions/publishers appear.
7. `/<robot>/localization/health` reports robot-scoped readiness and EKF-active evidence. Runtime stale, future/out-of-order, invalid numeric/covariance, missing TF, or absent EKF output conditions become degraded after 2 seconds; two healthy seconds recover without status flicker.
8. A failed or degraded robot invocation does not stop Gazebo and does not alter another robot's topics, transforms, or process.
9. `colcon test` covers profile, readiness, health, launch, timeout, namespace, and transform-ownership behavior with no network access and no manual GUI action.
10. No AMCL, map server, SLAM Toolbox, map lock, initial pose, map frame, global localization health, or fleet-level localization orchestration is added in this slice.

## Ordered Implementation Tasks
1. Reconcile the slice-01 `fleet_localization` skeleton and fixtures with the likely paths above; confirm the installed Jazzy `robot_localization` executable and parameter names. Stop if the predecessor did not establish the agreed manifest and source contracts.
2. Extend package dependencies and installation data for `robot_localization`, diagnostics, TF, launch testing, the profile, and new executables without vendoring upstream content.
3. Create the one canonical `burger_sim` profile with the exact planar fusion vectors, frame placeholders, simulation-time behavior, explicit timeouts/covariance, and health thresholds; add static tests that reject forbidden or duplicated measurement inputs.
4. Implement shared selected-robot interface resolution from the slice-01 manifest model so launch, readiness, EKF, and health derive topics and frames from one resolved object rather than reconstructing names independently.
5. Implement the bounded readiness executable with aligned subscriptions and TF buffer. Aggregate all unmet conditions, expose useful progress logs/diagnostics, and return an unambiguous success/failure code.
6. Implement launch gating so only a successful readiness exit starts the external `robot_localization/ekf_node` and persistent health process; a failure or timeout shuts down this invocation. Do not start an idle EKF early.
7. Implement persistent health evaluation and `DiagnosticArray` publication with pure/testable state transitions for freshness, validity, TF, estimator output, degradation persistence, and recovery persistence.
8. Add unit and parameter-contract tests covering valid, missing, malformed, stale, future, out-of-order, non-finite, zero-covariance, wrong-frame, and recovery cases.
9. Add launch tests with synthetic simulation time and data to prove the gate, namespacing, EKF output, exclusive odometry TF, timeout report, and isolation between two separately launched robot selections where practical.
10. Run package lint, unit, launch, and synthetic estimator verification; inspect the ROS graph/TF evidence for forbidden unprefixed or duplicate interfaces. Record any tuning-only observations as deviations rather than broadening the slice.

## Verification Commands And Expected Outcomes
Run from the workspace root after sourcing Jazzy. Commands assume the slice-01 workspace layout; adjust only if that approved slice establishes an equivalent canonical path.

```bash
source /opt/ros/jazzy/setup.bash
rosdep check --from-paths src --ignore-src
```

Expected: all declared dependencies for `fleet_localization`, including `robot_localization`, resolve from the Jazzy installation; no vendored dependency is required.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select fleet_localization
```

Expected: the package, readiness/health entry points, launch file, and installed profile build successfully.

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select fleet_localization --event-handlers console_direct+
colcon test-result --verbose
```

Expected: manifest-regression, profile, readiness, health, and launch tests pass; the summary reports zero failed tests.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fleet_localization localization.launch.py fleet_config:=$(pwd)/src/fleet_localization/test/fixtures/fleet_valid.yaml robot:=robot1 mode:=localization timeout:=5.0 rviz:=false
```

Expected with the slice-01 simulation running: readiness completes, one robot1 EKF starts, filtered odometry and health appear, and the command remains attached to only robot1. Expected without sources: it exits nonzero after five simulation seconds (or the documented startup-clock fallback used by tests) and lists `/robot1/wheel/odometry`, `/robot1/imu`, `/robot1/scan`, `/clock`, and required TF as unmet rather than hanging indefinitely.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic info /robot1/odometry/filtered --verbose
ros2 topic echo /robot1/odometry/filtered --once
ros2 topic echo /robot1/localization/health --once
ros2 run tf2_ros tf2_echo robot1/odom robot1/base_footprint
```

Expected with the valid simulation and localization invocation running: one selected-robot filtered-odometry publisher reports reliable/volatile compatibility; the message uses the prefixed odom/base frames and finite covariance; health reports EKF active; the transform resolves and updates. Graph assertions in automated tests, not visual inspection alone, prove there is no second publisher for the odometry edge.

```bash
rg -n "(^|[\"'])/(map|odom|base_link|base_footprint)([\"']|$)|geometry_msgs/msg/Twist$|docker|compose" src/fleet_localization
```

Expected: no hard-coded unprefixed localization frames, obsolete plain-`Twist` interface, or Docker compatibility path. Matches in explicit negative tests or documentation fixtures must be reviewed and justified.

## Risks And Dependencies
- Slice 01 may discover that the official Gazebo plugin cannot suppress odometry TF without a local SDF/plugin override. This slice depends on that override being complete; it must not compensate by turning off EKF TF.
- A readiness timeout measured only in simulation time can never expire before `/clock` begins. The implementation must use a documented monotonic startup deadline while waiting for first clock, then simulation time for data freshness; it must not stamp sensor decisions with wall time after simulation starts.
- ROS 2 launch process-exit gating must distinguish readiness success from failure. If the established launch API cannot reliably branch on exit status and shut down only this invocation, stop and choose a small project-owned launch event handler—not a polling sleep or early-running EKF.
- IMU orientation conventions and covariances must satisfy REP-103/ENU assumptions. If official simulation output is invalid or orientation covariance denotes unavailable data, fix the simulation source contract in slice 01 rather than silently enabling a different EKF input.
- `robot_localization` may transform measurements using body/sensor TF at their timestamps; missing historical transforms can look like estimator silence. Tests must use timestamp-consistent `/clock`, messages, and TF.
- Diagnosing duplicate TF authority from the ROS graph is subtler than counting `/tf` publishers because each publisher carries many edges. Use a focused test listener/authority observation or controlled launch fixture, and treat ambiguity as a stop condition.
- Process-noise and measurement covariance tuning can affect numeric trajectories. This slice proves finite, responsive planar estimation and correct input selection; precision tuning beyond aligned health thresholds must not expand the scope.

## Stop Conditions And Blocked Behavior
Stop implementation and report back when:
- Slice 01 is absent, incomplete, or contradicts the selected manifest, topic, QoS, covariance, frame, or TF-ownership contract.
- The installed Jazzy `robot_localization` API differs materially from the inspected 3.8.3 `ekf_node` parameters or cannot publish the required prefixed transform.
- The official simulation cannot provide valid wheel/IMU covariance, timestamps, or prefixed sensor frames without broad upstream reimplementation.
- Reliable readiness-before-EKF launch or per-invocation failure isolation cannot be achieved with a bounded project-owned mechanism.
- Tests cannot observe transform ownership reliably, required verification commands cannot run, or success would depend on manual GUI inspection.
- The work requires AMCL, map server, SLAM Toolbox, map persistence/locking, a fleet launcher, physical-robot adapters, or broad unrelated changes.
- The slice grows beyond one robot's readiness, EKF, local transform, and local-estimation health.

When blocked, preserve the working tree, record the exact command/output and contract conflict, and request a planning/alignment correction. Do not add fallback topics, frames, modes, or alternate TF ownership.

## Preserve User Work
- Inspect `git status` before edits once the repository exists.
- Preserve unrelated tracked and untracked user changes.
- Do not reset, overwrite, reformat, or commit unrelated paths.
- If a user change overlaps a planned file, stop and report the overlap before replacing it.

## Expected Final Summary
The implementing agent's final report must include:
- Slice 02 status and commit hash, if committed.
- Files/components added or changed, grouped as EKF profile, readiness gate, launch gating, health, and tests.
- The exact selected-robot topics and frame ownership proven.
- Verification commands run and concise pass/fail results, including `colcon test-result`.
- Evidence that EKF starts only after readiness, publishes finite filtered odometry, and solely owns `<robot>/odom -> <robot>/base_footprint`.
- Evidence for timeout aggregation, degradation/recovery, namespace isolation, and absence of AMCL/mapping scope.
- Deviations, unresolved risks, and deferred `FUP-001` through `FUP-005` status.
- Unrelated user changes intentionally preserved.

## Readiness Gate Result
Ready for implementation after slice 01 is implemented and this slice plan is explicitly approved. The plan identifies exact scope, non-goals, deferred scope, likely files and installed patterns, required cleanup, acceptance criteria, ordered tasks, verification, risks, blocked behavior, preservation rules, and final-report content. This file is the implementation source for slice 02; [3_plan.md](3_plan.md) is not.
