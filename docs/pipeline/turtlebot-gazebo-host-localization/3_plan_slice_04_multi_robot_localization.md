# Slice 04 Plan: Independent Multi-Robot Localization

## Implementation Status
Status: planned
Commit:
Verification:
Deviations:
Follow-ups:

## Context
- Repository: `/home/syncrobot/localization` (not initialized as a Git repository at planning time)
- Branch: unavailable; implementation should use the feature branch established by Slice 01
- Feature folder: `docs/pipeline/turtlebot-gazebo-host-localization`
- Selected vertical slice: **Independent Multi-Robot Localization**
- Sequence: 04 of 06
- Design source: [1_design.md](1_design.md)
- Alignment source: [2_alignment.md](2_alignment.md)
- Roadmap source: [3_plan.md](3_plan.md)
- Routing source: [CURRENT.md](CURRENT.md)
- UX source: [../../about/05_ux_decision_catalogue.md](../../about/05_ux_decision_catalogue.md)
- Required predecessors: Slice 01's complete two-robot simulation and strict manifest contract, Slice 02's independently selected EKF/readiness/health path, and Slice 03's complete single-robot map-server/AMCL/RViz workflow and shared map-store read lock.

## Exact Slice
Prove the existing one-robot localization invocation is safely repeatable by running it as two independent processes for `robot1` and `robot2` against one immutable map ID. Both sessions must coexist while retaining separate nodes, lifecycle managers, topics, prefixed TF trees, RViz views, initialization inputs, health state, and failure boundaries. Both processes may hold the map store's shared/read lock concurrently.

This slice is primarily concurrency hardening and end-to-end proof. It must repair only identity leakage, global-name collisions, process-coupled shutdown, or non-reentrant resource handling discovered in the Slice 01-03 implementation. It does not add a fleet localization launcher, a common map frame, mapping, map writes, teleoperation, an alternate world, or performance guarantees beyond two robots.

## Repository Rules
- Consulted: [1_design.md](1_design.md), [2_alignment.md](2_alignment.md), [3_plan.md](3_plan.md), [CURRENT.md](CURRENT.md), Slice plans 01-03, and [../../about/05_ux_decision_catalogue.md](../../about/05_ux_decision_catalogue.md). `docs/rules/README.md` and repository rule files do not exist.
- Constraints applied: one selected robot per invocation; two invocations are started separately; identical map bytes do not create a shared TF frame; all runtime identity is derived from the selected manifest robot; shared map access is read-only and concurrent; operator controls and status remain robot-scoped; no compatibility aliases or orchestration shortcut.
- Rule updates: None

## Follow-Up Scope
- Included deferred items: None.
- Still deferred:
  - `FUP-001`: performance guarantees beyond two robots. This slice functionally verifies exactly two concurrent instances, not capacity for N.
  - `FUP-002`: real non-TurtleBot sensor profiles.
  - `FUP-003`: automatic initial pose.
  - `FUP-004`: shared global multi-robot frame. The two map frames remain independent even though their occupancy data is identical.
  - `FUP-005`: concurrent or collaborative mapping.

## Central Issue Scope
- Included register IDs: None.
- Still deferred register IDs: None.
- Newly created register IDs: None. No central issue register exists, and this slice introduces no durable issue beyond the approved follow-ups.

## Explicit Non-Goals
- Creating a launcher, supervisor, lifecycle manager, RViz process, or health aggregator that starts or controls the fleet as one unit.
- Joining `robot1/map` and `robot2/map`, overlaying both robots in one RViz fixed frame, or introducing an unprefixed global `map`.
- Adding SLAM Toolbox, an exclusive map lock, map-save APIs, map mutation, or mapping downtime behavior; those belong to Slice 05.
- Adding the custom world, safe keyboard teleoperation, GUI/headless hardening, or final fault matrix; those belong to Slice 06.
- Changing the EKF/AMCL tuning, readiness thresholds, map format, map metadata, health thresholds, or manual-initialization model unless a repeatability defect proves the existing implementation violates alignment.
- Automatic, persisted, copied, or broadcast initial poses.
- Guaranteed fleet performance beyond the verified two-robot case.

## Canonical Mental Model And Required Cleanup
The operator starts the same canonical launch twice, with a different required robot selection each time:

```text
process A: robot:=robot1                 process B: robot:=robot2
  shared read lock                         shared read lock
  robot1 EKF                               robot2 EKF
  robot1 map server                        robot2 map server
  robot1 AMCL                              robot2 AMCL
  robot1 lifecycle/health/RViz             robot2 lifecycle/health/RViz

same immutable map files; no shared ROS node, frame, lifecycle, or operator state
```

Required cleanup is limited to defects that prevent this model:
- Replace hard-coded `robot1`, global node names, absolute cross-robot topics, unprefixed frames, fixed lock ownership IDs, and shared temporary parameter/RViz filenames with values scoped to the selected robot and process.
- Ensure generated runtime files use process-private temporary directories and deterministic cleanup; never rewrite installed templates or create durable per-robot copies.
- Ensure stopping or failing one launch does not signal, lifecycle-transition, unlock, or remove resources owned by the other.
- Do not solve collisions by adding aliases, random ROS names, a fleet wrapper, separate map copies, separate ROS domains, or separate TF topics. Namespaces plus unique prefixed frame IDs are the canonical mechanism on one ROS graph.

## Reconnaissance Findings And Existing Patterns
- The workspace contains pipeline documents only, so implementation must first reconcile this plan against the actual files delivered by Slices 01-03. There is no current source tree or Git history to inspect.
- Installed Jazzy packages include `robot_localization`, `nav2_amcl`, `nav2_map_server`, `nav2_lifecycle_manager`, `rviz2`, and SLAM Toolbox. Slice 04 should reuse the launch and parameter interfaces already proven in Slice 03 rather than add upstream wrappers.
- Nav2's namespaced localization pattern relies on unique node names/namespaces, rewritten parameters, and shared `/tf` and `/tf_static` transport with unique frame IDs. Do not isolate each robot onto namespaced TF topics.
- Multiple map servers can read one occupancy YAML/image concurrently. The project map-store shared lock from Slice 03 is the authority that permits these readers and must be process-owned rather than globally singleton state.
- ROS 2 launch testing can start two independently included launch descriptions, but production behavior remains two separate operator invocations. A test-only harness is acceptable; it must not be installed or documented as a supported fleet launcher.

## Likely Files, Modules, Components, And Tests
Reconcile exact names with completed prerequisite slices and extend canonical components rather than creating parallel implementations.

### Existing Components To Harden If Evidence Requires
- `src/fleet_localization/launch/localization.launch.py` — guarantee every node, lifecycle target, parameter rewrite, temporary path, RViz substitution, and shutdown handler is scoped to the selected robot/process; do not add multi-robot launch arguments.
- `src/fleet_localization/fleet_localization/map_store_lock.py` — prove two independent shared holders coexist and that releasing one holder does not affect the other. Preserve the same API Slice 05 will extend with exclusive locking.
- `src/fleet_localization/fleet_localization/localization_health.py` — ensure subscriptions, publications, TF lookups, lifecycle observations, and state transitions cannot consume another robot's data.
- `src/fleet_localization/rviz/robot_localization.rviz` and its runtime substitution helper — generate process-private robot-specific views without modifying the installed template.
- `src/fleet_localization/config/burger_ekf.yaml` and `config/burger_amcl.yaml` — remain single shared profiles. Only launch-time identity injection is allowed; do not create `robot1`/`robot2` copies.

### New Or Extended Tests
- `src/fleet_localization/test/test_multi_robot_launch.py` — test-only orchestration of two separate localization launch instances; assert unique node/lifecycle names, no global node collisions, and independent shutdown.
- `src/fleet_localization/test/test_multi_robot_interfaces.py` — assert exact topic ownership and message types; reject cross-robot subscriptions/publications and unprefixed localization frames; inspect TF publishers for one owner per required edge.
- `src/fleet_localization/test/test_multi_robot_map_lock.py` — two process-level shared holders succeed; releasing or crashing one preserves the other's lock; a prospective exclusive acquisition remains blocked until both readers exit. The exclusive mapping implementation remains Slice 05.
- `src/fleet_localization/test/test_multi_robot_initialization.py` — a pose on `/robot1/initialpose` changes only robot1 state and health; robot2 remains awaiting its own pose until `/robot2/initialpose` is supplied.
- `src/fleet_localization/test/test_multi_robot_failure_isolation.py` — terminate or fault one localization process/sensor path and prove the other lifecycle nodes, transforms, lock, and health continue normally.
- `src/fleet_localization/test/test_multi_robot_rviz.py` — structural/process-temporary configuration proof that each session has its own fixed frame and selected-robot topics. Use `rviz:=false` for authoritative headless integration where GUI is unavailable.
- A test fixture or script under `src/fleet_localization/test/` may start both invocations for CI. It must remain test-only and must not become an installed fleet launcher.

## Behavior That Must Not Regress
- Slice 01's strict YAML validation, complete two-robot atomic simulator startup, canonical sensor topics, prefixed source frames, covariance/QoS/time contract, and exclusion of simulator odometry TF.
- Slice 02's exact planar fusion inputs, sole EKF ownership of each odometry transform, 60-second per-invocation readiness timeout, and isolated health behavior.
- Slice 03's immutable world-matched map resolution, shared/read lock lifetime, namespaced map server and AMCL lifecycle, map QoS, manual initial pose, robot-only RViz, measurable health thresholds, and sole AMCL ownership of each map transform.
- Both sessions use the same map ID and underlying bytes while publishing separate `/<robot>/map` topics with frames `<robot>/map`.
- Simulation continues if either or both localization invocations exit.

## Acceptance Criteria
1. Two separately issued `localization.launch.py` commands for `robot1` and `robot2` run concurrently against the same fleet manifest and map ID; no fleet-level production launcher is introduced.
2. Each process starts exactly one EKF, map server, AMCL, lifecycle manager, health node, and optional RViz for its selected robot. Node names and lifecycle service targets do not collide.
3. Both processes hold shared/read access to the same canonical map store concurrently. Exiting or crashing one releases only its holder, leaves the other operational, and does not permit an exclusive holder until all readers exit.
4. Each map server reads the same resolved map assets but publishes only `/<robot>/map` with frame `<robot>/map`; neither publishes an unprefixed `/map` or a shared frame.
5. Each EKF and AMCL consumes only its selected robot's inputs and exclusively owns `<robot>/odom -> <robot>/base_footprint` and `<robot>/map -> <robot>/odom`, respectively. No duplicate or cross-robot TF edge exists.
6. Manual initialization on `/<robot>/initialpose` affects only that robot. Each health state independently remains `awaiting initial pose`, becomes `localized`, or becomes `degraded` from its own inputs and covariance.
7. Two default RViz sessions, when GUI is enabled, each show only the selected robot and use its `<robot>/map` fixed frame. Closing one RViz does not terminate or alter either localization graph.
8. Loss or termination of one robot's localization invocation leaves the other robot's lifecycle nodes active, map available, TF chain resolvable, lock held, and health current. Gazebo remains running.
9. Restarting the failed invocation succeeds without restarting simulation or the healthy invocation and without stale nodes, locks, temporary files, or TF publishers.
10. Automated tests detect global names, unprefixed localization frames, cross-robot topic use, per-robot tuning-file copies, process-global temporary files, and shutdown coupling.

## Ordered Implementation Tasks
1. Verify Slices 01-03 satisfy their own acceptance criteria in isolation. Stop if the canonical one-robot workflow, complete two-robot source graph, shared map lock, or health extension point is missing.
2. Audit the implemented launch graph and runtime-generated artifacts for hard-coded robot identity, global node/lifecycle names, absolute topic leakage, unprefixed frames, shared temporary paths, singleton lock state, and broad shutdown handlers.
3. Add failing concurrency-focused unit and launch tests before changing canonical components. Keep any two-instance harness in test scope only.
4. Make the minimum reentrancy fixes in existing launch/config/health/lock/RViz helpers. Derive all identity from the selected manifest entry and ensure process-private cleanup.
5. Add process-level shared-lock tests proving two readers, independent release/crash handling, and continued exclusion of a writer until the final reader exits.
6. Add two-robot interface and TF ownership assertions against the Slice 01 Gazebo fleet and two independent localization processes.
7. Add independent manual-initialization and health-transition tests, including robot1 localization while robot2 remains uninitialized and the inverse.
8. Add failure/restart evidence by terminating or faulting one invocation and proving the other session and Gazebo remain healthy before restarting the failed session.
9. Perform a host GUI check with two separate robot-scoped RViz sessions; retain structural/headless tests as the reproducible automated proof.
10. Update usage documentation only to show two separate shell invocations and their independent lifecycle. Do not document a fleet launcher, mapping, custom world, or teleoperation as implemented.

## Verification Commands And Expected Outcomes
Run from `/home/syncrobot/localization` after sourcing ROS 2 Jazzy and the built workspace. Adapt only to the canonical workspace layout established by Slice 01.

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to turtlebot_fleet_sim fleet_localization
```

Expected: prerequisite and localization packages build with external Jazzy dependencies; no per-robot package/config copies or new fleet orchestration package is required.

```bash
colcon test --packages-select fleet_localization --event-handlers console_direct+
colcon test-result --verbose
```

Expected: all existing single-robot tests plus multi-robot launch, interface, lock, initialization, RViz, and failure-isolation tests pass with zero failures.

Start Slice 01's default two-robot simulation, then in two separate terminals run:

```bash
ros2 launch fleet_localization localization.launch.py fleet_config:=<absolute-fleet-yaml> robot:=robot1 mode:=localization rviz:=true
```

```bash
ros2 launch fleet_localization localization.launch.py fleet_config:=<absolute-fleet-yaml> robot:=robot2 mode:=localization rviz:=true
```

Expected: both commands remain independent, both acquire shared map access, and each creates only its selected robot's nodes, topics, frames, health, and RViz session.

```bash
ros2 lifecycle get /robot1/map_server
ros2 lifecycle get /robot1/amcl
ros2 lifecycle get /robot2/map_server
ros2 lifecycle get /robot2/amcl
ros2 topic echo /robot1/map --once
ros2 topic echo /robot2/map --once
ros2 run tf2_ros tf2_echo robot1/odom robot1/base_footprint
ros2 run tf2_ros tf2_echo robot2/odom robot2/base_footprint
ros2 run tf2_ros tf2_echo robot1/map robot1/odom
ros2 run tf2_ros tf2_echo robot2/map robot2/odom
```

Expected: all lifecycle nodes are active after readiness; map messages contain their corresponding prefixed frame; each required transform resolves only within its robot tree and has one owner.

Set only robot1's pose using robot1 RViz, then inspect both robot-scoped health topics using the exact health topic established by Slice 02. Repeat with robot2.

Expected: robot1 may become localized while robot2 remains awaiting initial pose; robot2 initialization never changes robot1's pose or state.

Terminate the robot1 localization command while leaving robot2 and Gazebo running, then run:

```bash
ros2 lifecycle get /robot2/map_server
ros2 lifecycle get /robot2/amcl
ros2 topic echo /robot2/map --once
ros2 run tf2_ros tf2_echo robot2/map robot2/odom
```

Expected: robot2 remains active and healthy, its map and TF remain available, and Gazebo continues. Restarting the robot1 command succeeds without touching robot2.

```bash
ros2 node list
ros2 topic list
ros2 run tf2_tools view_frames
```

Expected: the graph contains no duplicate global localization node names, no cross-robot localization topics, and no unprefixed or shared localization frames. Generated TF evidence shows two disjoint prefixed trees.

## Risks And Dependencies
- **Prerequisite reentrancy:** Slice 03 may pass for one process while using global temporary filenames, broad process cleanup, or singleton lock state. Concurrency tests must expose and remove those defects in canonical components.
- **TF transport versus frame isolation:** Both robots must share `/tf` and `/tf_static` transport while using unique frame IDs. Namespacing TF topics would fragment visibility; unprefixed frames would collide.
- **Nav2 lifecycle naming:** A default lifecycle manager name or absolute lifecycle service path can couple sessions. Verify node and managed-node names after namespace resolution.
- **RViz template substitution:** Simultaneous template rewriting can race or cross-wire initial-pose topics. Use immutable installed templates and process-private generated output.
- **Lock semantics:** Thread-only tests do not prove process-level advisory locking. Include separate-process acquisition, abnormal exit, and final-reader behavior on the host filesystem.
- **Fault injection:** Stopping a simulated sensor may affect Gazebo plugins more broadly than one robot. Prefer terminating one localization process for the required isolation proof, then add source fault injection only where Slice 01 provides a robot-scoped mechanism.
- **Resource load:** Two Gazebo lidars, two AMCL filters, and two RViz processes may stress the host. Functional verification covers two; performance characterization remains `FUP-001`.

## Stop Conditions And Blocked Procedure
Stop implementation and report back when:
- Any prerequisite slice is incomplete or conflicts with this plan's canonical one-robot invocation or two-robot source graph.
- Two instances cannot coexist without a fleet launcher, separate ROS domains, duplicated map stores, namespaced TF transport, or a shared global map frame.
- Installed Jazzy Nav2 lifecycle or TF behavior requires compatibility aliases or duplicate ownership forbidden by alignment.
- Correct process-level shared locking cannot be achieved with the Slice 03 operating-system lock model.
- Failure isolation requires simulator, EKF, AMCL, map-server, or health redesign outside targeted reentrancy fixes.
- Required automated tests, two-robot Gazebo proof, or host GUI verification cannot be run.
- Work expands into Slice 05 mapping, Slice 06 operator/simulation hardening, deferred scope, or broad unrelated changes.

When blocked, preserve the working tree, record the exact command/error and affected acceptance criterion, identify whether Slice 01-03 must be repaired, and describe the smallest alignment or prerequisite change needed. Do not add fallback topic paths, aliases, duplicate configurations, or a partial fleet implementation.

## Preserve User Work
- Inspect `git status` before editing once the repository exists.
- Preserve unrelated tracked and untracked user changes.
- Do not reset, overwrite, reformat, or commit unrelated paths.
- If prerequisite slice work is concurrently changing a required file, stop and coordinate rather than replacing it.

## Expected Final Summary
The implementing agent must report:
- Canonical components changed for reentrancy, grouped by launch/lifecycle identity, locking, TF/topics, RViz/runtime files, health, and tests.
- The two separate launch commands and proof that they used one immutable map ID while retaining independent map topics and frames.
- Manual-initialization, health, TF-ownership, lock, failure, restart, and GUI/headless verification outcomes.
- Commands run, results, unavailable checks, and the slice implementation-status block update including commit, deviations, and follow-ups.
- All still-deferred `FUP-001` through `FUP-005`; do not claim mapping, alternate-world, teleoperation, or fleets larger than two are complete.
