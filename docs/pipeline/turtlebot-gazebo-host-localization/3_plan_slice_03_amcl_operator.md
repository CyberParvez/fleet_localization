# Slice 03 Plan: Single-Robot AMCL And Operator Session

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
- Selected vertical slice: **Single-Robot AMCL And Operator Session**
- Sequence: 03 of 06
- Source design: [1_design.md](1_design.md)
- Source alignment: [2_alignment.md](2_alignment.md)
- Roadmap: [3_plan.md](3_plan.md)
- UX decisions: [../../about/05_ux_decision_catalogue.md](../../about/05_ux_decision_catalogue.md)
- Prerequisites: Slice 01's strict manifest/map-store resolution and simulated sensor/TF contract; Slice 02's selected-robot launch, readiness gate, EKF, odometry TF ownership, and health publisher.

## Slice Goal
Extend one separately selected robot's working EKF invocation into a complete existing-map localization session. Resolve one immutable, world-compatible map ID; hold a shared map-store session lock; automatically lifecycle-manage a namespaced map server and AMCL after readiness; accept a manual pose only on that robot's interface; open a robot-only RViz session by default; and report `awaiting initial pose`, `localized`, or `degraded` from observable data rather than process existence.

This slice proves one end-to-end operator workflow. It does not yet prove two concurrent localization invocations; that is Slice 04.

## Repository Rules
- Consulted: [1_design.md](1_design.md), [2_alignment.md](2_alignment.md), [3_plan.md](3_plan.md), [CURRENT.md](CURRENT.md), and [../../about/05_ux_decision_catalogue.md](../../about/05_ux_decision_catalogue.md). `docs/rules/README.md` and repository rule files do not exist.
- Constraints applied: one selected robot per invocation; one canonical strict configuration path; prefixed frames without leading `/`; independent map frame/topic; automatic lifecycle after readiness; immutable map IDs; robot-scoped RViz and initialization; measurable health; no compatibility aliases or fleet launcher.
- Rule updates: None

## Follow-Up Scope
- Included: None. This slice implements only approved current scope.
- Still deferred: `FUP-001` performance guarantees beyond two robots; `FUP-002` real non-TurtleBot sensor profiles; `FUP-003` automatic initial pose; `FUP-004` shared global multi-robot frame; `FUP-005` concurrent or collaborative mapping.
- Do not turn manual initialization into remembered, inferred, or automatic initialization under `FUP-003`.
- Do not add a shared `map` frame or fleet-combined RViz under `FUP-004`.

## Central Issue Scope
- Included register IDs: None.
- Still deferred register IDs: None.
- Newly created register IDs: None; the repository has no `docs/issues/README.md` or issue-register schema, and the alignment intentionally preserves the feature-local `FUP-###` items instead.

## Canonical Mental Model
One `fleet_localization` invocation selects exactly one manifest robot and owns one independent localization graph:

```text
resolved immutable map files -> /<robot>/map [frame: <robot>/map]
                                             |
/<robot>/scan + EKF odometry --------------> AMCL
                                             |
                          <robot>/map -> <robot>/odom

RViz fixed frame: <robot>/map
RViz 2D Pose Estimate -> /<robot>/initialpose
AMCL estimate         -> /<robot>/amcl_pose
```

The map image and YAML may later be read by other independent invocations, but this invocation publishes its own namespaced map topic and prefixed map frame. Identical map bytes do not imply a shared TF tree.

### Required Cleanup And Removals
- Do not include or document Nav2 navigation, costmaps, planners, controllers, behavior trees, goal tools, or a full `nav2_bringup` launch.
- Do not publish an unprefixed `map`, `odom`, `base_footprint`, `base_link`, scan frame, or sensor frame.
- Do not introduce a shared global map frame, combined fleet view, fleet-level localization launcher, automatic initial pose, map-path fallback, mutable map ID, or map copying into an installed package share.
- Do not leave lifecycle activation available only as a manual service sequence; the selected invocation owns automatic activation after readiness.
- Do not treat an active AMCL process or a single pose message as proof of localization.
- Do not copy Nav2 source or maintain a second supported wrapper around the same localization path.

## Reconnaissance Findings And Existing Patterns
- The workspace contains planning documents only; there is no ROS workspace, package, code, test pattern, or Git history to preserve. Slice 01/02 must establish the actual `fleet_localization` package layout before this slice begins.
- Installed Jazzy packages include `nav2_amcl`, `nav2_map_server`, `nav2_lifecycle_manager`, and `rviz2`. At planning time the Nav2 packages are Debian version `1.3.12` and RViz2 is `14.1.22`; package manifests should depend on Jazzy package names rather than hard-code these observed patch versions.
- `/opt/ros/jazzy/share/nav2_bringup/launch/localization_launch.py` is the upstream pattern for a namespaced `map_server`, `amcl`, and `lifecycle_manager_localization`, parameter rewriting, relative `tf`/`tf_static` remaps, and lifecycle node order `['map_server', 'amcl']`.
- `/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml` confirms Jazzy AMCL parameter names including `base_frame_id`, `global_frame_id`, `odom_frame_id`, `scan_topic`, `robot_model_type: nav2_amcl::DifferentialMotionModel`, `laser_model_type: likelihood_field`, and `tf_broadcast`.
- `/opt/ros/jazzy/share/nav2_bringup/launch/rviz_launch.py` and `rviz/nav2_namespaced_view.rviz` demonstrate namespace substitution and explicit remapping of `/initialpose`, `/map`, `/tf`, and `/tf_static`. The project config must be smaller and set the fixed frame to `<robot>/map`, not upstream's unprefixed `map`.
- Nav2 `map_server` publishes a transient-local map; preserve the aligned reliable/transient-local map contract. Sensor QoS and simulation time remain the Slice 01/02 contract.

## Likely Files, Modules, Components, And Tests
Paths below assume the package layout created by prerequisite slices; reconcile names with that layout rather than creating duplicate modules.

### Package And Launch
- `src/fleet_localization/package.xml` — add runtime/test dependencies for `nav2_amcl`, `nav2_map_server`, `nav2_lifecycle_manager`, `rviz2`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `tf2_ros`, and any launch-testing packages actually used.
- `src/fleet_localization/setup.py` and/or `CMakeLists.txt` — install localization parameters, RViz config, bundled default map assets/metadata if Slice 01 did not already install them, and new launch/support modules.
- `src/fleet_localization/launch/localization.launch.py` — extend the canonical single-robot launch arguments (`fleet_config`, required `robot`, `mode:=localization`, optional `map_id`, `rviz`, `timeout`); resolve configuration before starting nodes; preserve the Slice 02 readiness gate; start exactly one namespaced map server, AMCL, lifecycle manager, health monitor, and optional RViz.
- `src/fleet_localization/fleet_localization/map_catalog.py` (or Slice 01's canonical resolver) — resolve `map_store` relative to the manifest, validate safe `map_id`, require `map.yaml`, referenced image, and world metadata, reject symlink/path escape and obvious active-world mismatch, and never search fallback locations.
- `src/fleet_localization/fleet_localization/map_store_lock.py` — session-scoped operating-system shared/read lock held for the complete localization invocation and automatically released at process exit. Establish the shared side now; Slice 05 adds the exclusive mapping side. Do not use a persistent boolean marker as authority.

### Localization Configuration And Operator Surface
- `src/fleet_localization/config/burger_amcl.yaml` — one reusable Burger profile with differential motion and likelihood-field models; launch injects `<robot>/map`, `<robot>/odom`, `<robot>/base_footprint`, `/<robot>/scan`, simulation time, and TF broadcasting rather than copying per-robot files.
- `src/fleet_localization/rviz/robot_localization.rviz` — localization-only displays for the selected robot: map, scan, robot model, TF, filtered odometry, and AMCL pose; fixed frame `<robot>/map`; 2D Pose Estimate targets `/<robot>/initialpose`; omit navigation panels/tools and other robot topics.
- `src/fleet_localization/fleet_localization/localization_health.py` (extend Slice 02's canonical health node rather than add a second monitor) — consume lifecycle state, readiness state, filtered odometry, AMCL pose/covariance, sensor freshness, and TF availability; publish selected-robot health transitions with configured persistence thresholds.
- Robot-specific launch substitutions or a generated temporary parameter/RViz view — use launch substitutions or process-temporary output; do not modify installed templates or create durable per-robot copies.

### Maps
- Canonical `maps/turtlebot3_world_v1/map.yaml`, referenced image, and project-defined world metadata at the manifest-relative `map_store`, if Slice 01 did not already supply the default world-matched map set. Map content must actually match the default Gazebo world; do not label an unrelated upstream Nav2 demo map as TurtleBot3 World.

### Tests
- `src/fleet_localization/test/test_map_catalog.py` — valid resolution; relative path behavior; unsafe IDs; missing YAML/image/metadata; malformed occupancy YAML; path escape; active-world mismatch; no fallback; immutable read behavior.
- `src/fleet_localization/test/test_map_store_lock.py` — multiple shared holders succeed; acquisition/release on normal exit and exception; future exclusive acquisition is blocked by a held read lock; no stale authority after process exit.
- `src/fleet_localization/test/test_localization_launch.py` — launch argument/default resolution; required robot; map override precedence; one namespace only; exact lifecycle node list; `rviz:=false`; missing or mismatched map fails before Nav2 activation.
- `src/fleet_localization/test/test_amcl_parameters.py` — injected prefixed frames/topics, `use_sim_time`, differential model, likelihood-field model, and identical profile values for different robot identities without copied configs.
- `src/fleet_localization/test/test_localization_health.py` — awaiting-initial-pose behavior; finite/current pose requirement; `0.25 m²` planar position and `0.10 rad²` yaw covariance boundaries; 1-second freshness; 2-second degradation and recovery persistence; stale data and broken TF.
- `src/fleet_localization/test/test_robot_rviz_config.py` — fixed frame, exact selected-robot topics, initial-pose target, required displays, and absence of navigation/fleet topics.
- `src/fleet_localization/test/test_amcl_integration.py` — one simulated robot reaches active map server/AMCL lifecycle, receives transient-local map, accepts namespaced manual initial pose, owns only `<robot>/map -> <robot>/odom`, and reaches localized health after finite current estimates within thresholds.

## Behavior That Must Not Regress
- Slice 01's strict manifest validation, active-world selection, namespaced raw topics, prefixed body/sensor frames, and simulator exclusion from odometry TF.
- Slice 02's required robot selection, 60-second configurable readiness timeout, EKF fusion policy, `use_sim_time`, QoS compatibility, sole ownership of `<robot>/odom -> <robot>/base_footprint`, and per-invocation failure boundary.
- Simulation continues to run when one localization invocation fails.
- `rviz:=false` leaves the localization graph running without a GUI.
- All project nodes and upstream localization nodes use Gazebo time.

## Acceptance Criteria
1. Starting the canonical localization launch with a valid manifest and robot selects exactly one robot and resolves the manifest-relative default `map_id`; an explicit safe `map_id` overrides the fleet default.
2. Missing, malformed, unsafe, incomplete, or active-world-mismatched map data fails before map server or AMCL activation and reports the exact map ID/reason without fallback.
3. The invocation holds one shared map-store lock throughout the localization session; another shared localization holder is allowed, while an exclusive holder cannot coexist; process exit releases the lock.
4. After Slice 02 readiness succeeds, the namespaced lifecycle manager automatically configures and activates only that robot's `map_server` and `amcl`; lifecycle failure terminates only this invocation with an actionable error.
5. The map server publishes `/<robot>/map` reliably/transient-locally with frame `<robot>/map` from the resolved immutable files.
6. AMCL consumes `/<robot>/scan`, uses `<robot>/map`, `<robot>/odom`, and `<robot>/base_footprint`, uses the differential-drive and likelihood-field profiles, and exclusively broadcasts `<robot>/map -> <robot>/odom`.
7. Manual initialization sent to `/<robot>/initialpose` affects only the selected AMCL instance; no automatic or persisted initial pose is supplied.
8. RViz opens by default with fixed frame `<robot>/map` and only selected-robot map, scan, model, TF, filtered odometry, AMCL pose, and initialization interfaces. `rviz:=false` suppresses it without changing localization.
9. Health reports `awaiting initial pose` until a post-initialization AMCL estimate exists. `localized` additionally requires fresh sensor/EKF/AMCL data, a current resolvable map-to-sensor TF chain, finite state/covariance, planar position covariance at or below `0.25 m²`, and yaw covariance at or below `0.10 rad²`.
10. A health violation persisting beyond 2 seconds reports `degraded`; recovery requires 2 consecutive healthy seconds. Thresholds are configurable in the shared Burger profile.
11. Tests prove there are no unprefixed frames, cross-robot topics, navigation nodes/tools, duplicate map/odom TF publishers, per-robot copied tuning files, or hidden map paths.

## Ordered Implementation Tasks
1. Reconcile this plan with the completed Slice 01/02 package layout and tests. Stop if those slices do not provide strict configuration selection, selected-robot readiness, EKF output, or the health-state extension point assumed here.
2. Add focused map catalogue resolution and validation on top of the canonical manifest model: relative store resolution, safe immutable ID, required occupancy assets, metadata schema, and active simulation-world association.
3. Add a session-owned shared/read map-store lock with deterministic cleanup and unit tests; expose one API that Slice 05 can extend with exclusive acquisition rather than a second locking mechanism.
4. Add the single reusable Burger AMCL profile and launch-time injection of the selected robot's namespace and prefixed frames. Verify exact Jazzy parameter names from installed package interfaces before implementation.
5. Extend the one-robot launch to acquire the map lock, wait for Slice 02 readiness, start the namespaced map server and AMCL, and automatically lifecycle-manage exactly those two nodes. Ensure map validation/lock failure precedes lifecycle activation.
6. Build the minimal robot-scoped RViz config and substitution path, default it on, and ensure closing or disabling RViz does not silently change localization ownership or start another robot.
7. Extend the existing health monitor with lifecycle, initial-pose/AMCL estimate, covariance, freshness, finite-value, and current-time TF conditions plus 2-second degradation/recovery persistence.
8. Add unit and launch tests for map validation, locking, parameters, lifecycle scoping, RViz routing, and health thresholds.
9. Run a one-robot Gazebo integration: verify map receipt, lifecycle states, pre-initialization health, manual initialization, scan alignment, TF ownership, localized health, and isolated failure behavior.
10. Update package-facing usage documentation only as needed to describe this one-robot localization invocation and its observable states; do not document Slice 04 concurrency or Slice 05 mapping as completed.

## Verification Commands And Expected Outcomes
Run from `/home/syncrobot/localization` after sourcing ROS 2 Jazzy. Adjust only the workspace directory if Slice 01 establishes a different canonical root; do not bypass failing checks.

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Expected: all declared Jazzy dependencies resolve; no vendored upstream localization source is required.

```bash
colcon build --symlink-install --packages-up-to fleet_localization
```

Expected: the package and installed launch/config/RViz resources build without warnings that indicate missing files or parameters.

```bash
colcon test --packages-select fleet_localization --event-handlers console_direct+
colcon test-result --verbose
```

Expected: map catalogue, shared lock, AMCL parameter, launch, RViz, health, and integration tests pass with zero failures.

With Slice 01's default simulation running and the workspace sourced:

```bash
ros2 launch fleet_localization localization.launch.py fleet_config:=<absolute-fleet-yaml> robot:=robot1 mode:=localization rviz:=true
```

Expected: readiness precedes activation; only `/robot1/map_server`, `/robot1/amcl`, that invocation's lifecycle/health/EKF nodes, and robot1 RViz are started; health waits for manual initialization.

```bash
ros2 lifecycle get /robot1/map_server
ros2 lifecycle get /robot1/amcl
ros2 topic info /robot1/map --verbose
ros2 topic echo /robot1/map --once
ros2 param get /robot1/amcl global_frame_id
ros2 param get /robot1/amcl odom_frame_id
ros2 param get /robot1/amcl base_frame_id
ros2 param get /robot1/amcl scan_topic
```

Expected: both lifecycle nodes are active; map durability is transient-local and reliability is reliable; the map message frame is `robot1/map`; AMCL parameters resolve to `robot1/map`, `robot1/odom`, `robot1/base_footprint`, and the robot1 scan interface.

After setting robot1's initial pose using that RViz session and moving the robot enough to converge:

```bash
ros2 topic echo /robot1/amcl_pose --once
ros2 run tf2_ros tf2_echo robot1/map robot1/odom
ros2 node list
ros2 topic list
```

Expected: AMCL emits a current finite pose/covariance, the map-to-odom transform resolves, health reaches `localized` only within configured limits, and no robot2 localization nodes/topics or unprefixed localization frames are created by this invocation.

```bash
ros2 launch fleet_localization localization.launch.py fleet_config:=<absolute-fleet-yaml> robot:=robot1 mode:=localization rviz:=false map_id:=does_not_exist
```

Expected: launch fails before Nav2 lifecycle activation with an exact immutable-map resolution error and no fallback map.

## Risks And Dependencies
- **Prerequisite drift:** Slice 01/02 may choose different canonical module names. Reuse their validators, launch, readiness, and health components; do not create parallel implementations merely to match the likely paths above.
- **Prefixed global frames:** Nav2 examples conventionally use unprefixed `map`. Prove AMCL accepts `robot1/map` and that every producer/consumer agrees; stop rather than add an unprefixed bridge.
- **TF remapping:** Upstream Nav2 explicitly remaps absolute `/tf` and `/tf_static`. A naive namespace may isolate TF topics and hide simulation transforms. Verify the project's intended shared TF transport with prefixed frame IDs and preserve one TF graph.
- **Lifecycle/readiness ordering:** Nav2's stock lifecycle manager autostarts immediately. The project launch must not allow activation before the selected robot is ready and the map is validated/locked.
- **Initial-pose detection:** AMCL process activation is not initialization. The health monitor must distinguish a post-operator pose estimate from stale/default output using timestamps/state, with deterministic tests.
- **Map provenance:** Installed Nav2 sample maps are not proof of a TurtleBot3 World match. Integration requires a geometrically matching default map plus declared world metadata.
- **GUI testing:** RViz may be unavailable in headless CI. Structural config tests and `rviz:=false` integration must remain authoritative; GUI behavior still needs one host manual verification.
- **Covariance tuning:** Aligned thresholds are contractual defaults but simulated convergence may expose profile tuning needs. Tune the shared profile with evidence; do not loosen health semantics or special-case robot identities.

## Stop Conditions And Blocked Procedure
Stop implementation and report back when:
- Slice 01 or Slice 02 is incomplete, conflicts with this plan, or lacks the assumed canonical extension points.
- The installed Jazzy Nav2 API cannot support prefixed map/odom/base frames or namespaced initialization without a compatibility path forbidden by alignment.
- A geometrically matching, legally reusable default-world map cannot be identified or created within this slice's agreed scope.
- Correct map locking would require replacing the aligned operating-system shared/read model.
- Tests or the one-robot Gazebo verification cannot run, or required GUI verification cannot be performed on the host.
- The work requires a shared global map frame, automatic initialization, navigation behavior, fleet orchestration, mapping implementation, broad unrelated changes, or a slice substantially larger than planned.

When blocked, preserve the working tree, record the exact command/error and affected acceptance criterion, describe the smallest alignment or prerequisite change needed, and do not add fallbacks, aliases, or partial alternative launch paths.

## Preserve User Work
- Inspect `git status` before editing once the repository exists.
- Preserve unrelated tracked and untracked user changes.
- Do not reset, overwrite, reformat, or commit unrelated paths.
- If prerequisite slice work is concurrently changing a required file, stop and coordinate rather than replacing it.

## Expected Final Summary
The implementing agent must report:
- Files/components added or changed, grouped by map resolution/locking, AMCL lifecycle launch, RViz/operator behavior, health, and tests.
- The selected robot's resolved map ID/path, topic/frame contract, lifecycle ownership, and initialization behavior.
- Verification commands run and their outcomes, including any unavailable GUI or Gazebo checks.
- Implementation-status block updates: commit, verification evidence, deviations, and follow-ups.
- Any remaining risks or deferred `FUP-###` items; do not claim Slice 04 multi-robot concurrency or Slice 05 mapping is complete.
