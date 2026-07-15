# Slice 05 Plan: Exclusive Mapping And Immutable Map Save

## Implementation Status
Status: review-failed
Commit: `bb117d8`
Verification: Both packages build successfully. `fleet_localization` passes 62/62 tests and the workspace reports 87 tests with zero errors, failures, or skips; `git diff --check` passes. Automated coverage proves exclusive/shared process locking, fresh async-SLAM parameters, lifecycle and mapping-health semantics, mapping-only RViz, session-bound save validation, atomic no-replace publication, collision/failure cleanup, catalogue validation, lock release, and later localization activation. Isolated live runs proved readiness, bonded active SLAM, robot-prefixed OccupancyGrid and TF, mapping health, localization lock rejection, explicit immutable save, duplicate rejection, no staging residue, and subsequent active map-server/AMCL loading of the saved map.
Deviations: Live operator verification used `rviz:=false`; the dedicated mapping RViz is structurally tested. Gazebo movement was supplied through the existing simulation interface rather than a Slice 06 teleop tool. No approved behavior deviation.
Follow-ups: None in slice scope. `FUP-001` through `FUP-005` remain deferred.

## Context
- Repository: `/home/syncrobot/localization` (not initialized as a Git repository at planning time)
- Branch: unavailable; implementation should use the feature branch established by Slice 01
- Feature folder: `docs/pipeline/turtlebot-gazebo-host-localization`
- Selected vertical slice: **Exclusive Mapping And Immutable Map Save**
- Sequence: 05 of 06
- Source design: [1_design.md](1_design.md)
- Source alignment: [2_alignment.md](2_alignment.md)
- Roadmap: [3_plan.md](3_plan.md)
- UX decisions: [../../about/05_ux_decision_catalogue.md](../../about/05_ux_decision_catalogue.md)
- Prerequisites: Slice 01's strict manifest/map-store/world model and simulated sensor contract; Slice 02's selected-robot readiness, EKF, health, and odometry TF; Slice 03's canonical map catalogue and session-lock abstraction; Slice 04's proof that independent localization sessions share map data safely.

## Slice Goal
Add an explicit `mapping` mode to one separately launched `fleet_localization` invocation. The invocation must acquire the map store exclusively, pass the selected robot's readiness gate, run its existing EKF with asynchronous fresh-map SLAM Toolbox instead of map server/AMCL, expose observable mapping health, and save only on a separate explicit operator request. A save stages and validates occupancy assets plus world metadata, then publishes one new immutable map-ID directory atomically; it never overwrites or exposes a partial map.

This slice proves one authoritative map-creation session and the transition back to later localization. It does not add teleoperation, an alternate world, or final cross-mode fault hardening assigned to Slice 06.

## Repository Rules
- Consulted: [1_design.md](1_design.md), [2_alignment.md](2_alignment.md), [3_plan.md](3_plan.md), [CURRENT.md](CURRENT.md), neighboring slice plans, and [../../about/05_ux_decision_catalogue.md](../../about/05_ux_decision_catalogue.md). `docs/rules/README.md` and repository rule files do not exist.
- Constraints applied: exactly one selected robot per invocation; one canonical launch and configuration path; exclusive map-store ownership for mapping; asynchronous fresh-map SLAM only; explicit non-overwriting save; prefixed frames; EKF/SLAM TF ownership; no AMCL/map server in mapping mode; no hidden fallback or destructive option.
- Rule updates: None

## Follow-Up Scope
- Included: None. This slice implements only approved current scope.
- Still deferred: `FUP-001` performance guarantees beyond two robots; `FUP-002` real non-TurtleBot sensor profiles; `FUP-003` automatic initial pose; `FUP-004` shared global multi-robot frame; `FUP-005` concurrent or collaborative mapping.
- Do not reinterpret `FUP-005` as permission to merge maps, accept multiple robots' scans, or allow more than one mapping writer.

## Central Issue Scope
- Included register IDs: None.
- Still deferred register IDs: None.
- Newly created register IDs: None; no `docs/issues/README.md` or issue-register schema exists, so the approved feature-local follow-ups remain authoritative.

## Canonical Mental Model
Mapping is a mutually exclusive operating session over the common map store, not a background feature of localization:

```text
mapping launch for selected robot
  acquire exclusive map-store lock
  -> readiness gate
  -> existing planar EKF owns <robot>/odom -> <robot>/base_footprint
  -> async fresh-map SLAM consumes /<robot>/scan
     and owns <robot>/map -> <robot>/odom
  -> health reports mapping/degraded

explicit save(new_map_id)
  -> reserve ID while exclusive lock is held
  -> save into private staging directory
  -> validate map YAML + image + world metadata
  -> atomically rename staging directory to maps/<new_map_id>
```

Localization sessions hold shared/read locks; a mapping session holds the one exclusive/write lock for its complete lifetime. The operating-system lock is authoritative and releases on process death. The final map directory's existence means the complete map is committed; callers must never observe it while files are still being generated.

### Required Cleanup And Removals
- Extend Slice 03's canonical lock implementation; do not add a second lock file, ROS graph heuristic, PID registry, or persistent `mapping_active` boolean.
- Extend the existing one-robot launch with the canonical `mode` branch; do not create a second supported mapping launcher that bypasses manifest validation, readiness, lock ownership, or health.
- Do not start map server, AMCL, AMCL lifecycle management, initial-pose handling, or localization RViz behavior in mapping mode.
- Do not use SLAM Toolbox localization, lifelong, synchronous, continuation, serialized-pose-graph, merge-map, or multi-robot modes.
- Do not save automatically on shutdown, accept an output filesystem path, mutate `fleet.map_id`, overwrite an existing ID, expose a force option, or fall back outside the manifest-relative `map_store`.
- Do not publish an unprefixed `map` frame or allow SLAM Toolbox and another component to publish the same map-to-odom edge.
- Do not vendor SLAM Toolbox or Nav2 map-server source.

## Reconnaissance Findings And Existing Patterns
- The workspace contains planning artifacts only. Slice 01-04 must establish the actual ROS workspace and canonical package modules before implementation; reuse those names and extension points rather than duplicating likely paths below.
- Installed Jazzy packages are `slam_toolbox` Debian `2.8.5` and `nav2_map_server` Debian `1.3.12` at planning time. Depend on ROS package names, not these observed patch versions.
- `/opt/ros/jazzy/share/slam_toolbox/launch/online_async_launch.py` starts lifecycle executable `async_slam_toolbox_node`, supports `use_sim_time`, and configures/activates the node. The project must preserve readiness-before-activation and its selected-robot namespace rather than include that launch unchanged.
- `/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml` is the upstream fresh asynchronous mapping baseline. Relevant Jazzy parameters include `mode: mapping`, `odom_frame`, `map_frame`, `base_frame`, `scan_topic`, `transform_publish_period`, `map_update_interval`, and `use_map_saver`. Project parameters must inject prefixed frames and the namespaced scan topic.
- Jazzy exposes `slam_toolbox/srv/SaveMap` with a string filename and result codes. It writes occupancy assets but does not encode the project map-ID schema, active-world metadata, directory immutability, or atomic commit contract. Direct operator calls to the raw service are therefore not the supported persistence interface.
- `nav2_map_server` includes map I/O and saver components, but using a second saver path would create duplicate semantics. The project should wrap exactly one proven upstream save mechanism behind its own validated transaction boundary.
- The aligned OS-lock requirement is best met by advisory file locking on a stable lock inode outside map-ID directories. Renaming or deleting the lock inode while sessions run would split the lock domain and must be prohibited.

## Likely Files, Modules, Components, And Tests
Paths assume the package shape established by earlier slices. Reconcile names with implemented canonical modules rather than create parallel variants.

### Package, Launch, And Configuration
- `src/fleet_localization/package.xml` — add runtime/test dependencies actually used for `slam_toolbox`, its service interfaces, lifecycle integration, map I/O validation, and launch testing.
- `src/fleet_localization/setup.py` and/or `CMakeLists.txt` — install mapping parameters and the explicit save executable/module.
- `src/fleet_localization/launch/localization.launch.py` — implement the already aligned `mode:=localization|mapping` selection; in mapping mode resolve one robot, acquire the exclusive lock before readiness/node creation, retain the Slice 02 EKF, and activate only asynchronous SLAM Toolbox plus health after readiness.
- `src/fleet_localization/config/burger_slam_async.yaml` — one reusable fresh-map asynchronous Burger profile; inject `<robot>/map`, `<robot>/odom`, `<robot>/base_footprint`, `/<robot>/scan`, and `use_sim_time` at launch.
- `src/fleet_localization/fleet_localization/map_store_lock.py` — extend the Slice 03/04 API with non-blocking exclusive acquisition, deterministic same-lock-inode behavior, ownership for the process lifetime, actionable holder-conflict errors where available, and cleanup on normal failure/signal/process exit.
- `src/fleet_localization/fleet_localization/localization_health.py` — extend the one canonical state machine with `mapping`, SLAM lifecycle/map/TF freshness, and degraded/recovery semantics; do not add a separate health publisher.

### Explicit Persistence Boundary
- `src/fleet_localization/fleet_localization/map_save.py` — operator-facing ROS executable or service client taking required `fleet_config`, selected `robot`, and required new `map_id` (not an arbitrary output path); prove it is attached to the live mapping session and same resolved store; validate safe/nonexistent ID; invoke the selected robot's namespaced upstream save endpoint into a private staging directory; validate outputs; write world-association metadata; atomically commit the complete directory; report the committed ID and files.
- `src/fleet_localization/fleet_localization/map_catalog.py` — reuse Slice 03 map-ID, path-containment, occupancy YAML/image, and world-metadata validators for preflight and post-save validation; add no alternate catalogue.
- `src/fleet_localization/fleet_localization/map_transaction.py` (only if separation from `map_save.py` is justified by tests) — same-filesystem staging under `map_store`, exclusive creation, cleanup of failed staging, durable metadata generation, and atomic directory rename with no replacement. Treat unexpected pre-existing final IDs as conflicts even after preflight to close the race.
- World metadata should follow the schema established in Slice 01/03 and identify the active world used for mapping. Do not infer or accept a caller-supplied conflicting world ID.

### Tests
- `src/fleet_localization/test/test_map_store_lock.py` — multiple shared readers; exclusive writer rejected while any reader lives; reader and second writer rejected while writer lives; acquisition succeeds after normal exit, exception, signal, and subprocess death; every path targets the stable canonical lock inode.
- `src/fleet_localization/test/test_mapping_launch.py` — valid selected robot; default/explicit mapping mode resolution; exclusive lock acquired before readiness and SLAM lifecycle activation; exact async executable/parameters; EKF retained; map server, AMCL, initial-pose path, and localization lifecycle nodes absent; lock conflict fails only this invocation before SLAM starts.
- `src/fleet_localization/test/test_slam_parameters.py` — `mode: mapping`, asynchronous executable, `use_sim_time`, namespaced scan, prefixed map/odom/base frames, SLAM map-to-odom broadcasting, fresh-map-only settings, and absence of continuation/lifelong/merge parameters.
- `src/fleet_localization/test/test_map_save.py` — required safe new ID; no arbitrary destination; existing ID rejected before upstream save; upstream failure/no-map result leaves no committed ID; malformed/missing YAML or image rejected; world metadata is correct; path/symlink escape rejected; staging cleaned after failure; successful directory commit is atomic; collision at final rename never overwrites; no automatic shutdown save.
- `src/fleet_localization/test/test_mapping_health.py` — mapping only after readiness and active SLAM with current map/TF; stale scan, EKF, map, lifecycle failure, invalid timestamps, or broken TF becomes degraded after 2 seconds; recovery requires 2 healthy seconds; one invocation's health remains robot-scoped.
- `src/fleet_localization/test/test_mapping_integration.py` — one simulated robot maps after teleoperated movement supplied externally, publishes only its prefixed map TF/topic, explicitly commits a new map ID, exits and releases the exclusive lock, then the saved map passes Slice 03 catalogue validation and can be loaded by a later localization invocation.

## Existing Patterns To Follow And Behavior That Must Not Regress
- Reuse Slice 01's strict manifest validation, resolved active world, safe map IDs, manifest-relative canonical map store, selected robot identity, sensor QoS, prefixed body/sensor TF, and simulator exclusion from odometry TF.
- Reuse Slice 02's 60-second configurable readiness timeout, planar EKF policy, exclusive odometry-TF ownership, simulation time, finite/timestamp validation, and isolated failure behavior.
- Reuse Slice 03/04's one launch entry point, map catalogue, stable lock object/path, selected-robot health interface, independent namespaces/frames, and immutable existing maps.
- Keep multiple localization read locks working when no mapping session exists. Adding exclusive acquisition must not serialize normal localization sessions.
- Simulation remains running if mapping readiness, lock acquisition, lifecycle activation, or save fails.
- SLAM Toolbox is the sole `<robot>/map -> <robot>/odom` owner in mapping mode; EKF remains the sole `<robot>/odom -> <robot>/base_footprint` owner.
- The current fleet `map_id` and every previously committed directory remain byte-for-byte unchanged by mapping or save failures.

## Acceptance Criteria
1. `mode:=mapping` selects exactly one manifest robot, resolves the one canonical map store, and attempts a non-blocking exclusive session lock before readiness or SLAM node creation.
2. Mapping fails clearly and starts no SLAM node while any localization read lock or another mapping write lock exists. While mapping holds the lock, new localization and mapping invocations fail; after mapping exits or crashes, acquisition succeeds without manual stale-flag cleanup.
3. After selected-robot readiness succeeds, the existing EKF and namespaced asynchronous fresh-map SLAM Toolbox lifecycle node activate automatically; no map server, AMCL, initial-pose behavior, or AMCL lifecycle node runs.
4. SLAM consumes only `/<robot>/scan`, uses `<robot>/map`, `<robot>/odom`, and `<robot>/base_footprint`, uses simulation time, and exclusively publishes `<robot>/map -> <robot>/odom`. It does not load, continue, localize against, merge, or serialize an existing pose graph.
5. Health reports `mapping` only with fresh selected-robot inputs, finite/current EKF output, active SLAM, current map output, and a resolvable prefixed map-to-sensor chain. Existing 2-second degradation and recovery persistence remains in force.
6. Stopping mapping without an explicit save request creates no final map-ID directory and modifies no existing map.
7. The supported save action requires a new safe `map_id`, derives the destination only from the invocation's manifest-relative map store, and rejects an existing ID. No overwrite, force, raw output path, hidden fallback, or automatic save interface exists.
8. A successful save generates a valid occupancy YAML and referenced image, adds active-world metadata, validates all assets through the canonical catalogue, and atomically exposes exactly one complete `maps/<new_map_id>/` directory.
9. Upstream save failure, no map received, invalid output, metadata failure, interruption, or a late ID collision leaves no committed final directory, removes private staging where safely possible, and never changes an existing ID.
10. After explicit save and mapping shutdown, the exclusive lock releases and a normal localization invocation can acquire a shared lock and load the newly committed ID without moving or editing its files.
11. Tests prove robot/frame/topic isolation, correct TF ownership, no direct supported bypass around the transactional save boundary, and no regression to concurrent localization readers.

## Ordered Implementation Tasks
1. Reconcile this plan with completed Slices 01-04. Stop if there is no single strict manifest/map resolver, stable shared-lock abstraction, selected-robot launch/readiness/EKF path, or extensible health state machine; repair prerequisite slice scope rather than build parallel components here.
2. Extend the existing lock API and tests with process-lifetime exclusive acquisition on the same stable canonical lock inode used by localization readers. Make conflicts non-blocking/actionable and verify release under exceptions, signals, and subprocess death.
3. Add the reusable Burger asynchronous fresh-mapping profile using installed Jazzy parameter names. Inject selected-robot frames/topics and prohibit continuation, localization, lifelong, serialization, and merge settings.
4. Add the `mapping` branch to the canonical one-robot launch: validate configuration, acquire exclusive lock, pass readiness, retain EKF, lifecycle-activate namespaced `async_slam_toolbox_node`, and extend health. Ensure every failure tears down this invocation and releases the lock without stopping Gazebo.
5. Define one explicit project-owned save command/interface tied to the active mapping session. Require `fleet_config`, selected robot, and a safe new ID; do not accept a destination path or expose the raw upstream filename service as the documented workflow.
6. Implement the save transaction: recheck ID absence; create a private same-filesystem staging directory; request upstream occupancy save there; verify success result; validate YAML/image containment and content; create active-world metadata; run canonical catalogue validation; atomically rename without replacement; clean failed staging.
7. Extend mapping health tests and reporting for readiness, active lifecycle, fresh map/TF output, degraded inputs, and persistence windows.
8. Add unit, subprocess-lock, launch, parameter, failure-injection, and save-transaction tests. Include late collision, process death, upstream partial output, and no-map cases.
9. Run a one-robot mapping integration using simulation and external teleoperation, explicitly commit a new map ID, stop mapping, and prove a subsequent normal localization launch loads that immutable result.
10. Update package-facing usage documentation only for the one-robot mapping launch, lock conflict, explicit save, health states, and return-to-localization workflow. Do not claim Slice 06 teleoperation or alternate-world hardening as complete.

## Verification Commands And Expected Outcomes
Run from `/home/syncrobot/localization` after sourcing ROS 2 Jazzy. Adjust only the workspace root if Slice 01 established another canonical path; never bypass a failing prerequisite.

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Expected: declared Jazzy dependencies, including SLAM Toolbox and its interfaces, resolve without vendored upstream source.

```bash
colcon build --symlink-install --packages-up-to fleet_localization
colcon test --packages-select fleet_localization --event-handlers console_direct+
colcon test-result --verbose
```

Expected: mapping launch, exclusive/shared locking, SLAM parameters, mapping health, transactional save, and integration tests pass with zero failures.

With a Slice 01 simulation running and no localization invocation holding a read lock:

```bash
ros2 launch fleet_localization localization.launch.py fleet_config:=<absolute-fleet-yaml> robot:=robot1 mode:=mapping rviz:=true timeout:=60
```

Expected: configuration and exclusive locking precede readiness; robot1 EKF and namespaced asynchronous SLAM activate; health reaches `mapping` when current map/TF data exists; no robot1 map server or AMCL starts.

```bash
ros2 lifecycle get /robot1/slam_toolbox
ros2 param get /robot1/slam_toolbox mode
ros2 param get /robot1/slam_toolbox map_frame
ros2 param get /robot1/slam_toolbox odom_frame
ros2 param get /robot1/slam_toolbox base_frame
ros2 param get /robot1/slam_toolbox scan_topic
ros2 run tf2_ros tf2_echo robot1/map robot1/odom
ros2 node list
```

Expected: SLAM is active in mapping mode with `robot1/map`, `robot1/odom`, `robot1/base_footprint`, and `/robot1/scan`; its map-to-odom transform resolves; node list contains no map server or AMCL for robot1.

In another sourced terminal while mapping is running:

```bash
ros2 launch fleet_localization localization.launch.py fleet_config:=<absolute-fleet-yaml> robot:=robot2 mode:=localization rviz:=false
```

Expected: the second invocation fails before estimator/localization activation with an explicit exclusive map-store lock conflict. Stopping the mapping process releases the lock, after which this command may acquire a shared lock normally.

After moving robot1 enough to produce a useful occupancy map, invoke the project save executable selected during implementation:

```bash
ros2 run fleet_localization map_save --fleet-config <absolute-fleet-yaml> --robot robot1 --map-id turtlebot3_world_mapped_v2
```

Expected: it reports successful commit only after complete occupancy assets and matching active-world metadata are validated at `<resolved-map-store>/turtlebot3_world_mapped_v2/`; the live mapping session continues until explicitly stopped.

```bash
ros2 run fleet_localization map_save --fleet-config <absolute-fleet-yaml> --robot robot1 --map-id turtlebot3_world_mapped_v2
```

Expected: the repeated save fails before upstream persistence because the ID exists; the committed files remain unchanged.

After stopping mapping:

```bash
ros2 launch fleet_localization localization.launch.py fleet_config:=<absolute-fleet-yaml> robot:=robot1 mode:=localization map_id:=turtlebot3_world_mapped_v2 rviz:=true
```

Expected: shared locking succeeds, Slice 03 catalogue validation accepts the committed files/metadata, map server and AMCL activate after readiness, and manual initialization can localize robot1 against the new immutable map.

## Risks And Dependencies
- **Prerequisite API drift:** actual Slice 01-04 paths may differ. Extend their canonical validators, locks, launch, and health code; do not create duplicate implementations to match this plan's likely names.
- **Advisory-lock discipline:** OS locks protect only participants using the canonical API. All project read/save paths must use it, and documentation must not present raw upstream save services as supported. Manual filesystem edits remain explicitly outside runtime guarantees.
- **Stable lock inode:** replacing or placing the lock inside an atomically renamed map directory can split ownership. Keep one stable store-level inode and stop if the established lock model cannot guarantee that.
- **Atomic no-replace commit:** generic rename may replace an empty destination on some systems. Use an implementation that proves no-replace behavior or perform a defensible exclusive reservation under the already-held write lock; stop if the target filesystem cannot provide the required same-filesystem atomic directory commit.
- **Upstream save behavior:** `SaveMap` accepts a filename prefix and may create partial files before returning failure. Always target private staging, inspect actual Jazzy output naming, and never point it at the final ID directory.
- **Map-frame metadata:** the upstream occupancy YAML may record the SLAM frame only in message data, not project world identity. Generate metadata from the validated active simulation world, not untrusted save arguments.
- **Lifecycle and TF ownership:** SLAM Toolbox must activate only after readiness and be the sole map-to-odom broadcaster. Stop rather than add a bridge or tolerate duplicate TF.
- **Useful-map proof:** successful file output does not prove geometric quality. Integration must include scan/map visual evidence sufficient for the demo, while general map-quality scoring remains outside scope.
- **Simulation pause/time jumps:** paused `/clock` can make health appear stale. Preserve aligned simulation-time semantics and distinguish an intentionally paused simulator only if earlier slices already define that behavior; do not silently switch to wall time.

## Stop Conditions And Blocked Procedure
Stop implementation and report before broadening scope when:
- Slices 01-04 do not supply the canonical manifest/map resolver, shared lock, selected-robot EKF/readiness launch, or health extension points assumed here.
- Installed Jazzy SLAM Toolbox cannot accept the required prefixed frames/namespaced scan or cannot exclusively own the selected map-to-odom transform.
- The canonical map store resides on a filesystem where tested advisory locking or atomic same-filesystem no-replace publication cannot satisfy the approved safety contract.
- The installed upstream save interface cannot be safely confined to staging or does not produce occupancy assets compatible with the canonical catalogue.
- Verification cannot prove lock release after crash, immutable collision behavior, lifecycle ordering, or later map loading.
- Work would require collaborative mapping, pose-graph continuation, a shared global frame, custom upstream forks, teleoperation implementation, or broad unrelated changes.

When blocked, preserve diagnostics and partial test evidence, leave the implementation status `blocked` only under the workflow's status rules, record the exact unmet contract and affected files, and return to planning/alignment rather than adding a fallback or weakening map safety.

## Preserve User Work
- Inspect the worktree before editing and preserve all unrelated user changes.
- Reconcile overlapping changes in shared launch, lock, catalogue, health, manifest, package, and test files rather than overwriting them.
- Do not delete or rewrite existing map directories during tests. Use isolated temporary stores and unique disposable IDs.
- Do not use destructive Git or filesystem commands to recover from failures.

## Expected Final Summary
The implementing agent's final report must include:
- Files and components changed for mapping mode, exclusive locking, health, and transactional save.
- The selected Jazzy asynchronous SLAM executable/service and the resolved robot-specific frames/topics.
- Evidence that localization/read locks exclude mapping, mapping excludes new readers/writers, and crash/exit releases the lock.
- The explicit test map ID and resulting validated files/metadata, plus evidence that repeated save did not overwrite it.
- Build, unit, launch, failure-injection, and end-to-end verification commands with results.
- Confirmation that AMCL/map server were absent in mapping mode, TF ownership remained exclusive, and existing maps were unchanged.
- Deviations, unresolved risks, deferred `FUP-001` through `FUP-005`, and any blocked verification.
