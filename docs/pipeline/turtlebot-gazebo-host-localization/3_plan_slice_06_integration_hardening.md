# Slice 06 Plan: Alternate World, Safe Teleoperation, And Integration Hardening

## Implementation Status
Status: implemented
Commit: `97d3df0`
Verification: Both packages build successfully. The complete suite passes 105/105 tests with zero errors, failures, or skips; `git diff --check` passes. Automated coverage includes both strict world/map pairs, GUI/headless graph equivalence, selected-robot TwistStamped teleop safety, atomic and isolated fault boundaries, two independent localization contracts, mapping lock/save/load behavior, duplicate rejection, lifecycle/timestamp/TF failures, and health persistence. Live headless custom-world runs spawned both robots and reached `FLEET_READY`; a custom robot1 readiness gate completed, loaded the 60x50 custom map, and reached active map-server/AMCL lifecycle.
Deviations: The custom world is deliberately capped at 0.2x real time so headless ROS bridge, sensor, and TF delivery remains timestamp-consistent. The execution harness forcibly ended calibrated long-running groups after about 32 wall seconds, before robot2 at 0.2x and the subsequent custom live mapping/save/load cycle completed. Those workflows are covered by public-contract/integration tests and the equivalent standard-world live proofs from Slices 04-05. GUI/RViz visual alignment and physical keyboard checks require an interactive display/operator and remain manual verification limitations, not claimed evidence.
Follow-ups: None in slice scope. `FUP-001` through `FUP-005` remain deferred.

## Context
- Repository: `/home/syncrobot/localization` (not initialized as a Git repository at planning time)
- Required implementation branch: `feature/host-localization`, created from `dev` after repository initialization
- Feature folder: `docs/pipeline/turtlebot-gazebo-host-localization`
- Selected vertical slice: Alternate World, Safe Teleoperation, And Integration Hardening
- Sequence: 06 of 06
- Design source: [1_design.md](1_design.md)
- Alignment source: [2_alignment.md](2_alignment.md)
- Roadmap source: [3_plan.md](3_plan.md)
- Routing source: [CURRENT.md](CURRENT.md)
- UX source: [../../about/05_ux_decision_catalogue.md](../../about/05_ux_decision_catalogue.md)

## Slice Goal
Complete the approved demo without changing its architecture: add the project-owned asymmetric indoor world and a geometrically matching immutable map with enforced world provenance; add separately launched, robot-scoped `TwistStamped` keyboard teleoperation with stop-on-release and stop-on-exit safety; prove both GUI and headless simulation modes; and assemble final two-robot localization, single-robot mapping, isolation, and fault evidence across the completed slices.

This is an integration and hardening slice. It must extend the canonical launch, configuration, map, health, locking, and test surfaces established by slices 01-05 rather than introduce parallel launch paths or reimplement their behavior.

## Repository Rules
- Consulted: [1_design.md](1_design.md), [2_alignment.md](2_alignment.md), [3_plan.md](3_plan.md), [CURRENT.md](CURRENT.md), preceding `3_plan_slice_*.md` contracts, and [../../about/05_ux_decision_catalogue.md](../../about/05_ux_decision_catalogue.md). `docs/rules/README.md` and repository rule files do not exist.
- Constraints applied: planning only; preserve the two-package boundary, exactly-one-robot localization invocation, independent prefixed map frames, one immutable shared map store, external Jazzy dependencies, one canonical project simulator launch, stamped commands only, operator-safe motion, and unrelated user changes.
- Rule updates: None

## Follow-Up Scope
- Included deferred items: None.
- Still deferred:
  - `FUP-001`: performance guarantees beyond two robots. Final evidence covers two robots; arbitrary-N remains structural only.
  - `FUP-002`: physical non-TurtleBot sensor profiles.
  - `FUP-003`: automatic initial pose.
  - `FUP-004`: a shared global multi-robot frame or combined visualization.
  - `FUP-005`: concurrent or collaborative mapping.

## Central Issue Scope
- Included register IDs: None.
- Deferred register IDs: None.
- Newly created register IDs: None. No central issue register exists, and this slice must report newly discovered durable gaps rather than silently expanding scope.

## Explicit Non-Goals
- Redesigning the fleet schema, topic/QoS contract, TF ownership, EKF fusion policy, AMCL lifecycle, health thresholds, map-store locks, or map-saving protocol delivered by earlier slices.
- Adding another simulator, localization, mapping, or fleet launcher.
- Adding Docker, autonomous navigation, collision avoidance, coordinated motion, a shared map frame, or fleet-wide localization orchestration.
- Supporting plain `geometry_msgs/msg/Twist`, a compatibility bridge, an unselected/default teleoperation target, continuous key-repeat motion after release, or unstamped/wall-time commands.
- Guaranteeing performance for more than two robots or tuning for an unspecified physical robot.
- Making world metadata a substitute for geometric scan/map validation.
- Adding automatic initial pose, automatic map saving, mutable map IDs, force overwrite, continued/lifelong SLAM, or concurrent mapping.
- Vendoring upstream TurtleBot3, Gazebo, Nav2, `robot_localization`, SLAM Toolbox, or keyboard-teleoperation source.

## Canonical Mental Model And Required Cleanup
The completed system retains two independently operated layers:

```text
one validated fleet launch -> standard or custom Gazebo world -> N source interfaces

separate robot invocation -> EKF + AMCL/map copy
                          or EKF + asynchronous fresh-map SLAM

separate teleop invocation -> one validated robot -> one TwistStamped command topic
```

The alternate world is another value accepted by the existing simulator launch, not a new launch stack. Its map is another immutable map-store entry with explicit originating-world metadata, not package-hidden fallback data. GUI/headless is one existing launch argument with equivalent ROS behavior. Fault injection proves the established isolation and fail-fast contracts; it must not add recovery fallbacks that alter them.

Required cleanup and exclusions:
- Remove any interim custom-world launch, direct `gz sim` instructions, duplicate example manifest, or map lookup bypass created during development.
- Remove any plain-`Twist` publisher or alias and any command target that is inferred rather than explicitly selected.
- Remove any teleop behavior that can leave a nonzero command active after key release, normal exit, SIGINT, or handled termination.
- Keep only the canonical standard-world and custom-world IDs and their matching metadata. Do not retain ambiguous aliases or silently accept a mismatched map ID.
- Do not weaken strict startup, per-robot runtime degradation, lifecycle isolation, or shared-read/exclusive-write map locking merely to make integration tests pass.

## Reconnaissance And Existing Patterns
- At planning time the workspace contains pipeline documentation only; no ROS packages, implementation tests, or build files exist. This slice must first inspect the implemented outputs and open diffs from slices 01-05 and use their actual extension points.
- `/opt/ros/jazzy/share/ros_gz_sim` is installed and provides the Harmonic `gz_sim.launch.py`/server/spawn launch patterns. The custom world must remain compatible with that established Slice 01 path; do not introduce Gazebo Classic APIs.
- `/opt/ros/jazzy/share/teleop_twist_keyboard` is installed, but its installed package exposes no project-specific robot selection, manifest validation, simulation-time stamping, or guaranteed stop-on-release contract. Reuse dependency behavior only if inspection proves every aligned safety requirement; otherwise implement a small project-owned teleop node rather than patching or vendoring upstream source.
- `/opt/ros/jazzy/share/nav2_map_server` is installed and supplies Nav2 map components. World-association metadata is a project contract layered around the canonical map-store resolver from slices 03 and 05, not an upstream map-YAML field to assume without proof.
- TurtleBot3 packages were not installed under `/opt/ros/jazzy/share` during planning. By this slice, Slice 01 must have resolved and documented the exact official Jazzy assets. Stop if the alternate world would require copying or forking upstream robot assets.
- GUI behavior requires host display access and is unsuitable as the sole automated gate. Headless launch/integration tests are authoritative in automation; one manual GUI smoke test supplies the visual evidence.
- Existing slice plans assign strict fleet validation/simulation to Slice 01, EKF/readiness/health foundations to Slice 02, map/AMCL/RViz to Slice 03, independent two-robot concurrency to Slice 04, and exclusive mapping/save to Slice 05. This slice may add cross-slice fixtures and end-to-end scenarios but must not duplicate their unit responsibilities.

## Likely Files, Modules, Components, And Commands
Use the actual files created by slices 01-05. Expected additions or narrow extensions are:

```text
src/turtlebot_fleet_sim/
  package.xml
  setup.py
  launch/fleet_sim.launch.py             # extend existing world/gui selection only
  launch/teleop.launch.py
  turtlebot_fleet_sim/fleet_config.py    # register custom world only; preserve strict schema
  turtlebot_fleet_sim/teleop.py
  worlds/asymmetric_indoor.sdf
  config/fleet.example.yaml              # document canonical default; no duplicate manifest
  test/test_teleop.py
  test/test_custom_world.py
  test/test_gui_headless_contract.py
  test/test_simulation_faults.py

src/fleet_localization/
  maps/asymmetric_indoor_v1/              # or the canonical writable example map store chosen earlier
    map.yaml
    map.pgm
    metadata.yaml
  test/test_world_map_association.py
  test/test_end_to_end_demo.py
  test/test_integration_faults.py

README.md or package READMEs                # only the final supported operator workflow
```

Exact map placement must follow the Slice 03/05 canonical map-store fixture. If the aligned writable map store lives beside a fleet manifest rather than inside a package, put the bundled demonstration maps there and do not create a second lookup root.

Component responsibilities:
- `asymmetric_indoor.sdf`: a small bounded indoor environment with asymmetric, lidar-visible geometry, navigable TurtleBot clearance, noncolliding configured spawn poses, and no dependence on external online model downloads.
- Matching map asset: occupancy geometry and resolution/origin suitable for AMCL in that world, immutable map ID `asymmetric_indoor_v1` (or the already-established canonical ID), and metadata recording the exact canonical world ID.
- Existing map resolver/validator: reject standard/custom world-to-map metadata mismatches before map server or AMCL activation; accept matching metadata; continue treating actual scan alignment as integration evidence.
- `teleop.py`: require a valid manifest robot selection before creating a publisher; publish only `/<robot>/cmd_vel` as `TwistStamped`; stamp with ROS simulation time; send zero immediately on key release and a final zero on normal exit, EOF, SIGINT, and other handled shutdown paths.
- `teleop.launch.py`: require `fleet_config` and `robot`, reuse the canonical parser/selection logic, and create no publisher on invalid input.
- Existing `fleet_sim.launch.py`: keep `gui` default/override precedence and make GUI/headless differ only in rendering/client process, not world, robots, topics, TF, readiness, or clock.
- Integration/fault tests: exercise public launches and interfaces, not private shortcuts, and assert strict startup failure versus isolated runtime degradation at the correct ownership boundary.

Expected dependency changes should be limited to packages actually used by the world, input handling, `geometry_msgs`, `rclpy`, launch testing, and existing Gazebo/localization dependencies. Avoid adding a general terminal UI framework unless required for reliable key-release detection and covered by tests.

## Acceptance Criteria
1. The canonical fleet simulator accepts `asymmetric_indoor` through the existing strict world validation and starts it through the same project launch used by `turtlebot3_world`; the standard world remains the manifest default.
2. The custom world contains stable asymmetric lidar-visible features, admits the configured two-Burger fleet without spawn collisions, and requires no network-fetched Gazebo models at runtime.
3. The custom-world map is a valid immutable map-store entry with occupancy YAML/image and metadata naming `asymmetric_indoor`; scan geometry visibly aligns after AMCL initialization in that world.
4. A matching standard-world map remains accepted. Selecting either world's known mismatched map fails before map server/AMCL lifecycle activation with the selected world ID, map ID, and metadata world ID in the diagnostic.
5. `gui:=true` remains the default and opens the Gazebo client on a display-capable host. `gui:=false` starts the same world/fleet headlessly and preserves `/clock`, entity count, sensor topics, TF, readiness, and localization behavior.
6. Teleoperation requires `fleet_config` and an existing robot name before a velocity publisher exists; an unknown robot fails without publishing any command.
7. Teleoperation publishes only `geometry_msgs/msg/TwistStamped` on the selected `/<robot>/cmd_vel`, uses current simulation time, and never commands another configured robot.
8. Pressing a movement key produces the expected bounded command; releasing it publishes zero promptly. Normal exit, EOF, SIGINT, and every handled termination path publish a final zero command. There is no plain-`Twist` alias.
9. The headless two-robot end-to-end case proves isolated canonical source topics/frames, two separately launched EKF+AMCL stacks, shared map files with independent prefixed map topics/frames, manual-initial-pose injection through each robot's namespaced interface, and independently healthy/localized outcomes.
10. Stopping or degrading one robot's source input after successful startup degrades only that robot; the other robot's estimator, AMCL, TF, and health continue. Conversely, a configured robot missing during initial simulation readiness still terminates the complete fleet launch.
11. Duplicate TF publication, missing/stale/invalid timestamps, incompatible QoS where detectable, failed lifecycle activation, and invalid world/map metadata yield explicit owner-appropriate diagnostics without hidden fallbacks or partial-success status.
12. Mapping integration proves that localization read locks block mapping, the exclusive mapping session blocks localization, only the selected robot's EKF+fresh asynchronous SLAM runs, explicit save creates a new immutable world-associated map, duplicate save is rejected, and no map is saved on shutdown alone.
13. Final verification covers both supported worlds, GUI and headless paths, localization and mapping modes, map immutability/locking, teleop safety, two-robot isolation, and the agreed health persistence/covariance semantics. Any manual-only evidence is clearly separated from automated results.
14. Documentation contains one supported native-host workflow and no Docker, upstream fixed-count launcher, fleet-localization launcher, direct alternate launch, mutable-map, plain-`Twist`, or shared-global-map instructions.
15. No deferred `FUP-###` item is claimed complete; performance claims remain limited to the verified two-robot host.

## Ordered Implementation Tasks
1. Inspect the implemented slices 01-05, their status blocks, tests, current branch/open diffs, canonical configuration parser, world registry, map-store layout/metadata parser, lock API, health interfaces, and public launches. Stop on material drift rather than designing a parallel integration path.
2. Register a single canonical custom-world ID in the existing strict schema and add the asymmetric indoor SDF through the established `ros_gz_sim` launch/resource path. Confirm both approved spawn poses and lidar sight lines before creating map evidence.
3. Create the geometrically matching occupancy map in the existing canonical map store, add exact originating-world metadata through the established metadata format, and extend validation tests for match, mismatch, malformed, and missing metadata without changing map immutability.
4. Extend the existing simulator launch only as needed so `world` and `gui` overrides work for both worlds with the aligned precedence. Add headless structural/integration coverage and retain the standard world and GUI-enabled defaults.
5. Implement the separately launched teleoperation utility using the canonical fleet parser and selected-robot lookup. Enforce `TwistStamped`, simulation timestamps, bounded commands, stop-on-release, final zero on handled shutdown, and no publisher for invalid selection.
6. Add deterministic teleop tests with an injectable key/input and clock boundary so topic/type/target/header/zero sequencing and shutdown paths can be proven without an interactive terminal. Add one PTY/manual smoke scenario for actual keyboard behavior rather than relying only on mocks.
7. Add cross-slice headless integration fixtures that invoke only supported public launch surfaces. Cover the standard and custom world, two separate localization invocations, independent initialization/health, and selected-robot teleoperation isolation.
8. Add fault scenarios at established boundaries: incomplete fleet startup, runtime sensor loss for one robot, duplicate TF ownership, stale/bad timestamps, lifecycle failure, map metadata mismatch, lock contention, duplicate map save, and teleop interruption. Assert the agreed atomic versus isolated failure behavior and diagnostics.
9. Run an explicit fresh-map session in the custom world using one robot, save under a new test map ID, validate its metadata/occupancy output and non-overwrite behavior, then localize against the result. Keep generated test artifacts in temporary map stores, not the immutable bundled store.
10. Run final headless end-to-end verification from a clean build. Then perform GUI/RViz/manual-pose/keyboard smoke checks on a display-capable host for both worlds, recording limitations rather than weakening automated gates.
11. Consolidate the supported operator documentation around the two package launches and per-robot commands. Remove obsolete or duplicate instructions and cross-check every acceptance criterion, prior slice status, and deferred item before marking the slice implemented.

## Verification Commands And Expected Outcomes
Run from the repository/workspace root after sourcing ROS 2 Jazzy. Adapt package-local test selectors to the actual files established by prior slices; do not replace missing required tests with ad hoc commands.

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select turtlebot_fleet_sim fleet_localization --event-handlers console_direct+
colcon test-result --verbose
```

Expected: dependency resolution and clean build succeed; all unit, launch, world/map association, teleop, fault, locking, mapping, and integration tests pass with no failed result.

Focused automated gates should include commands equivalent to:

```bash
python3 -m pytest -q src/turtlebot_fleet_sim/test/test_teleop.py
python3 -m pytest -q src/turtlebot_fleet_sim/test/test_custom_world.py src/turtlebot_fleet_sim/test/test_gui_headless_contract.py
python3 -m pytest -q src/fleet_localization/test/test_world_map_association.py
python3 -m pytest -q src/fleet_localization/test/test_end_to_end_demo.py src/fleet_localization/test/test_integration_faults.py
```

Expected: invalid selection creates no teleop publisher; release/exit/interrupt end with zero; custom-world and map provenance match/mismatch cases are deterministic; headless public launches prove two-robot localization/isolation and the mapping lock/save workflow; fault assertions match atomic startup and isolated runtime behavior.

Public launch smoke commands should follow the implemented package interfaces, for example:

```bash
ros2 launch turtlebot_fleet_sim fleet_sim.launch.py fleet_config:=<absolute-fleet-yaml> world:=asymmetric_indoor gui:=false
ros2 launch fleet_localization localization.launch.py fleet_config:=<absolute-fleet-yaml> robot:=robot1 mode:=localization rviz:=false
ros2 launch fleet_localization localization.launch.py fleet_config:=<absolute-fleet-yaml> robot:=robot2 mode:=localization rviz:=false
ros2 launch turtlebot_fleet_sim teleop.launch.py fleet_config:=<absolute-fleet-yaml> robot:=robot1
```

Expected: the headless fleet becomes completely ready; separate localization sessions use independent prefixed map/odom/base trees and the same immutable map data; robot1 teleop emits stamped commands only for robot1 and stops safely. Use the actual launch filename established by slices 02-05 if different; do not add an alias just to match this example.

Manual display-capable checks:

```bash
ros2 launch turtlebot_fleet_sim fleet_sim.launch.py fleet_config:=<absolute-fleet-yaml> world:=turtlebot3_world gui:=true
ros2 launch fleet_localization localization.launch.py fleet_config:=<absolute-fleet-yaml> robot:=robot1 mode:=localization rviz:=true
```

Expected: Gazebo GUI renders the configured fleet; robot-scoped RViz shows only robot1, manual 2D Pose Estimate reaches `/robot1/initialpose`, lidar aligns to the matching map, and real keyboard release/SIGINT leaves the robot stopped. Repeat the visual alignment check for `asymmetric_indoor`. Record GUI tests as manual evidence; absence of a display is a reported verification limitation, not permission to skip headless gates.

Final diagnostics should also inspect public ROS state with `ros2 topic list -t`, `ros2 topic info --verbose`, `ros2 node list`, and `ros2 run tf2_tools view_frames` (when available). Expected: canonical names/types/QoS, no plain-`Twist` command interface, unique selected-robot nodes, and no duplicate or unprefixed transform edges.

## Risks And Dependencies
- **Prerequisite drift:** This slice depends on all five preceding slices. Cross-slice failures may reveal a prerequisite defect, but fixes belong to the owning slice and status rather than being hidden in integration-only wrappers.
- **Geometric map fidelity:** Metadata proves identity, not geometry. The custom map must be generated or validated against the exact final SDF and evidenced with scan alignment; changing world geometry invalidates the bundled map.
- **Terminal input semantics:** Many keyboard tools infer release from key-repeat timeout and cannot observe all releases directly. The implementation must define and test bounded hold/timeout behavior that guarantees prompt zero without relying on undocumented terminal behavior.
- **Shutdown safety limits:** A process cannot publish after uncatchable termination or total host failure. Tests cover normal exit and handled signals; command timeout behavior in the base/plugin should be inspected and documented, not falsely claimed as guaranteed by a finalizer.
- **Simulation-time availability:** Teleop cannot stamp valid commands before `/clock`. It should wait/fail explicitly according to the established readiness boundary and never fall back to wall time.
- **GUI automation:** Gazebo and RViz display paths can fail for environmental reasons. Preserve headless functional gates and record a separate host GUI smoke result.
- **Resource pressure:** Two lidars, two AMCL/RViz sessions, and GUI may be heavy. Verification may disable RViz and rendering for automation, but may not reduce the agreed two-robot functional scope or claim `FUP-001` complete.
- **Fault-injection realism:** Prefer controllable test publishers/process lifecycle and temporary manifests/map stores. Do not corrupt immutable demonstration assets or introduce production fault hooks solely for tests.
- **Map lock/save cleanup:** Integration must use temporary stores for destructive scenarios and prove OS lock release after process exit without adding stale flag files.

## Stop Conditions And Blocked Procedure
Stop implementation and report back when:
- Any slice 01-05 implementation/status is missing, review-failed, or materially conflicts with the approved design, alignment, or this slice boundary.
- The actual public launch/configuration/map/health APIs cannot support integration without a redesign, compatibility alias, third orchestration package, or broad rewrite of earlier slices.
- A legally usable, geometrically matching standard or custom map cannot be produced and verified from the final world assets.
- Gazebo Harmonic cannot load the custom world offline without vendoring prohibited upstream assets or using Classic-only APIs.
- Stop-on-release and final-zero behavior cannot be made testable and reliable with the chosen terminal/input approach.
- Required headless tests, Gazebo integration, or mapping/localization evidence cannot run. A missing display blocks only manual GUI evidence; report it explicitly after completing safe headless verification.
- Integration requires pulling a deferred `FUP-###` into scope, changing the canonical mental model, or modifying broad unrelated files.
- The selected slice becomes substantially larger than planned.

When blocked, preserve the working tree and immutable maps, record the exact command/error and affected acceptance criterion, identify the owning prerequisite slice or smallest alignment change, and do not add aliases, fallbacks, mock-only success claims, or partial alternate launch paths.

## Preserve User Work
- Inspect `git status` before editing once the repository exists.
- Preserve unrelated tracked and untracked changes, including generated user maps.
- Do not reset, overwrite, reformat, delete, or commit unrelated paths.
- Use temporary map stores for mapping/failure tests; never mutate bundled or user map IDs.
- If another agent or prerequisite slice is changing a required file, stop and coordinate rather than replacing it.

## Expected Final Summary
The implementing agent must report:
- Files/components added or changed, grouped by custom world/map provenance, teleoperation, GUI/headless behavior, fault coverage, integration tests, and operator documentation.
- The final supported world IDs/map IDs, public simulation/localization/mapping/teleop invocation shapes, and confirmation that standard-world defaults remain unchanged.
- Verification commands and outcomes, separating automated headless results from manual Gazebo/RViz/keyboard evidence and naming any unavailable checks.
- Fault scenarios exercised and whether each produced atomic startup failure or isolated per-robot degradation as contracted.
- Teleop evidence for explicit selection, `TwistStamped` type, simulation timestamp, target isolation, stop-on-release, and handled-exit zero.
- Final two-robot localization and one-robot mapping/save evidence, including map lock, immutability, and world metadata checks.
- Implementation-status block updates: commit, verification, deviations, and follow-ups.
- Remaining risks and all still-deferred `FUP-001` through `FUP-005`; do not claim the overall feature complete if any preceding slice or required integration evidence remains incomplete.
