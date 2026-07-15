# Host-Native TurtleBot Localization Roadmap

## Context
- Repository: `/home/syncrobot/localization` (not initialized as a Git repository at planning time)
- Branch: unavailable; planning publication requires `dev`
- Feature folder: `docs/pipeline/turtlebot-gazebo-host-localization`
- Design source: [1_design.md](1_design.md)
- Alignment source: [2_alignment.md](2_alignment.md)
- UX source: [../../about/05_ux_decision_catalogue.md](../../about/05_ux_decision_catalogue.md)

## Vertical Slices
1. **Fleet Contract And Shared-World Simulation** — Establish the ROS workspace, strict shared fleet manifest, project-owned multi-robot Gazebo Harmonic launch, official TurtleBot3 Burger asset integration, atomic readiness, isolated topics/frames, and exclusive simulator TF boundary.
2. **Single-Robot EKF And Readiness** — Make one separately selected robot pass input readiness and run the canonical planar wheel-odometry/IMU EKF as sole odometry-transform owner with observable health.
3. **Single-Robot AMCL And Operator Session** — Load an immutable world-matched map copy for one robot, manage map-server/AMCL lifecycle, support manual initialization in a robot-scoped RViz session, and report measurable localization health.
4. **Independent Multi-Robot Localization** — Prove two separately launched localization instances can share map files while remaining isolated in topics, frames, locks, lifecycle, RViz, health, and failures.
5. **Exclusive Mapping And Immutable Map Save** — Replace one robot's AMCL path with asynchronous fresh-map SLAM Toolbox under exclusive map-store locking and explicit non-overwriting map persistence.
6. **Alternate World, Safe Teleoperation, And Integration Hardening** — Add the custom asymmetric world and matching map metadata, stamped stop-safe per-robot teleoperation, GUI/headless coverage, fault cases, and final two-robot integration evidence.

## Deferred Scope
No deferred item is pulled into this roadmap.

- `FUP-001`: performance guarantees beyond two robots remain deferred.
- `FUP-002`: physical non-TurtleBot profiles remain deferred.
- `FUP-003`: automatic initial pose remains deferred.
- `FUP-004`: a shared global multi-robot frame remains deferred.
- `FUP-005`: concurrent or collaborative mapping remains deferred.

## Dependencies And Recommended Order
- Implement slices sequentially from 01 through 06.
- Slice 01 establishes package, configuration, simulation, topic, and frame contracts used by every later slice.
- Slice 02 establishes selected-robot readiness, health, and local TF ownership before global localization.
- Slice 03 establishes one complete localization workflow before concurrency is introduced.
- Slice 04 validates independent repeated instances before map mutation is added.
- Slice 05 depends on the map model, health model, and estimator boundary from slices 02-04.
- Slice 06 completes alternate-environment, operator-control, failure, and integration behavior after both core modes exist.

## Next Planning And Implementation Source
- Plan next: `3_plan_slice_01_fleet_simulation.md`.
- After every slice plan is written, cross-checked, and approved, the first unimplemented slice plan becomes the implementation source.
- This roadmap is an index only. `3_plan.md` is not a valid implementation source for `4_Implement`.

## Repository Rules
- Consulted: [1_design.md](1_design.md), [2_alignment.md](2_alignment.md), [CURRENT.md](CURRENT.md), and [../../about/05_ux_decision_catalogue.md](../../about/05_ux_decision_catalogue.md); `docs/rules/README.md` and rule files do not exist.
- Constraints applied: Preserved two packages, per-robot localization invocation, independent map frames, one canonical interface, immutable maps, stable deferred IDs, and vertical end-to-end slice order.
- Rule updates: None
