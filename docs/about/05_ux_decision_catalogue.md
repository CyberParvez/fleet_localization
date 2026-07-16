# UX Decision Catalogue

## Robot-Scoped Operator Sessions
- Every operator session targets exactly one explicitly selected robot.
- A robot-specific visualization shows only that robot's map, sensor data, model, transforms, estimator output, and initialization controls.
- Controls that mutate robot state, including initial-pose estimation, must publish only to the selected robot's namespaced interface.
- Background robot instances may run without opening a visualization session.
- Applied example: `fleet_localization` opens an RViz session fixed to `<robot>/map` and routes 2D Pose Estimate to `/<robot>/initialpose`.

## Safe Manual Robot Motion
- Manual motion requires an explicit valid robot selection before command publication begins.
- Releasing movement input stops the selected robot.
- Normal exit or interruption sends a final zero-velocity command.
- Invalid or unknown robot selection fails before any velocity publisher becomes active.
- Applied example: TurtleBot fleet teleoperation publishes stamped commands only to the selected `/<robot>/cmd_vel` interface.

## Observable Localization Health
- Operator status distinguishes waiting for inputs, ready, awaiting initial pose, localized or mapping, and degraded/error states.
- A running process alone must not be presented as successful localization.
- Localized status depends on fresh inputs, a resolvable transform chain, a current finite estimate, and configured uncertainty thresholds.
- Degradation and recovery use persistence windows to avoid flickering status.
- Applied example: each `fleet_localization` invocation reports health for only its selected robot and isolates failures from other robot invocations.

## Validated Source Families
- `docs/pipeline/_archive/2026-07-16-08-06-turtlebot-gazebo-host-localization/2_alignment.md`
