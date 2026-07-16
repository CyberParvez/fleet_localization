# TurtleBot Gazebo Host Localization Backlog

## Purpose
Capture selected recommendations and issue context for later `1_Brainstorm` and `2_Align`. This file preserves starting context; it does not approve a solution, implementation scope, or slice plan.

## Repository And Routing
- Repository: `/home/syncrobot/localization` (not initialized as a Git repository at capture time)
- Branch: unavailable
- Feature folder: `docs/pipeline/turtlebot-gazebo-host-localization`
- Current phase: backlog capture
- Later phase entry point: `1_Brainstorm`
- Routing note: No repository routing rules or existing pipeline artifacts were present. The folder name follows the feature described by the user.

## Source Materials
| Source | Location / Link | Notes |
| --- | --- | --- |
| User request | Current conversation | Requests a TurtleBot localization demo using Gazebo and ROS 2 Jazzy, with the simulation running in a Docker container and localization running on the host PC. |

## Selection Criteria
- Selected because: This is the single demo concept explicitly requested for capture.
- Not selected because: No additional recommendations were supplied.
- Time sensitivity or trigger: Explore when the user is ready to define the demo behavior and environment boundaries.

## Issue Context
- Current observation: A reproducible demo is wanted in which a containerized TurtleBot Gazebo simulation supplies data to localization software on the host PC under ROS 2 Jazzy.
- Evidence or symptoms: The user identified the desired simulator, robot family, ROS distribution, container boundary, and host-side localization placement; no existing project files or runtime evidence are present in the workspace.
- Affected user/operator: A developer or robotics operator demonstrating and validating localization across a Docker-to-host boundary.
- Impact if ignored: The demo lacks preserved context for deciding interoperability, observability, startup, and validation expectations before implementation.
- Known constraints: TurtleBot; Gazebo; ROS 2 Jazzy; Gazebo runs from a Docker container; localization runs on the host PC.
- What remains undecided: TurtleBot model, Gazebo release and world, host OS, localization package and mode, map source, ROS middleware and discovery configuration, graphical-display arrangement, sensor and transform contracts, launch workflow, and success evidence.
- Context that must not be lost: Localization must execute on the host rather than inside the simulation container.

## Selected Recommendations Summary
| ID | Short label | Source | Why selected | Suggested next phase |
| --- | --- | --- | --- | --- |
| BLI-001 | Containerized TurtleBot simulation with host localization | User request | It captures the requested demo and its essential runtime boundary. | `1_Brainstorm` |

## Backlog Items

### BLI-001: Containerized TurtleBot Simulation With Host Localization
- Status: captured
- Source recommendation: Create a TurtleBot localization demo using Gazebo and ROS 2 Jazzy, run Gazebo from a Docker container, and run localization on the host PC.
- Source context: Direct user request in the current conversation.
- Issue context: The demo crosses a container/host boundary, so later design work must establish what simulation data, transforms, time, and control interfaces cross that boundary while keeping localization host-side.
- Evidence or symptoms: No implementation, configuration, map, robot selection, or environment description currently exists in the workspace.
- Impact / why it matters: Capturing the boundary and unknowns reduces the risk that later work accidentally colocates localization with simulation or assumes incompatible ROS, networking, time, graphics, or model behavior.
- Affected users or systems: Demo operator, host ROS 2 environment, Docker runtime, Gazebo simulation, TurtleBot model, and the eventual localization subsystem.
- Known constraints: ROS 2 Jazzy is required; Gazebo and TurtleBot simulation are containerized; localization is executed on the host PC.
- Open questions for brainstorm or alignment: Which TurtleBot variant is required? Which Gazebo release and world should be used? Is the host also ROS 2 Jazzy, and what host platform is expected? Is localization map-based or mapping-and-localization? Which localization behavior and evidence define a successful demo? How should ROS discovery, middleware, simulation time, topics, transforms, sensor data, maps, visualization, and startup responsibilities cross the boundary? Must the demo support hardware acceleration or a headless mode?
- Candidate directions to explore, not decisions: Compare compatible TurtleBot simulation choices; compare host-side localization approaches; explore ROS 2 container-to-host communication patterns; explore graphical and headless simulator operation; define alternative demo validation evidence.
- Explicit non-decisions: No robot model, image base, Gazebo version, localization package, middleware, network mode, map, world, launch topology, visualization tool, file structure, or implementation approach is approved by this backlog.
- Suggested next phase: Run `1_Brainstorm` for BLI-001 to clarify intent, constraints, success criteria, and candidate approaches.

## Not Selected
| Source recommendation | Why not selected now | Revisit trigger |
| --- | --- | --- |
| None | No other recommendations were provided. | New user input or findings. |

## Repository Rules
- Consulted: Attempted `docs/rules/README.md` and `docs/rules/pipeline-docs.md`; neither exists in the current workspace.
- Constraints applied: Used the `0-backlog` artifact structure and preserved the request as undecided pre-design context.
- Rule updates: None
