# Arctos Jazzy / Gazebo Implementation Plan

## Goal and scope

Target Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, MoveIt 2, and
`gz_ros2_control`.

The first milestone is a six-axis Arctos arm that can be spawned in Gazebo,
hold a pose, and execute a MoveIt-planned trajectory from RViz. Gazebo and RViz
must agree on the resulting robot state. Provide a repeatable headless
verification path as well as an interactive demo.

Start with a bare arm and an explicit tool attachment frame. Gripper actuation,
grasping, sensors, accurate actuator dynamics, multi-arm operation, and physical
hardware validation are subsequent milestones.

This plan is based on source inspection, not successful builds or runtime tests.
Validate the environment and current package behavior during implementation.

## Architecture

Retain the standard ROS interfaces:

```text
RViz / application
        |
        v
     MoveIt 2
        | FollowJointTrajectory
        v
joint_trajectory_controller
        | position commands
        v
    ros2_control
        +-- GazeboSimSystem --> Gazebo joints
        +-- GenericSystem  --> mock joints
        +-- ArctosInterface --> physical motors (later validation)

Joint states --> robot_state_publisher --> TF
             --> MoveIt current state
```

### Package ownership

| Package | Responsibility |
| --- | --- |
| `arctos_description` | Meshes, geometry, physical properties, joint definitions, tool/attachment frames |
| `arctos_control` (new) | Shared control-interface Xacro, controller configuration, mock backend startup |
| `arctos_moveit_config` | SRDF, kinematics, planning pipelines and limits, MoveIt controller mapping |
| `arctos_gazebo` (new) | Gazebo wrapper/plugin configuration, worlds, spawning, clock bridge, simulation entry point |
| `arctos_bringup` | Physical robot startup and CAN orchestration |
| `arctos_hardware_interface` | Physical `ros2_control` backend |
| `arctos_motor_driver` | CAN protocol and motor operations |

The simulation dependency closure must build without requiring the physical
driver packages. Description and shared control packages must be usable without
installing Gazebo. Keep simulator-specific XML in a simulation wrapper and
select the control backend explicitly.

Generate one consistent robot description for robot state publisher, Gazebo,
and MoveIt. Avoid separate launch files silently selecting different models.

### Configuration ownership

- Physical joint bounds, axes, geometry, and inertias: `arctos_description`.
- Command/state interfaces, controller settings, and shared initial positions:
  `arctos_control`.
- Planning-specific settings and conservative velocity/acceleration limits:
  `arctos_moveit_config`, checked against physical bounds.
- World, simulator physics, resource paths, and bridges: `arctos_gazebo`.
- Load shared data where practical; otherwise add checks for consistency.

## 0. Freeze the interface contract

Complete this short design step before parallel implementation.

- [ ] Preserve arm joints `X_joint`, `Y_joint`, `Z_joint`, `A_joint`, `B_joint`,
      and `C_joint`.
- [ ] Use planning group `arm` and controller `arm_controller`.
- [ ] Expose action `arm_controller/follow_joint_trajectory` relative to the
      robot namespace.
- [ ] Use position command interfaces with position and velocity feedback.
- [ ] Define `world`, `base_link`, and `tool0`, including the tool transform and
      axis convention.
- [ ] Agree on one URDF/SRDF root and Gazebo anchoring arrangement, with a single
      authority for each TF transform.
- [ ] Define backend selection for `mock` and `gazebo`; preserve an extension
      point for hardware without claiming Jazzy hardware support yet.
- [ ] Choose one collision-free initial joint configuration, consumed by all
      relevant components.
- [ ] Make the gripper optional and disabled for the initial milestone.

## A. Jazzy environment and dependencies

- [ ] Inspect installed OS, ROS, Gazebo, and workspace overlays; record the
      supported environment and avoid mixing ROS distributions.
- [ ] Establish a reproducible Ubuntu 24.04 / Jazzy / Harmonic build environment.
- [ ] Declare runtime/build dependencies for Xacro, robot state publisher,
      controller manager, trajectory controller, joint-state broadcaster,
      MoveIt, `ros_gz_sim`, `ros_gz_bridge`, and `gz_ros2_control` in their owning
      packages.
- [ ] Replace stale Gazebo Classic references in the supported simulation path.
- [ ] Keep warehouse/database functionality optional for the basic demo.
- [ ] Provide a documented dependency installation and selective build path
      that does not pull in CAN packages.
- [ ] Verify package installation rules include all required launch, model,
      configuration, and world resources.

**Gate:** dependencies resolve and the simulation package closure builds in a
clean Jazzy environment. Physical backend migration is not a prerequisite.

## B. Canonical full-arm model and reference comparison

### Reference repository

Compare against `https://github.com/coen132/Arctos` at inspected commit
`603bdcb079340604b5da2b364c8c03169d0ea96c`.

Useful reference file:
`arctos_description/urdf/arctos_urdf.xacro`.

That model provides alternative wrist/jaw inertias and opposing jaw axes, but
also substantially different masses and link origins. For example, its wrist
mass is about 0.103 kg rather than the local 0.001 kg; its second-link mass is
about 11.56 kg rather than 2.53 kg. Neither set should be treated as validated
physical measurements. Compare geometry and frames before adopting properties.

Its Gazebo file loads the obsolete `libgazebo_ros_control.so`; its MoveIt/control
configuration has naming and backend inconsistencies. Use it as a model
reference, not as a replacement simulation stack.

### Tasks

- [ ] Compare joint origins, axes, and mesh alignment across the two projects;
      document the chosen convention and provenance of adopted model data.
- [ ] Refactor full-arm geometry into reusable Xacro macros, independent of the
      hardware backend.
- [ ] Reconcile the full/reduced model Y-axis direction discrepancy.
- [ ] Correct reversed Y control limits and malformed expressions such as
      `{-2*pi}`.
- [ ] Reconcile physical and control bounds, including Z and wrist limits.
- [ ] Validate masses, centers of mass, inertia tensor symmetry/positivity and
      physical plausibility; investigate the wrist discrepancy specifically.
- [ ] Make the gripper optional so zero-mass moving jaws do not enter the
      initial simulation.
- [ ] Add a documented tool attachment frame.
- [ ] Remove obsolete ROS 1-style transmissions from the new simulation path.
- [ ] Check collision geometry, ground clearance, initial self-collisions and
      model anchoring; simplify collision meshes where needed.
- [ ] Set justified damping/friction values where needed for stable simulation.
- [ ] Validate Xacro expansion and URDF structure; inspect URDF-to-SDF conversion
      to ensure all six moving joints survive.

**Gate:** one consistent six-axis model with valid physics properties and a
collision-free initial pose. Accurate motor/gear dynamics are deferred.

## C. Shared controllers and mock execution

- [ ] Introduce `arctos_control` with shared joint interfaces and backend
      selection.
- [ ] Configure `joint_state_broadcaster` and `arm_controller` consistently for
      the six arm joints.
- [ ] Resolve the existing `arctos_controller` versus
      `joint_trajectory_controller` naming mismatch by adopting the contract.
- [ ] Resolve command-interface mismatches: position commands for all six
      joints, position/velocity state interfaces.
- [ ] Centralize initial positions and set explicit update rate and trajectory
      tolerances.
- [ ] Provide mock bringup using `mock_components/GenericSystem` and a standalone
      controller manager.
- [ ] Send a direct `FollowJointTrajectory` goal and verify successful action
      completion and final joint positions.

**Gate:** mock trajectory execution works independently of MoveIt and Gazebo.

## D. Gazebo integration

- [ ] Introduce `arctos_gazebo` and a minimal world with ground/support geometry.
- [ ] Add the `gz_ros2_control/GazeboSimSystem` hardware backend and Gazebo system
      plugin, loading the shared controller configuration.
- [ ] Configure mesh/resource resolution from installed package paths.
- [ ] Spawn the arm with explicit pose and a verified fixed-base arrangement.
- [ ] Bridge simulator time to `/clock` and propagate `use_sim_time` consistently.
- [ ] Use the plugin-owned controller manager; do not launch a second standalone
      manager in Gazebo mode.
- [ ] Sequence spawn and controller activation using readiness checks,
      timeouts, and process success checks rather than arbitrary sleeps or
      treating every process exit as success.
- [ ] Ensure the joint-state broadcaster is the authoritative joint-state
      source; do not run a competing joint-state publisher.
- [ ] Support configurable world, GUI/headless operation, RViz selection, and
      robot namespace.
- [ ] Verify stable pose holding, then execute a direct six-joint trajectory.
- [ ] Tune simulator/controller timing and position response if needed to meet
      explicit execution tolerances.

**Gate:** Gazebo holds the initial pose and completes a direct controller goal
with valid feedback, without physical hardware nodes.

## E. MoveIt configuration

- [ ] Reconcile or regenerate the full-arm SRDF against the canonical model.
- [ ] Fix stale `base_joint`, `jaw1`, and `jaw2` references and root/virtual-joint
      inconsistencies.
- [ ] Define the six-joint arm chain and `tool0` endpoint.
- [ ] Configure KDL and a basic OMPL planning pipeline for Jazzy.
- [ ] Set usable velocity/acceleration limits and trajectory time parameterization.
- [ ] Map execution to `arm_controller/follow_joint_trajectory`.
- [ ] Review/regenerate the self-collision matrix after model geometry is final.
- [ ] Define validated initial/home and target configurations.
- [ ] Ensure MoveIt uses the same robot description and simulation time as the
      execution stack.
- [ ] Represent ground/support geometry in the planning scene. Gazebo objects
      are not automatically MoveIt collision objects.
- [ ] Plan and execute known joint-space goals with the mock backend.
- [ ] Validate a reachable tool-pose goal, then repeat with Gazebo.

**Gate:** MoveIt planning and execution work through the shared action interface
with both mock and Gazebo backends.

## F. End-to-end verification and documentation

- [ ] Provide one documented launch command for Gazebo + controllers + MoveIt +
      RViz, with a headless equivalent.
- [ ] Add a small repeatable planning/execution example; keep task logic out of
      launch files.
- [ ] Add targeted model/configuration consistency checks for joint names,
      interfaces, limits, and required frames.
- [ ] Add integration coverage for controller readiness, direct trajectory
      completion, and MoveIt plan-and-execute.
- [ ] Set explicit acceptance tolerances and timeouts; assert action results and
      measured final joint positions rather than relying on visual inspection.
- [ ] Run headless checks in a pinned Jazzy/Harmonic environment.
- [ ] Check repeated fresh startup/shutdown, including cleanup after failures.
- [ ] Document dependencies, launch arguments, backend behavior, expected
      topics/actions/frames, and common startup/execution failures.
- [ ] Clearly document which packages/configurations are Jazzy-validated and the
      status of the reduced three-joint and physical hardware paths.

## Parallel execution and sequencing

### Workstream ownership

Agree on the interface contract first. Assign ownership by package to avoid
parallel edits to shared files. Changes to joint names, frame conventions, or
controller interfaces must be coordinated across workstreams.

| Workstream | Can start | Validation dependencies |
| --- | --- | --- |
| A: Environment/dependencies | Immediately after target selection | Final package graph for clean-build verification |
| B: Model/reference comparison | After interface agreement | No runtime dependency on other implementation streams |
| C: Controllers/mock | Alongside model cleanup | Canonical model for execution tests |
| D: Gazebo world/plugin/launch scaffolding | Alongside B and C | A + B + C for motion validation |
| E: MoveIt scaffolding | Alongside D | B for final SRDF/collision matrix; C for execution |
| F: CI/docs scaffolding | Alongside implementation | Working launches for final integration checks |

### Required integration order

```text
Interface contract
       |
       +--> Environment/dependencies ---------------------+
       +--> Canonical model/reference comparison          |
       +--> Shared control configuration                  |
                         |                                |
                  Validated model                         |
                         |                                |
                  Mock controller execution <-------------+
                         |
             +-----------+------------+
             |                        |
     Direct Gazebo trajectory   MoveIt + mock execution
             |                        |
             +-----------+------------+
                         |
                  MoveIt + Gazebo
                         |
              Repeated end-to-end checks
```

Gazebo and MoveIt scaffolding can precede model completion, but final physics,
collision, and motion validation must use the canonical model.

## Acceptance criteria for milestone one

- [ ] A clean Jazzy environment builds the documented simulation package set.
- [ ] One command starts the six-axis arm, Gazebo, controllers, MoveIt and RViz.
- [ ] Exactly one controller manager owns the simulated arm and all six joints
      have the expected interfaces.
- [ ] Joint states, TF and time are consistent, with no competing publishers.
- [ ] The anchored arm remains stable at its initial pose.
- [ ] A direct trajectory completes successfully within stated tolerances.
- [ ] MoveIt plans and executes between at least two validated configurations.
- [ ] A reachable tool-pose goal also plans and executes successfully.
- [ ] Gazebo and RViz agree on the final pose.
- [ ] Headless verification and repeated fresh launches succeed.
- [ ] Simulation startup requires no CAN hardware or calibration scripts.

## Extension roadmap

1. **Gripper:** validate geometry and inertias against the reference model; add
   correct jaw coupling/mimic behavior, controller, MoveIt group, contact
   properties, and grasp tests.
2. **Worlds and tasks:** package application motion sequences separately; give
   world objects corresponding planning-scene representations.
3. **Sensors:** add optional attachment macros and simulator bridges.
4. **Physical backend:** migrate and validate the hardware interface on Jazzy
   against the same action and joint contracts.
5. **Reduced model:** derive the three-joint variant from shared definitions
   instead of maintaining divergent geometry/control copies.
6. **Multi-arm support:** build on relative ROS names and prefix-ready model
   macros; validate namespace, frame, controller and planning isolation.
7. **Higher-fidelity dynamics:** refine measured inertias, friction, actuator
   limits and gear behavior after basic planning/execution is stable.

## References

- Jazzy Gazebo integration:
  https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html
- Alternative Arctos model:
  https://github.com/coen132/Arctos/tree/603bdcb079340604b5da2b364c8c03169d0ea96c
