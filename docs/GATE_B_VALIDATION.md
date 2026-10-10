# Gate B validation report

Date: 2026-10-10.

## Environment and command

- Ubuntu 24.04.5, aarch64, ROS 2 Jazzy underlay only.
- Gazebo Sim 8.15.0 (Harmonic), MoveIt 2.12.4.
- Fresh non-symlink build/install directories; no physical hardware nodes run.
- No system packages or external Python packages were installed for this task.

```bash
bash scripts/validate_jazzy.sh /tmp/opencode/arctos-gate-b-final
```

Result: dependency resolution passed, all four packages built, **27 pytest
cases passed**. Colcon reports 30 tests because it also counts three CTest
wrappers. Zero failures/errors/skips. Total colcon test phase: approximately
3.1 seconds on this host.

## Checks performed

### Description: 12 cases

- Default, prefixed and optional fixed-gripper model expansion and `check_urdf`.
- Six moving joints with no implicit hardware, Gazebo plugin or ROS 1
  transmissions in the default description.
- Canonical bounds, normalized joint axes, positive velocity/effort limits and
  explicit damping/friction.
- Positive masses and inertia eigenvalues, principal-moment triangle
  inequalities, COM bounds and radius-of-gyration plausibility for bare and
  optional fixed-gripper variants.
- Identical shared geometry/axes/limits for the first three joints of both
  variants.
- Zero-pose meshes match CAD assembly coordinates within 0.1 mm; all moving
  links clear z=0 by more than 0.1 m.
- Full/reduced legacy control bounds agree with URDF bounds.
- Two prefixed geometry-macro instances have unique link/joint names.
- The legacy reduced control include still emits only three joints.

### Packaging: 11 cases

Gate A checks continue to pass: dependency closure, simulator-independent core,
dependency availability, and installed resource completeness/content.

### SDF and collision: 4 cases

- `gz sdf -p` conversion emits no diagnostics, preserves all six revolute joints
  and their inertial child links/limits/damping, retains the fixed world-to-base
  joint, and preserves `tool0` as an SDF frame after fixed-link lumping.
- MoveIt/FCL finds no non-adjacent self-collision at the shared all-zero pose.
  Only adjacent pairs are excluded; the legacy SRDF is not used.
- A support box with its top at z=0 does not collide with moving links. Only
  base/support contact is permitted. An obstructing box is correctly detected.
- A known folded pose is correctly detected as self-colliding.

The test harness initially assumed that plain-CMake `urdfdom` registered an
ament resource and that MoveIt Python's `apply_collision_object` returned a
boolean. Both assumptions were corrected: executable discovery checks urdfdom,
and the collision test asserts the object actually exists in the planning
scene. The final full run passes after these fixes.

## Model decisions and scope

See [MODEL_REVIEW.md](MODEL_REVIEW.md) for the pinned reference comparison,
identical-mesh evidence, wrist COM transformation, limits and physical-property
assumptions. The initial pose is now stored in
`arctos_control/config/initial_positions.yaml` for later backend consumption.

Gate B establishes valid model structure, plausible physics properties and a
collision-free bare-arm initial pose. This is not a measurement of real-arm
dynamics, a gripper-contact test, or a running Gazebo controller test. Pose
holding, control tuning, actual world spawning and MoveIt trajectory execution
remain Gates C–E. Optional gripper jaws are fixed previews until the gripper
milestone.

Local logs and version inventories are under the result directory above;
re-run the script with a fresh directory to reproduce the checks.
