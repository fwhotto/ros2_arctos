# arctos_description

## Description

This package provides the URDF description of the Arctos robot.

## Jazzy model (Gate B)

After building and sourcing the workspace:

```bash
xacro "$(ros2 pkg prefix --share arctos_description)/urdf/arctos.xacro" > /tmp/arctos.urdf
check_urdf /tmp/arctos.urdf
```

The default is an anchored six-axis bare arm with `tool0`, no gripper and no
control plugin. `arctos_arm.xacro` provides a reusable macro for custom mounts;
it accepts `prefix`, `include_wrist` and `include_gripper`.

Entry-point arguments:

- `prefix:=left_`: prefix robot links and joints (world stays shared).
- `include_gripper:=true`: 15 mm opposing prismatic jaws, one independent joint
  and a mimic follower. The RViz mock demo adds the controller.
- `use_hardware:=true`: explicit legacy hardware-plugin opt-in; physical Jazzy
  operation is not validated.

The reduced `urdf/base_xyz/arctos.xacro` entry point uses the same first three
joint/link definitions. Its old reversed Y-axis convention is no longer used.
Legacy launch/controller configurations require migration before execution;
the model no longer implicitly selects physical hardware.

Physical bounds are in `config/physical_joint_limits.yaml`. The initial pose is
owned by `arctos_control/config/initial_positions.yaml`. See
`docs/MODEL_REVIEW.md` in the repository for provenance and validation details.
