# Simulation interface contract

This is the agreed contract for Gates B–E. Gate B implements the geometry,
frames, optional attachment and initial pose. Controller and MoveIt runtime
configuration follow in later gates.

| Interface | Decision |
| --- | --- |
| Arm joints, in order | `X_joint`, `Y_joint`, `Z_joint`, `A_joint`, `B_joint`, `C_joint` |
| Planning group | `arm` |
| Controller | `arm_controller` |
| Action | `arm_controller/follow_joint_trajectory`, relative to robot namespace |
| Commands | Position, radians |
| Feedback | Position (radians), velocity (radians/second) |
| Root | URDF `world` link with fixed `world_joint` to `base_link` |
| TF ownership | `robot_state_publisher` publishes the model's fixed and moving transforms; no duplicate world-to-base broadcaster |
| SRDF root | Use the URDF world root; no additional world-to-base virtual joint |
| Gazebo base | Preserve/verify fixed world attachment during URDF-to-SDF conversion; do not make the entire articulated model static |
| Tool | `tool0`, fixed to `Link_6_1` with zero translation and rotation initially; axes coincide with the wrist link, not an assumed REP-103 tool convention |
| Tool purpose | Bare-wrist planning endpoint and attachment frame; each future tool defines its own TCP transform |
| Backend selection | Explicit `mock` or `gazebo`; reserved `hardware` path remains unvalidated on Jazzy |
| Controller manager | Standalone in mock mode; owned by `gz_ros2_control` in Gazebo mode |
| Joint-state source | `joint_state_broadcaster` during execution |
| Time | Simulator `/clock` and `use_sim_time=true` in Gazebo; wall time in mock |
| Gripper | Optional, disabled in the first milestone |

## Validated initial pose

All six arm joints are at zero radians. The authoritative configuration is
`arctos_control/config/initial_positions.yaml`; Gate C/D backends and later
planning states should consume it. MoveIt/FCL verifies the bare arm is free of
non-adjacent self-collision and clears a support surface at world z=0, permitting
only the mounted base to contact the support.

See [the model review](MODEL_REVIEW.md) for axis/limit decisions, inertia
provenance, the fixed gripper preview, and the scope of collision validation.
