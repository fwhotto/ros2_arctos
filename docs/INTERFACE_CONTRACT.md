# Simulation interface contract

This is the agreed target contract for Gates B–E. Gate A establishes package
boundaries; existing model and launch files have not yet been migrated to it.

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

## Initial pose: validation pending

The candidate is zero radians for all six arm joints, matching the existing
home configuration. This is **not yet certified collision-free**. Gate B must
validate it against the corrected model and support geometry, or select a new
pose. Gate C will install one shared initial-position configuration consumed by
mock, simulation and named planning states. Do not duplicate an unvalidated pose
across newly added configuration files.

Joint axes/limits and physical properties remain subject to Gate B's model
review. This contract fixes public names and ownership, not disputed geometry.
