# Arctos control

Gate A establishes this package's dependency boundary. Shared controller YAML,
control-interface Xacro and mock launch are implemented in Gate C.

The contract is documented in `docs/INTERFACE_CONTRACT.md` at repository root.
This package depends on the robot description and ROS control packages, not
Gazebo or CAN drivers. It currently provides no runnable backend.
