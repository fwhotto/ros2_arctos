# Arctos control

Gate B provides the validated bare-arm pose in `config/initial_positions.yaml`.
`urdf/mock.urdf.xacro` consumes it and adds GenericSystem mock hardware to the
canonical model. `config/ros2_controllers.yaml` defines the position-only
`arm_controller` and joint state broadcaster. The interactive mock launch is
`ros2 launch arctos_moveit_config mock_demo.launch.py`.

The contract is documented in `docs/INTERFACE_CONTRACT.md` at repository root.
This package depends on the robot description and ROS control packages, not
Gazebo or CAN drivers.
