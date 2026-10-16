# Arctos control

Gate B provides the validated bare-arm pose in `config/initial_positions.yaml`.
`urdf/mock.urdf.xacro` consumes it and adds GenericSystem mock hardware to the
canonical model. `config/ros2_controllers.yaml` defines the position-only
`arm_controller` and joint state broadcaster. The interactive mock launch is
`ros2 launch arctos_moveit_config mock_demo.launch.py`.

The mock wrapper enables a gripper by default. `gripper_controller` commands
`Left_jaw_joint`; the right jaw mimics it without its own command interface.
Use `include_gripper:=false` on the mock launch to run only the arm.

`urdf/controlled.urdf.xacro` owns the shared interfaces and initial positions.
`mock.urdf.xacro` selects GenericSystem by default; the Gazebo package's wrapper
selects GazeboSimSystem and adds simulator plugins. Launch the latter with
`ros2 launch arctos_gazebo demo.launch.py`.

The contract is documented in `docs/INTERFACE_CONTRACT.md` at repository root.
This package depends on the robot description and ROS control packages, not
Gazebo or CAN drivers.
