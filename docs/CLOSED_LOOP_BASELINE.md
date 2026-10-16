# Closed-loop kit baseline and collaborator adaptations

Working assumption: latest **documented closed-loop kit**, reviewed 2026-10-16
against [the manufacturer's documentation](https://arctosrobotics.com/docs/).
This is a dated software baseline, not a confirmed bill of materials for the
collaborator's delivered kit. The official documentation calls the assembly
v2.9.7 while linking a “v0.2.9.1 (Latest)” manual. Record the actual CAD/kit
revision when it becomes available rather than treating those labels as equal.

## Verified documentation versus current simulation assumptions

| Item | Manufacturer documentation / working baseline | Adaptation when kit is confirmed |
| --- | --- | --- |
| Electronics | CANable v2, 2× MKS Servo 57D and 4× MKS Servo 42D; wiring table assigns 57D to X/Y | Confirm board and firmware revisions; update `arctos_motor_driver` protocol and `arctos_bringup` configuration |
| Alternative drivers | CLMD is described as **in development**, not the default kit driver | If supplied, implement its protocol/feedback mapping explicitly rather than assuming MKS compatibility |
| CAN IDs | Driver setup table: X/Y/Z/A/B/C = 01/02/03/04/05/06. The specifications table instead lists B as 07 | Resolve this documentation conflict from actual driver settings; do not derive bus IDs from our URDF joint names |
| Reductions | X 13.5:1, Y/Z 150:1, A 48:1, B/C 67.82:1 | Verify motor-to-output conversions in `arctos_hardware_interface` and the motor driver; avoid applying reductions twice |
| Wrist coupling | Manufacturer Studio code uses motor combinations C−B and C+B | Confirm motor signs and reduction order; implement inverse mapping for measured feedback as well as forward commands |
| Geometry | Existing manufacturer ROS meshes, matching our CAD assembly; current docs note changed A/B/C motor mounting | Compare actual latest CAD with `arctos_description/urdf/arctos_arm.xacro` and meshes; revise origins, collision geometry and inertias if mounting changed |
| Joint travel and zero | Studio software defaults with our elbow sign conversion, documented in `MOTION_SPECIFICATIONS.md` | Confirm output-axis signs, homing offsets and usable travel; update `physical_joint_limits.yaml`, initial positions and SRDF poses together |
| Feedback and homing | Encoder-feedback MKS drivers with driver-specific endstop/remap instructions | Confirm encoder units, wrapping, polarity and home offsets; put calibration in the hardware backend, keeping ROS joint states in radians/metres |
| Gripper electronics | Closed-loop docs specify Nano firmware, 6 V supply and commands `0763` open / `0700` close | Confirm firmware command encoding and available feedback; these command strings do not establish distance or measured jaw feedback |
| Gripper kinematics | 15 mm per jaw and 30 mm opening from the older official ROS model and mesh measurements | Verify supplied jaw/servo design; update `arctos_gripper.xacro`, control bounds and SRDF open/closed targets if travel differs |
| Dynamics | Existing masses/inertias and speed/effort limits are simulation estimates | Measure/obtain revision-specific values before treating Gazebo tracking as torque/payload evidence |

The DS3225 is named in the **open-loop** gripper wiring section. It is not
independent confirmation of the servo shipped with the latest closed-loop kit.
Likewise, “closed-loop arm” does not establish that the gripper reports measured
jaw positions. The earlier motion review's command references should be read
with this distinction.

## What closed-loop means in this milestone

The simulation preserves output-joint position commands and position/velocity
feedback through `gz_ros2_control/GazeboSimSystem`. Controllers and MoveIt use
the same names as the mock backend. The simulated mimic follower is a physical
engine constraint, and validation checks Gazebo feedback independently from
the controller's published state.

This interface is suitable for a future MKS hardware backend, but simulation
does not establish physical encoder accuracy, homing behavior, force limits or
bus performance. No hardware calibration values have been guessed from the
ambiguous CAN table.

## Collaborator handoff

Collect kit/CAD revision, motor and driver labels, firmware versions, actual
CAN IDs, gear ratios, home-position photos/offsets, wrist motor directions,
and gripper firmware/model/stroke. Compare against the table above. Then rerun
model/collision checks and the mock/Gazebo named-pose checks after any geometry,
limits or calibration changes.
