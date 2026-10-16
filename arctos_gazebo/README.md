# Arctos Gazebo

Target: ROS 2 Jazzy and Gazebo Harmonic via `gz_ros2_control`.

## Build and launch

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to arctos_gazebo
source install/setup.bash
ros2 launch arctos_gazebo demo.launch.py
```

This starts Gazebo, the arm and gripper controllers, MoveIt and RViz. Gazebo owns
the controller manager. Joint commands and initial positions are shared with
the mock backend; the independent Gazebo state stream is `/gazebo/joint_states`.
All robot/planning nodes use simulation time from the one-way `/clock` bridge.

Options: `gui:=false rviz:=false` for headless operation;
`include_gripper:=false` for the bare arm; `moveit:=false rviz:=false` for the
simulator/controller checkpoint alone. Run only one demo per ROS domain.

## User validation checkpoints

### VM graphics compatibility

If Gazebo reports `OpenGL 3.3 is not supported` and crashes in Ogre2 while RViz
reports OpenGL 2.1, select the legacy Ogre GUI renderer:

```bash
ros2 launch arctos_gazebo demo.launch.py render_engine:=ogre
```

After updating the launch file, rebuild `arctos_gazebo` and source the workspace.
This changes the GUI renderer only; Bullet physics and controllers are identical.
Alternatively, `gui:=false` runs Gazebo physics with RViz still available.
The missing Octomap sensor and `/recognize_objects` messages refer to optional
perception features and are unrelated to the OpenGL crash.

### 1 — Model and pose holding

Launch with `moveit:=false rviz:=false`. Confirm:

- One anchored arm appears upright, with the two gripper jaws open.
- Links, wrist and jaw meshes align; there is no falling or visible shaking.
- `ros2 control list_controllers` shows `joint_state_broadcaster`,
  `arm_controller` and `gripper_controller` **active**.

Confirm the wrist mounting and gripper appearance with your collaborator. The
[kit adaptation notes](../docs/CLOSED_LOOP_BASELINE.md) identify provisional geometry.
Stop this launch before moving to checkpoint 2.

### 2 — Arm motion agrees between Gazebo and RViz

Launch the default demo. Wait for `Planning ground ready` in the terminal and
active controllers. In RViz select group **arm**, current start state and named
goal **demo**, then **Plan & Execute**. Confirm the physical-looking Gazebo arm
and RViz robot move together and settle. Execute **home** to return.

### 3 — Gripper motion

Select group **gripper**, goal **closed**, then **Plan & Execute**; repeat with
**open**. Both jaws must travel equal distances in opposing directions in both
windows. The nominal opening is 30 mm, with 15 mm travel per jaw.

Please confirm checkpoints 1–3 before the next object-contact/grasping milestone.
They have headless coverage below; visual acceptance is still a user checkpoint.

## Headless validation

After sourcing the built workspace, from the repository root:

```bash
python3 scripts/validate_gazebo.py
python3 scripts/validate_gazebo.py --bare
```

Each command launches an isolated Gazebo transport partition on ROS domain 86,
checks clock/feedback and the planning ground, plans and executes named targets,
holds each for five seconds, and shuts down its launch. Use `--domain` to select
an unused ROS domain and `--output` to change the evidence directory. Default
logs: `/tmp/opencode/arctos-gazebo-runtime`.

For an already running launch in the same ROS domain:

```bash
python3 scripts/test_mock_moveit.py --sim-time --hold-seconds 5
```

Add `--bare` for the bare-arm variant. This commands motion. Arm tolerance is
0.01 rad; jaw tolerance is 0.1 mm. Both ros2_control feedback and Gazebo's separate
joint-state stream must remain fresh and agree throughout each hold.

## Physics scope

The world uses gravity, a fixed world anchor, 1 ms physics steps and **Bullet
Featherstone**. The installed DART engine did not move the mimic follower;
Bullet supports its SDF constraint and passes the two-jaw runtime check.
The ground is at z = −1 mm in both Gazebo and MoveIt, keeping the bolted base
clear of numerical ground contact. Gazebo renders a 10 m plane; MoveIt uses
an infinite plane at the same height.

Position control approximates closed-loop output-joint tracking. It does not
reproduce MKS encoder quantization, gearbox compliance, torque response or CAN
latency. Gripper planning bounds alone have a 1 µm endpoint allowance for
physics round-off; URDF/control travel remains 0–15 mm. Mesh self-contact,
grasp friction and payload handling are not validated by these free-space tests.

Evidence and remaining checkpoints: [Gazebo validation](../docs/GAZEBO_VALIDATION.md).
