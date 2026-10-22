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

Both launch files default to the legacy Ogre GUI renderer for OpenGL 2.1 VM
compatibility. No renderer argument is needed:

```bash
ros2 launch arctos_gazebo demo.launch.py
```

After updating the launch file, rebuild `arctos_gazebo` and source the workspace.
This changes the GUI renderer only; Bullet physics and controllers are identical.
On machines supporting OpenGL 3.3+, optionally select `render_engine:=ogre2`.
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

## Table/cube approach checkpoint

```bash
ros2 launch arctos_gazebo manipulation.launch.py
```

This adds a solid 24×24 cm pedestal/table with a 25 cm top height above the
ground, plus a dynamic 20 mm, 20 g cube. The cube starts 2 mm above the table
and settles under gravity. The table and cube share one configuration:
`arctos_gazebo/config/manipulation.yaml`. Override with
`scene_config:=/absolute/path/to/custom.yaml`. The empty-world demo is unchanged.

The table is mirrored into MoveIt from configuration; the cube's measured
Gazebo world pose (`/cube/pose`) updates its MoveIt collision object at 5 Hz.
This is an explicit object synchronizer, not automatic scene discovery or a TF
broadcaster. It must change when attached-object/grasp ownership is introduced.

### User checkpoints

1. Wait for `Manipulation scene ready`. Confirm the red cube rests on the grey
   table and both objects agree between Gazebo and RViz (enable Scene Geometry
   in MotionPlanning if needed). No arm motion starts automatically.
2. In another terminal with the same workspace sourced, from the repository root:

   ```bash
   python3 scripts/test_manipulation.py
   ```

   This checks settling, scene consistency, a valid pre-grasp/approach plan,
   and rejection of a table-intersecting target, **without commanding motion**.
3. To see the motion, start from the initial open-jaw pose and run:

   ```bash
   python3 scripts/test_manipulation.py --execute
   ```

   The arm moves above the cube and lowers straight down, stopping with the
   provisional `grasp_frame` 5 mm above the cube's top. Confirm the open fingers
   point down, remain clear of the table/cube, and both windows agree. The test
   leaves the arm at that endpoint. Relaunch to repeat from the initial pose.

The pre-grasp frame starts 60 mm above the cube centre; `approach_clearance`
and `surface_clearance` are configurable. The mesh-derived `grasp_frame` is at
the distal jaw-pad midpoint, 74 mm along −Y from `tool0`, with local +Z pointing
out of the fingers. It is provisional until checked against the actual kit.
The approach uses `tool0` Cartesian waypoints transformed to that frame,
because the KDL chain is configured with `tool0` as its solver tip.
During pre-grasp transit a path constraint keeps `tool0` at least 100 mm above
the tabletop, avoiding low sweeping motions before the straight approach.

**Scope limit:** insertion around the cube is not accepted yet. The conservative
convex jaw collision shapes in Gazebo can contact a 20 mm cube before MoveIt's
mesh geometry predicts contact. The checkpoint therefore stops above it rather
than allowing contacts or relaxing validation tolerances. Closing, attachment,
lifting and release are not implemented here.

For an isolated automated execution/cleanup:

```bash
python3 scripts/validate_gazebo.py --scene
```

It checks a settled cube against Gazebo feedback, planning-scene geometry,
pre-grasp execution, table collision rejection, a complete Cartesian approach,
executed position/orientation, open jaws and an undisturbed cube. Logs are
`manipulation-launch.log` and `manipulation-checks.log` in the output directory.

## Physics details

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
