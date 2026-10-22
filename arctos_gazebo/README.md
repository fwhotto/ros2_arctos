# Arctos Gazebo

Target: ROS 2 Jazzy and Gazebo Harmonic via `gz_ros2_control`.

## Build and launch

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to arctos_gazebo
source install/setup.bash
ros2 launch arctos_gazebo demo.launch.py
```

This starts Gazebo, the arm and gripper controllers, and MoveIt. RViz is off by
default; add `rviz:=true` for interactive planning or debugging. Gazebo owns
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

Launch the demo with `rviz:=true`. Wait for `Planning ground ready` in the terminal and
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
broadcaster. During the grasp cycle it is paused while MoveIt owns an attached
cube, then resumed after detachment to follow the released cube again.

### User checkpoints

1. Wait for `Manipulation scene ready`. Confirm the red cube rests on the grey
   table and both objects agree between Gazebo and RViz (enable Scene Geometry
   in MotionPlanning if needed; launch with `rviz:=true` for this optional comparison).
   No arm motion starts automatically.
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
and `surface_clearance` are configurable. The provisional `grasp_frame` is at
the collision-pad midpoint, 66 mm along −Y from `tool0`, with local +Z pointing
out of the fingers. It is provisional until checked against the actual kit.
The approach uses `tool0` Cartesian waypoints transformed to that frame,
because the KDL chain is configured with `tool0` as its solver tip.
During pre-grasp transit a path constraint keeps `tool0` at least 100 mm above
the tabletop, avoiding low sweeping motions before the straight approach.

The approach-only checkpoint still stops above the cube. The optional grasp
cycle below now inserts the open jaws using shared box-pad collision geometry
instead of the earlier overly conservative convex jaw meshes.

For an isolated automated execution/cleanup:

```bash
python3 scripts/validate_gazebo.py --scene
```

It checks a settled cube against Gazebo feedback, planning-scene geometry,
pre-grasp execution, table collision rejection, a complete Cartesian approach,
executed position/orientation, open jaws and an undisturbed cube. Logs are
`manipulation-launch.log` and `manipulation-checks.log` in the output directory.

## Physical cube grasp checkpoint

Start a fresh manipulation launch (no automatic motion), then in another sourced
terminal, from the repository root:

```bash
python3 scripts/test_manipulation.py --execute --grasp
```

The sequence approaches above the cube, lowers the open fingers around it,
closes, lifts **60 mm**, holds for **three seconds**, lowers back to the table,
opens, and retreats upward. Please confirm:

1. Both jaws surround the cube without pushing it during insertion.
2. The cube follows the jaws upward and stays held during the pause.
3. After opening and retreat, the cube stays on the table in both windows.

The cube is held by simulated **contact and friction**. No Gazebo fixed joint,
teleport or attachment plugin is used. MoveIt attaches its collision object to
`grasp_frame` only for carried-object planning, with the two jaw links as touch
links. `/manipulation/pause_cube_sync` (`std_srvs/SetBool`) prevents world-object
updates from competing with attachment ownership; the script restores measured
world updates after release. Do not run two manipulation scripts concurrently.

For reproducible headless testing with automatic launch/cleanup:

```bash
python3 scripts/validate_gazebo.py --scene --grasp
```

Logs are `grasp-launch.log` and `grasp-checks.log`. Beyond the approach checks,
the test verifies cube position throughout the lifted hold (within 5 mm), open
jaws after release, cube return within 3 mm, and restoration of MoveIt's world
object with no leftover attached cube. This is tested for the default scene;
other sizes/poses may need clearance, closing-travel and planning adjustments.
If the script fails after attachment, it does **not** automatically open the
jaws or attempt recovery. Inspect the logged failure and restart the simulation.

### Collision and contact assumptions

Each jaw now uses two explicit boxes, a pad and a proximal rail, in **both**
MoveIt and Gazebo; the visual meshes and inertias remain unchanged. Pads are
18×16×8 mm, centred at y = −66 mm with a 30 mm initial opening. Jaw friction
coefficients are set to 1.0. These are simplified simulation contact surfaces,
not measured kit dimensions, rubber friction or servo force ratings. The frame
is at the pad centre rather than the earlier distal mesh tip.

For the 20 mm cube, each jaw is commanded 5.5 mm inward: 5 mm nominal contact
plus 0.5 mm positional squeeze. The two-second closing trajectory and idealized
position control are not force control. Do not use this script on the physical
robot; real hardware needs calibrated stroke and appropriate force/current or
stall handling. The physical grasp is validated only for this light cube and
slow vertical lift, not arbitrary objects, lateral accelerations or payloads.

## Two-cube physics demo: transfer, stack and push

**Experimental:** the complete sequence has passed headless validation, but
repeat runs have lost the cube during lift/transport. Reliable grasp retention
is still a blocker; this is not yet a repeatably validated demo. Checks stop on
slippage rather than hiding it with a physics attachment or relaxed tolerances.

Launch the new scene with two 20 mm / 20 g cubes (Gazebo is the only
visualization by default):

```bash
ros2 launch arctos_gazebo stacking.launch.py
```

Wait for `Manipulation scene ready`, then in a second sourced terminal, from the
repository root:

```bash
python3 scripts/test_manipulation.py --execute --push
```

The arm grasps the red cube, lifts it 60 mm, carries it 40 mm sideways, and
places it on the blue cube. After opening and retreating, both dynamic cubes
must stand unsupported for three seconds. The open gripper then pushes the
upper cube sideways; it must fall and settle on the table. Gravity, friction
and contact determine the result—there are no Gazebo attachments or pose edits.

For smaller checkpoints, use **one** of these instead, with a **fresh launch
for every run**:

```bash
python3 scripts/test_manipulation.py --execute --transfer
python3 scripts/test_manipulation.py --execute --stack
```

`--transfer` places the red cube 40 mm to the opposite side on the table;
`--stack` leaves the two-cube tower standing. These stages currently require
20 mm cubes. If overriding `scene_config` in the launch, pass the same YAML
with `--scene-config` to the script. Do not run scripts concurrently or use
them on physical hardware. On failure, inspect the simulation and restart;
there is no automatic recovery or release.

The stacking scene gives the blocks an explicit rough-surface friction
coefficient of 2.0; jaw friction remains 1.0 and positional squeeze remains
0.5 mm. These are idealized simulation contact parameters, not measured material
properties or a hardware force/payload rating. The original scene is unchanged.

Isolated headless checks launch and clean up their own simulation:

```bash
python3 scripts/validate_gazebo.py --transfer
python3 scripts/validate_gazebo.py --stack
python3 scripts/validate_gazebo.py --push
```

Logs default to `/tmp/opencode/arctos-gazebo-runtime/{transfer,stack,push}-*.log`.
Checks use fresh measured Gazebo poses, not just trajectory completion, and
verify planning attachment cleanup after release. During the deliberate push,
only upper-cube/jaw collision pairs are temporarily allowed in MoveIt; table,
lower-cube and other robot collision checks remain active. The prior collision
matrix is restored afterward, including on a failed push.

RViz is optional (`rviz:=true`) for viewing planned paths, collision geometry
or TF. Disabling it does **not** disable MoveIt or measured-object collision
scene synchronization. Both cubes are synchronized independently; only the
carried red cube's updates pause while it is attached for planning.

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
physics round-off; URDF/control travel remains 0–15 mm. Arbitrary self-contact,
real actuator forces and hardware payload capacity are not validated here.

Evidence and remaining checkpoints: [Gazebo validation](../docs/GAZEBO_VALIDATION.md).

## Voron V0.2 printer model

`models/voron_v02` and `worlds/voron_printer.sdf` add a Voron V0.2 with a
passive hinged door for printer tending. Run `scripts/generate_voron_meshes.sh` before building
and set `GZ_SIM_RESOURCE_PATH`; see [the model notes](../docs/VORON_SIM_MODEL.md).
The arm is not yet placed in that world.
