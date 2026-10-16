# Gazebo integration validation — 2026-10-16

## Automated checkpoints completed

1. Fresh Jazzy dependency/build validation: four packages; **34 pytest cases**
   passed (39 including five CTest wrappers), zero errors/failures/skips.
   Added checks expand the actual Gazebo wrapper and confirm the SDF world
   anchor, six arm joints, optional jaws and retained mimic relationship.
2. Headless Gazebo with gripper: MoveIt planned/executed `demo → closed → open
   → home`. Every target held for five seconds. Arm drift stayed below 0.01 rad
   and jaw drift below 0.1 mm; fresh independent Gazebo joint states agreed with
   ros2_control feedback within those tolerances. Clock advanced during holds;
   the ground was present in the planning scene.
3. Headless bare-arm Gazebo: `demo → home` with the same hold/feedback checks.
4. Mock backend regression after shared-interface refactor: `demo → closed →
   open → home` planned and executed successfully.

Evidence: `/tmp/opencode/arctos-gazebo-final` (isolated build/install/test results,
`gripper-launch.log`, `gripper-checks.log`, `bare-launch.log`, `bare-checks.log`,
`mock-regression.log`). Build transcript: `/tmp/opencode/arctos-gazebo-build.log`.
Runtime used ROS domains 86 and 87; all test launches were stopped afterward.

Installed `gz_ros2_control`: 1.2.20; `ros2_control`: 4.48.0; Gazebo Harmonic.
The simulator uses Bullet Featherstone; DART failed the mimic feedback check.
An earlier attempt also exposed endpoint floating-point noise rejected by
MoveIt's strict bounds adapter. Gazebo-only planning bounds allow 1 µm beyond
each jaw endpoint; physical/control travel bounds are unchanged.

## User confirmation checkpoints pending

### VM renderer follow-up

The user's interactive run exposed OpenGL 2.1 in the VM, causing the default
Ogre2 GUI (requires OpenGL 3.3) to crash and shut down the launch. Added
`render_engine:=ogre` to select the compatible legacy GUI renderer.
A GUI-enabled run on DISPLAY=:0 with that option stayed alive through all four
arm/gripper targets and five-second holds, with no rendering errors in
`/tmp/opencode/arctos-gazebo-final/ogre-gui.log`. This verifies GUI startup and
continued execution; visual model acceptance still requires user confirmation.

Follow [the Gazebo README](../arctos_gazebo/README.md):

- Model appearance, latest-kit wrist mounting and open-jaw alignment.
- Stable anchored model and agreement of arm motion between Gazebo and RViz.
- Opposing, symmetric gripper opening/closing in both windows.

GUI rendering was not validated in this headless session. Free-space tracking
does not validate object contacts, gripper forces, self-contact physics or
hardware dynamics. Object-contact/grasping work follows visual acceptance.
The [closed-loop baseline](CLOSED_LOOP_BASELINE.md) records revision-dependent
assumptions and configuration adaptation points for the collaborator handoff.
