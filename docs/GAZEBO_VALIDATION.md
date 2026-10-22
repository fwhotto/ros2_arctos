# Gazebo integration validation — 2026-10-16

## Physical grasp follow-up — 2026-10-22

The earlier jaw-insertion blocker is resolved for the default cube by replacing
each jaw's convexified mesh collision with separate pad/rail boxes in the shared
URDF. MoveIt and Gazebo now check the same simplified surfaces. Visual meshes
and inertias are unchanged. `grasp_frame` moves to the pad centre, y = −66 mm
in `tool0` (previously the −74 mm distal tip).

Fresh build: four packages; **39 pytest cases** (45 including six CTest wrappers),
zero errors/failures/skips. The added positive/negative collision test verifies
that a 20 mm cube fits between open pads and intersects closing pads.

Two headless grasp runs passed, with the final control/approach regression checks:

- Open-jaw insertion to cube centre without displacement.
- Closing, then a collision-checked 60 mm vertical lift.
- Fresh independent Gazebo cube positions stay within 5 mm of the lifted target
  for three seconds. No physics attachment, teleport or fixed joint is used.
- Set-down, opening and retreat; cube returns within 3 mm of its settled pose,
  both jaws are open, and MoveIt has a synchronized world cube with no leftover
  attached object.
- Original approach-only scene check and empty-world arm/gripper motions plus
  five-second hold checks still pass.

Evidence: `/tmp/opencode/arctos-grasp-final` and `/tmp/opencode/arctos-grasp-repeat`;
fresh build transcript: `/tmp/opencode/arctos-grasp-build.log`. All validation
launches were stopped. MoveIt attachment is planning-only: the cube pose updater
pauses during attachment and resumes after release.

**User checkpoint pending:** observe insertion, physical lift/hold, and release
in Gazebo/RViz using `python3 scripts/test_manipulation.py --execute --grasp`
against a fresh manipulation launch. This is a slow vertical lift of the default
20 g cube, not validation of arbitrary payloads, dynamic grasps or hardware
forces. Pad dimensions, friction and 0.5 mm positional squeeze are simulation
estimates. The simulation script must not be used on the physical robot.

## Manipulation-scene follow-up — 2026-10-22

Historical approach-only checkpoint; the physical grasp follow-up above
supersedes its jaw-insertion acceptance boundary.

Implemented a configurable static table and dynamic 20 mm / 20 g cube, a
mesh-derived distal `grasp_frame`, and explicit Gazebo-to-MoveIt cube pose
synchronization. The existing empty-world launch remains the default.

Fresh build: four packages; **38 pytest cases** (44 including six CTest
wrappers), zero errors/failures/skips. Tests include shared scene/SDF geometry,
cube inertia, invalid configuration rejection and grasp-frame transform.

Headless manipulation checks passed twice after adding a high-wrist transit
constraint: cube settling, fresh Gazebo/planning-scene pose agreement,
pre-grasp execution, rejection of a table-intersecting target, and complete
collision-checked Cartesian approach. Executed frame position is within 3 mm,
arm endpoint error below 0.01 rad, orientation downward, jaw travel within
0.1 mm of open, and cube displacement below 1 mm. The original empty-world
arm/gripper execution and five-second holds also pass as a regression check.

Evidence: `/tmp/opencode/arctos-manipulation-final` and
`/tmp/opencode/arctos-manipulation-repeat`; build transcript:
`/tmp/opencode/arctos-manipulation-build.log`. Test launches are shut down.

**Acceptance boundary:** the frame stops 5 mm above the cube top. Earlier trials
inserting open jaws around the cube exposed Gazebo convex-hull contact not
predicted by MoveIt's triangle meshes. A low free-space transit also displaced
the cube in one trial; the pre-grasp path now constrains wrist height. These
failures were not accepted by relaxing endpoint or displacement checks.
Jaw insertion/contact geometry refinement remains necessary before grasping.
No object attachment, closing-on-object, lift or release is implemented here.

Visual user checkpoints: table/cube placement agrees in Gazebo and RViz; the
open jaws point down and approach without touching or disturbing the cube.
See the Gazebo README's table/cube checkpoint for launch and execution commands.

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
