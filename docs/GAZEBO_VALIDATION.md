# Gazebo integration validation — 2026-10-16

## Two-cube physics demo — 2026-10-22

The new `stacking.launch.py` uses two 20 mm / 20 g cubes, adding an independently
simulated blue support cube. The original single-cube scene is unchanged.
Both cubes have mass/inertia and separate measured pose streams; neither uses
a Gazebo joint attachment or pose edit. The existing single-cube scene remains
available. Gazebo launches now default to `rviz:=false`; MoveIt and object
collision-scene synchronization remain enabled. Use `rviz:=true` for debugging
or the older interactive RViz checkpoints below.

The execution script provides independent `--transfer`, `--stack` and `--push`
stages (each requires `--execute` and a fresh scene). Transfer checks a 40 mm
sideways carry; stacking checks both cube centres and upright orientations
through a three-second unsupported hold after opening and retreat. The push
allows only the upper cube/jaw collision pairs temporarily, restores the prior
MoveIt collision matrix, and checks that the cube falls onto the table and
settles. Scene generation and collision-matrix scope have unit-test coverage.

Headless full-sequence validation of the initial 20 g variant passed in
`/tmp/opencode/arctos-stacking-runtime` (`push-launch.log`, `push-checks.log`).
The original single-cube grasp/release regression passed in
`/tmp/opencode/arctos-grasp-regression`. One development run exceeded the
existing 5 mm lifted-hold tolerance (5.05 mm lateral displacement) and was
rejected; tolerances were not relaxed and no recovery motion was attempted.
A repeated full-sequence run slipped 14.5 mm vertically during transport and
was also rejected. Slower transport and increased positional squeeze were
tried but did not resolve retention and were not retained; a 10 g trial also
lost the cube. The new scene uses explicit rough-surface block friction of 2.0,
preserving the original contact geometry, jaw friction of 1.0, 0.5 mm squeeze
and motion scaling. These coefficients are simulation assumptions, not measured
material properties. Cartesian endpoints are additionally checked
against measured joint-state FK. This contact grasp is not guaranteed to
succeed on every run.

The final rough-surface configuration completed the full sequence, including
the FK endpoint and simulation-time stability checks, in
`/tmp/opencode/arctos-stacking-rough`. A sequential repeat in
`/tmp/opencode/arctos-stacking-rough-repeat` failed the original lifted-hold
check (7.3 mm lateral slip). A diagnostic post-close offset measurement was
near zero on another failed lift, confirming actual slip rather than merely
imperfect centring; that experimental compensation was not retained.
**Reliable physical grasp retention remains a blocker.** This is an experimental
demo, not a repeatably validated stacking implementation. No failed physics
trial was accepted by changing the slip or final placement tolerances.

The standalone transfer checkpoint for that variant passed in
`/tmp/opencode/arctos-transfer-runtime`. Empty-world arm/gripper motion and
five-second holds passed in `/tmp/opencode/arctos-empty-world-regression`.
Bare-arm motion/holds passed in `/tmp/opencode/arctos-bare-regression`. A later
parallel-load single-cube run in `/tmp/opencode/arctos-grasp-regression-final`
lost its planning-scene synchronizer to the existing two-second pose-freshness
guard and stopped before lifting. That failure was not counted as a pass.

Standard `colcon test --packages-select arctos_gazebo`: **37 pytest cases**
(42 including CTest wrappers), no failures/errors/skips. A single-process
`pytest arctos_gazebo/test` invocation failed the unchanged open-pad collision
test; that suite passed independently and on both standard CTest runs. The
standard runner isolates suites in separate processes.

See [the Gazebo README](../arctos_gazebo/README.md#two-cube-physics-demo-transfer-stack-and-push)
for commands. This remains an idealized, slow, 20 mm cube simulation demo, not
validation of real gripper forces or arbitrary payloads. Visual acceptance is
still a user checkpoint.

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
