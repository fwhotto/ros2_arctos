# Voron V0.2 Gazebo printer model

A Gazebo Harmonic model of a stock Voron V0.2R1 with a passive hinged front door,
for the printer-tending task in `CLAUDE_VORON_HANDOFF.md`. Built after the user
approved the split, limits and bed pose in `VORON_MEASUREMENTS.md`.

| File | Role |
| --- | --- |
| `scripts/build_voron_sdf.py` | Generates meshes and `model.sdf` from the hash-verified STEP |
| `arctos_gazebo/models/voron_v02/model.sdf` | Generated model, SDF 1.9 (committed) |
| `arctos_gazebo/models/voron_v02/meshes/` | Derived GPL-3.0 visual meshes (generated, git-ignored) |
| `arctos_gazebo/models/voron_v02/NOTICE` | Upstream source and licence of the meshes |
| `arctos_gazebo/worlds/voron_printer.sdf` | Printer-only world: printer and a 20 mm cube on the bed |
| `arctos_gazebo/test/test_voron_model.py` | Joint, inertia, finger-gap, cube-pose and `gz sdf --check` tests |

## Generate

Needs `~/voron-inspection` from `scripts/inspect_voron_cad.py --fetch-only`
(hash-verified download). Run from the repository root, with the CAD skill's
pinned Python, before `colcon build` so the meshes are installed:

```bash
uvx --no-config --managed-python --python 3.13 --from cadgen==0.7.26 python \
  scripts/build_voron_sdf.py
```

About 30 s and 32 MB of STL. `--mesh-tolerance` (mm, default 0.25) trades mesh
size for detail. Edit the generator, not `model.sdf`.

## Run the printer-only checkpoint

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select arctos_gazebo && source install/setup.bash
export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix arctos_gazebo)/share/arctos_gazebo/models:$GZ_SIM_RESOURCE_PATH
gz sim --render-engine ogre $(ros2 pkg prefix arctos_gazebo)/share/arctos_gazebo/worlds/voron_printer.sdf
colcon test --packages-select arctos_gazebo --ctest-args -R voron_model
```

Expected (acceptance step 2 of the handoff):

- the printer stands still on the ground and the cube settles on the bed;
- dragging the door handle (Gazebo's apply-force/torque tool) swings the door
  outward about its right edge and stops at closed (0) and about 160 degrees;
- the door stays roughly where it is released (light damping, no latch force).

## Model summary

- **Frames:** link frames equal the CAD frame (mm to m, +Z up, door at -Y),
  raised 2.07 mm so the feet sit at model z = 0.
- **chassis:** fixed to the world. Frame, panels, base, closed tophat, parked
  gantry and toolhead, Z drive, electronics boxes and the bed at the retrieval
  pose. Mass 5.0 kg from CAD volumes (electronics not included).
- **door:** 0.195 kg, COM and inertia from CAD solids. `door_hinge` is a revolute
  joint about +Z through CAD (116.502, -117.5) mm. Positive opens outward;
  limits 0 to 2.7925 rad.
- **Frames for planning:** `hinge`, `handle_grasp` (bar centre, pinch along X,
  approach +Y), `bed_surface` (world z ≈ 0.111 m), `front_opening`.
- **Collisions:** 65 boxes: frame members, thin panels, base, tophat, gantry and
  toolhead, rails, bed parts, Z motor and leadscrew, electronics, door panel,
  corner clips, and a five-box D-handle that keeps the 15 mm finger gap open.
  Visual meshes are never used for collision.

## Assumptions (all estimates unless stated)

- Stock printer: right hinge, left D-handle, magnetic latches (user checked the CAD).
- Bed lowered 110 mm from the saved pose; the Z carriage would touch the Z
  endstop at 114.4 mm. The drag chain is a fixed box envelope.
- Door limit 160 degrees (CAD clear to 165). Hinge damping 0.002 N m s/rad and
  friction 0.005 N m. No magnetic latch, so the door does not snap shut.
- Densities: acrylic panels 1.19 g/cm³, ABS parts 1.04 g/cm³ × 0.6 fill,
  aluminium 2.70, steel 7.85, magnets 7.5.
- Door-to-chassis contact is not simulated (Gazebo disables self-collision by
  default); the joint limits stop the door.
- Handle and cube friction μ = 1.0, matching the Arctos jaws.
- Cube: 20 mm, 20 g, no bed adhesion.

## Not yet verified

- `gz sdf --check` and a Gazebo load: `gz` is not available on the authoring Mac.
  The bundled `cadgen sdf validate --strict` passes for both files.
- Whether Bullet Featherstone handles the world-fixed chassis and door joint
  without jitter at 1 ms steps.
- Arm placement, reach and collision-free paths to the handle and bed;
  physically opening the door; cube extraction (handoff steps 3–7).
- Relative mesh URIs (`meshes/*.stl`) resolve against the SDF file in Gazebo
  Harmonic. If they do not in your setup, switch them to `model://voron_v02/meshes/`.
