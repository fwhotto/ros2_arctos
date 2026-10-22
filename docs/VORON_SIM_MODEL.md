# Voron V0.2 Gazebo printer model

A Gazebo Harmonic model of a stock Voron V0.2R1 with a passive hinged front door,
for the printer-tending task in `CLAUDE_VORON_HANDOFF.md`. Built after the user
approved the split, limits and bed pose in `VORON_MEASUREMENTS.md`.

| File | Role |
| --- | --- |
| `scripts/generate_voron_meshes.sh` | One command: download and verify the CAD, then write the meshes |
| `scripts/build_voron_sdf.py` | Generates meshes and `model.sdf` from the hash-verified STEP |
| `arctos_gazebo/models/voron_v02/model.sdf` | Generated model, SDF 1.9 (committed) |
| `arctos_gazebo/models/voron_v02/meshes/` | Derived GPL-3.0 visual meshes (generated, git-ignored) |
| `arctos_gazebo/models/voron_v02/NOTICE` | Upstream source and licence of the meshes |
| `arctos_gazebo/worlds/voron_printer.sdf` | Printer-only world: printer and a 20 mm cube on the bed |
| `arctos_gazebo/test/test_voron_model.py` | Joint, inertia, finger-gap, cube-pose and `gz sdf --check` tests |

## Generate the meshes (VM or any Linux/macOS machine)

The meshes are not in Git, so every checkout generates them once, **before**
`colcon build` (the build copies whatever is in `models/` into `install/`).
Without them Gazebo still loads the collision boxes, but the printer is invisible.

### 1. Install uv (once)

`uv` runs the CAD tool in its own Python 3.13, separate from ROS's Python. Use the
[official installer](https://docs.astral.sh/uv/getting-started/installation/),
then open a new terminal and check it:

```bash
uv --version
```

### 2. Generate

From the repository root:

```bash
scripts/generate_voron_meshes.sh
```

This:

1. downloads the pinned Voron CAD (32 MB ZIP, 179 MB STEP) into
   `~/voron-inspection` and checks every SHA256 hash (files already there are
   reused, never overwritten);
2. turns off the CAD tool's optional telemetry;
3. writes seven STL files (about 31 MB) into
   `arctos_gazebo/models/voron_v02/meshes/`. `model.sdf` is not changed.

Plan for the first run: uv downloads the CAD runtime (several hundred MB), and
the first import of the 179 MB STEP took about 2.5 minutes and 650 MB of memory
on an 8 GB Mac (it also needed more than 2 minutes on the earlier VM). Give the VM at least 4 GB of RAM and about 2 GB of disk.
Later runs reuse the cache and take under a minute. It is safe to run with a ROS
workspace sourced: the script unsets `PYTHONPATH`.

Options: `VORON_CAD_DIR=/other/dir scripts/generate_voron_meshes.sh` stores the
CAD elsewhere; `scripts/generate_voron_meshes.sh --mesh-tolerance 0.5` makes
lighter meshes (mm chord tolerance, default 0.25) if the VM's GUI is slow.

### 3. Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select arctos_gazebo && source install/setup.bash
ls install/arctos_gazebo/share/arctos_gazebo/models/voron_v02/meshes
```

The last line should list the seven `.stl` files. If you generate the meshes
after building, build again.

### Alternative: copy meshes from another machine

If the VM has no internet access, generate the meshes on another machine and copy
the folder over (they are identical for the same commit), for example:

```bash
scp -r arctos_gazebo/models/voron_v02/meshes <vm-user>@<vm-host>:<repo-on-vm>/arctos_gazebo/models/voron_v02/
```

### Regenerating model.sdf

Only needed after changing `scripts/build_voron_sdf.py` (edit the generator, not
`model.sdf`). This rewrites both the meshes and `model.sdf`:

```bash
uvx --no-config --managed-python --python 3.13 --from cadgen==0.7.26 python \
  scripts/build_voron_sdf.py
```

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
