# Claude handoff: prepare Voron V0.2 CAD for Gazebo

Give Claude this file in a local checkout of `ros2_arctos` and ask it to continue
the CAD inspection checkpoint. This transfers research and instructions, not
the VM's runtime, caches or downloaded CAD. Read the checkout's `AGENTS.md`, if
present, before working. Preserve unrelated local changes.

## User goal and authorization boundary

Prepare a Gazebo simulation in which an Arctos robot arm opens the hinged front
door of a Voron V0.2, grasps a loose cube on the print bed, removes it, and places
it outside the printer. A Bambu Lab P2S is a later target; prioritize the Voron.

The user confirmed a hinged Voron front door. Their hinge side, handle, build
revision and modifications have not been verified.

**Current authorized step:** inspect the official assembly, measure relevant
geometry, and present the proposed component split for approval. Do not proceed
straight to a finished simulator model. Do not invent printer dimensions or
silently substitute unrelated geometry. If new geometry is needed, explain what
is missing and agree on a controlled modeling step with the user. Record all
assumptions clearly.

## 1. Install text-to-cad locally

Upstream: <https://github.com/earthtojake/text-to-cad>.

Read its current installation instructions before executing commands; they may
have changed since this handoff. Inspect existing installations first, and avoid
installing both plugin and standalone skill copies in the same agent.

At the time of the VM setup, upstream recommended the following for Claude Code:

```bash
uv --version
claude plugin marketplace add earthtojake/text-to-cad#latest
claude plugin install text-to-cad@earthtojake
```

If `uv` is missing, use its official installation instructions:
<https://docs.astral.sh/uv/getting-started/installation/>. Do not modify the ROS
Python environment to install CAD dependencies.

Restart Claude Code after installation. Load the installed CAD skill and read
its inspection, export and snapshot references before using CAD commands. Load
the SDF skill when simulator model authoring is actually approved.

**Version discipline:** the VM installed the released skills from the `latest`
branch with `cadgen==0.7.26`, managed Python 3.13, and a working Open CASCADE
kernel. A new installation may pin a newer version. Follow the installed skill's
runtime command and verify it with `cadgen doctor <installed-cad-skill-directory>`.
Do not guess the skill directory or mix a newer skill's interface with an older
runtime. Locate it from Claude's installed plugin metadata/files.

For the exact runtime used in the VM, the following is the fully qualified
command; a shell alias named `cadgen` was **not** installed:

```bash
uvx --no-config --managed-python --python 3.13 --from cadgen==0.7.26 cadgen doctor
uvx --no-config --managed-python --python 3.13 --from cadgen==0.7.26 cadgen telemetry off
```

The VM disabled optional usage/crash telemetry. Preserve that preference locally:
use the installed runtime's telemetry-off command before normal use, and set
`CADGEN_TELEMETRY=0` for the first invocation if supported by the current release.
Do not copy VM credentials or configuration to the local machine.

For Claude Desktop rather than Claude Code, use upstream's current Desktop MCP
instructions instead of these plugin commands. Skill installation and MCP
configuration are client-specific.

## 2. Read the existing project knowledge

These files should be transferred with this handoff, or available in the checkout:

- `docs/PRINTER_TENDING_RESEARCH.md`: sources, assumptions, geometry approval
  boundary, later integration plan and P2S research.
- `docs/VORON_CAD_CHECKPOINT.md`: acquisition hashes, source hierarchy evidence,
  proposed component ownership and reproduction commands.
- `scripts/inspect_voron_cad.py`: checksum-verified download and CAD inventory.
- `scripts/test_inspect_voron_cad.py`: offline acquisition tests.
- `arctos_gazebo/README.md`: existing simulation and its limitations.

These additions were **not committed by the agent**. If transferring through Git,
make sure they are actually included; merely cloning the original upstream
repository will not fetch uncommitted VM files. If only this Markdown file was
copied, recover the other files from the VM before assuming those commands exist.

## 3. Acquire the authoritative CAD

Use the official Voron repository, pinned to:

```text
Repository: https://github.com/VoronDesign/Voron-0
Revision:   a53fc87562fd630c846af38d7de850c894dc3d85
Branch at research time: Voron0.2r1
License: GPL-3.0 (preserve upstream license and applicable obligations)
```

Source files, relative to that repository:

| Path | Bytes | SHA256 |
| --- | --- | --- |
| `CAD/V0.2R1_Master_Assembly_STEP.zip` | 32,088,462 | `a1cf81917dbeb55c13b23a490763297f85e4aeef5c0f2c45274a4969b5103b25` |
| ZIP member `V0.2R1 Master Assembly v63.step` | 179,234,641 | `d5a9d40db82c55eee20203608a5a4e784b54b87ae339861af577c5aeb182d61f` |
| `Drawings/Front_Door_v0.2.PDF` | 19,699 | `bf1134bacfae8655bad4520cf858acddb3b976d2fd1fb6be3b8dac729138f691` |
| `LICENSE` | 35,149 | `3972dc9744f6499f0f9b2dbf76696f2ae7ad8af9b23dde66d6af86c9dfb36986` |

Pinned ZIP download:
<https://raw.githubusercontent.com/VoronDesign/Voron-0/a53fc87562fd630c846af38d7de850c894dc3d85/CAD/V0.2R1_Master_Assembly_STEP.zip>.

Other assets use the same raw URL prefix plus their repository path. Do not
relicense converted Voron assets as Apache-2.0 just because the Arctos repository
uses that license. Keep third-party notices distinct.

### Reproduce with the transferred script

Run from the checkout root. Choose a local inspection directory; the examples
below use a disposable directory under the user's home rather than the VM path:

```bash
python3 scripts/test_inspect_voron_cad.py
python3 scripts/inspect_voron_cad.py --fetch-only --directory "$HOME/voron-inspection"
uvx --no-config --managed-python --python 3.13 --from cadgen==0.7.26 python \
  scripts/inspect_voron_cad.py --directory "$HOME/voron-inspection"
```

Use the installed skill's equivalent pinned Python command if its version differs.
The script creates its directory, verifies hashes, and never overwrites changed
source CAD. Default output is `inventory.json` in that directory. Choose another
`--output` to preserve an earlier report.

**Runtime warning:** the original 179 MB STEP exceeded a two-minute import limit
in the VM. The second import completed successfully, reporting **2,478
occurrences and 1,691 leaf occurrences**. This confirms CAD inventory loading,
not completed measurements or visual inspection. Allow a
longer task timeout locally, monitor resource use if needed, and avoid launching
multiple imports/tessellations of the full assembly concurrently. A timeout is
not evidence that the STEP is invalid.

## 4. What is actually known

Verified from official documentation/drawing/source text:

- Build volume: **120 × 120 × 120 mm**.
- V0.2 documentation specifies **3 mm** panels.
- Front-door drawing explicitly gives **212 × 239 mm**, consistent with
  `DXF/Front_Panel.dxf`. This is the panel outline, **not** the clear opening.
- The stock STEP has named `Door Assembly`, `Door Panel`, `Door Handle`,
  `Door Latch Assembly`, hinge parts and `Bed Assembly`.
- A lightweight parse found 2,456 source assembly-usage relationships. This was
  a text/hierarchy check, not a geometry-engine validation.
- `Door Assembly` contains `Printed Parts (4)`, `Door Panel`, `Latch Hardware`,
  `Handle Hardware` and `Hinge Hardware`.
- `Door Latch Assembly` includes a handle and two latch occurrences. Do not
  assume every nested latch part moves with the door without checking geometry.
- `Door Clips` includes right-side hinge parts and left-side clips. This supports
  a stock arrangement, not the configuration of the user's actual printer.
- `Bed Assembly` includes the build plate, hardware, printed parts, extrusions
  and drag chain. Consider these obstacles, not only the plate surface.
- The tophat has separate hinges; they are not the front-door joint.
- STEP text includes millimetre and metre unit contexts. The CAD skill documents
  normalized STEP measurements in millimetres; verify output scale with the
  known panel dimensions before exporting.

**Not yet measured or validated:** complete exterior envelope, usable opening,
hinge origin/axis and sign, door travel limits, handle grasp clearance, bed pose,
retrieval height, toolhead parking pose, door mass/inertia, arm reach, physical
door opening, cube extraction, or a Gazebo-ready printer model.

## 5. Complete the inspection checkpoint

Use the installed CAD skill's inspection API. With the VM's version, the relevant
pattern is:

```python
from cadgen import read_scene

scene = read_scene("/absolute/local/path/V0.2R1 Master Assembly v63.step")
for occurrence in scene.leaves():
    print(occurrence.ref, occurrence.label, occurrence.prototype_id)
```

The transferred script records hierarchy as well as leaf occurrences. Inspect
that inventory first, then select exact refs. Its `--measure '#o...'` option is
repeatable and records bounding boxes, solid counts and volume. Bounding boxes
are envelopes, not measurements of holes or clear access paths.

The completed VM inventory identified these selectors for the pinned STEP:

| Selector | Label |
| --- | --- |
| `#o1.3.1` | `Bed Assembly:1` |
| `#o1.3.1.1` | `Build Plate:1` (a subassembly, not just one plate solid) |
| `#o1.5.4` | `Door Assembly:1` |
| `#o1.5.4.1.1` | `Door Latch Assembly:1` |
| `#o1.5.4.1.1.2` | `Door Handle:1` |
| `#o1.5.4.1.2` | `Door Clips:1` |
| `#o1.5.4.2` | `Door Panel:1` |

Verify these labels against the locally loaded inventory before measuring.

Measure geometry that actually defines the task:

1. Door panel and handle; confirm scale against the drawing.
2. Coaxial hinge features; derive the rotation axis from those features, not from
   apparent orientation in a screenshot.
3. Door sweep and frame clearance; determine which hinge/latch hardware moves.
4. Handle finger gap, width and surrounding wrist clearance.
5. Clear front opening and obstacles along an extraction corridor.
6. Bed surface datum and plausible retrieval heights; gantry/toolhead clearance.

Save a measurement report containing source hash, exact selectors, coordinate
conventions, units, derived transforms and unresolved issues. Exact selectors
are revision-scoped, not permanent feature identities.

Show the user the intact CAD and proposed component split with the installed
skill's viewer tools. If only the CLI viewer is available, run its documented
detached localhost command and use the **returned** URL; do not reuse the VM's
viewer port. The VM's `127.0.0.1:3245` link is not a transferable asset.

Prepare useful front/isometric and internal or component-focused snapshots and
read them yourself. Follow the installed skill's current snapshot schema. Avoid
retessellating every fastener unnecessarily on a memory-constrained machine.

**Deliverable to the user:** original-model viewer link, measured component split,
open questions/assumptions, and an explicit request for approval before collision
simplification or new printer geometry. A front photo of their Voron would help
confirm the stock door and handle match.

## 6. Agreed direction after geometry approval

This is guidance for the next phase, not evidence it is implemented:

- Prefer a reusable **SDF printer model** derived from the official CAD. Arctos
  remains on its existing URDF/control path; do not rewrite the arm.
- Chassis anchored to the world; door a dynamic rigid link with one passive
  revolute joint; handle rigidly owned by that door link.
- Do **not** mark the entire printer static: the door must move physically.
- Initially keep the bed and parked toolhead fixed at documented retrieval poses,
  and the tophat closed. Full printer-axis simulation is unnecessary initially.
- Use separate visual and collision assets. Preserve the enclosure opening and
  handle finger gaps; one convex hull around either can block the task.
- Prefer simple panels/frame members and primitives or convex pieces for moving
  contact geometry. Do not use every screw or a detailed concave moving mesh as
  the default collision representation.
- Derive mass, COM and inertia from geometry/material evidence or explicitly
  documented estimates; do not freehand inertial tensors.
- Use explicit hinge, handle-grasp, bed-surface and approach frames. Track CAD
  millimetres versus simulator metres with one tested conversion.
- Preserve source hashes, export settings, license notices and assumptions.
- Review simplification overlays before simulator integration.

### Existing simulator constraints

- Target: **ROS 2 Jazzy / Gazebo Harmonic**, using `gz_ros2_control` for Arctos.
- `arctos_gazebo/worlds/empty.sdf` uses **Bullet Featherstone**, 1 ms maximum step,
  and SDF **1.9**. Preserve that baseline unless a change is justified and tested.
  Do not blindly use a newer skill's default SDF version.
- GUI defaults to legacy Ogre for VM compatibility; local graphics capabilities
  may differ, but graphics changes should not silently change physics.
- `arctos_gazebo/scripts/scene_objects.py` currently creates table/cube box models.
- `arctos_gazebo/scripts/planning_ground.py` synchronizes cube poses into MoveIt,
  with a pause mechanism during MoveIt collision attachment. It is **not** an
  articulated-printer synchronizer. Add printer-specific handling rather than
  assuming doors are already supported.
- MoveIt collision attachment is planning bookkeeping; current Gazebo cube
  grasping uses contact/friction, not a fixed attachment or teleportation.
- `arctos_gazebo/README.md` flags unreliable cube retention during transport.
  Improve repeatability before claiming reliable printer tending.

### Initial assumptions, all still explicit approximations

- Loose **20 mm cube, 20 g** (test fixture, not a measured print).
- No bed adhesion, build-plate removal or thermal simulation.
- Printing finished; bed/nozzle cool; toolhead parked clear.
- Printer rigidly anchored; no simulated chassis tipping/sliding initially.
- No latch/magnetic retention force initially; hinge resistance remains to be
  specified as a documented approximation.
- Existing Arctos gripper initially reused; handle fit and reach unproven.
- One printer per initial scene; workcell poses are design choices, not a scan of
  the user's room.

### Later acceptance sequence

1. Validate SDF with the skill and local `gz sdf --check`; check local resources.
2. Printer-only simulation: correct anchoring, hinge motion, limits and contacts.
3. Verify arm reach and collision-free paths for both handle and bed, not only IK
   at isolated endpoints.
4. Retrieve the cube with the door already open.
5. Physically grasp the handle and follow its hinge arc to open the door. An
   animated or position-commanded door is a debugging mode, not physical proof.
6. Combine opening, handle release, extraction, placement and retreat.
7. Repeat fresh-start runs and record slips/prohibited collisions. Measured door
   and cube state must agree with MoveIt's collision scene; stop on stale state.

Do not contact physical printers, start prints, or command the physical robot.
This handoff concerns local CAD preparation and simulation only.

## Status at handoff

- Released text-to-cad skills and isolated CAD runtime installed successfully in VM.
- Official CAD, drawing and license downloaded and hash-verified.
- Two offline acquisition tests passed; inspection CLI help/fetch verified.
- Source hierarchy and drawing researched; findings documented.
- CAD inventory import completed after retry: 2,478 occurrences, 1,691 leaves;
  JSON report at `/tmp/opencode/voron-inspection/inventory.json` in the VM.
- Viewer server started in VM; no completed geometric/snapshot review claimed.
- No printer geometry changed, no mesh conversion, no printer SDF, no physical
  manipulation validation, and no commits made by the agent.
