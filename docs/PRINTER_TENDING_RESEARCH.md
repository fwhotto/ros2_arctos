# Printer-tending simulation: sources and geometry checkpoint

Research date: 2026-10-22. Status: research only; no printer model has been
imported, generated, or validated in Gazebo. No printer motion has been tested.

## Agreed scope

- Arctos opens a hinged front door, retrieves a cube from the print bed, and
  places it outside the printer.
- Target printers: Bambu Lab P2S and Voron V0.2. The user confirmed a hinged
  front door for the Voron; its exact build variant remains unverified.
- Do not create replacement printer geometry without a separate, controlled
  modeling step using tools agreed with the user.

## Verified specifications and available CAD

### Bambu Lab P2S

Manufacturer specifications:
<https://bambulab.com/en/p2s/specs>

| Item | Published value |
| --- | --- |
| Physical dimensions | 392 × 406 × 478 mm |
| Build volume (W × D × H) | 256 × 256 × 256 mm |
| Net weight | 14.9 kg |
| Build plate | Flexible steel; textured PEI included |
| Door sensor | Supported |

The published envelope and build volume do **not** establish the usable front
opening, bed position inside the enclosure, handle geometry, or hinge axis.
Do not substitute the build volume for measured bed or opening dimensions.

Community CAD candidates (not manufacturer-certified):

1. [GrabCAD: Bambulab P2S](https://grabcad.com/library/bambulab-p2s-1).
   Search results identify a reverse-modeled printer for accessory design.
   The fetched page did not expose its file inventory or full description.
   Assembly structure, interior accuracy, download access and reuse terms are
   unverified. Do not treat this as a simulation-ready asset.
2. [MakerWorld: Bambu Lab P2S – STEP/CAD Model](https://makerworld.com/en/models/2590690-bambu-lab-p2s-step-cad-model).
   Search results describe a reverse-engineered body reference model and list
   a Standard Digital File License. Direct page access returned HTTP 403.
   Interior completeness, separate door components, dimensional accuracy and
   exact license permissions remain unverified. Do not redistribute it or
   derived meshes without checking permission.

No suitable, openly redistributable P2S assembly has been verified in this
research. Do not silently substitute a P1S or X1 model.

### Voron V0.2

Official sources:

- [Voron V0.2 page](https://vorondesign.com/voron0.2)
- [VoronDesign/Voron-0](https://github.com/VoronDesign/Voron-0)
- [CAD directory](https://github.com/VoronDesign/Voron-0/tree/Voron0.2r1/CAD)
- [Panel DXFs](https://github.com/VoronDesign/Voron-0/tree/Voron0.2r1/DXF)

The official repository documents a 120 × 120 × 120 mm build volume, 3 mm
panels for V0.2, and V0.2r1 as the current revision on the inspected branch.
Its hinged **tophat** description is not evidence of the user's front-door
hinge configuration. V0.2 also moved the bed 3 mm rearward relative to V0.1.

GitHub's contents API confirmed:

| File | Size (bytes) |
| --- | --- |
| `CAD/V0.2R1_Master_Assembly_STEP.zip` | 32,088,462 |
| `CAD/V0.2R1_Master_Assembly.f3d` | 51,129,981 |
| `DXF/Front_Panel.dxf` | 2,216 |

Reference commit inspected: `a53fc87562fd630c846af38d7de850c894dc3d85`.

[Pinned STEP archive](https://raw.githubusercontent.com/VoronDesign/Voron-0/a53fc87562fd630c846af38d7de850c894dc3d85/CAD/V0.2R1_Master_Assembly_STEP.zip)

The repository identifies its license as GPL-3.0. Preserve applicable license,
source and attribution obligations for imported or converted assets; do not
label them with this repository's Apache-2.0 license. Keep third-party assets
and their notices distinct if imported later.

The assembly is a strong starting point, not yet an inspected or converted
Gazebo asset. Derive the exterior envelope, bed location, front opening and
hinge transforms from CAD rather than guessing them. Compare the stock assembly
with the user's printer before claiming a match.

## Proposed simulation assumptions — not measured printer properties

| ID | Proposed assumption | Limit / verification needed |
| --- | --- | --- |
| A1 | Start with a loose 20 mm cube of mass 20 g, matching the existing grasp test. | Simulation test object, not a measured printed part; no adhesion or removal force. |
| A2 | Printing has finished; bed and nozzle are cool. | No thermal simulation or physical-printer safety interlocks implemented. |
| A3 | Toolhead is parked clear and bed is stationary at an accessible retrieval height. | Select and document actual poses only after geometry and reach checks; do not assume the print-finish pose is reachable. |
| A4 | Printer chassis is rigidly anchored in simulation. | Does not demonstrate that the real printer cannot slide or tip during door opening. |
| A5 | A single front door rotates about a fixed hinge axis. | User confirmed a hinged Voron door; side, limits, handle and axis must be checked for each printer. |
| A6 | Begin without latch/magnetic retention force. | Explicit initial simplification; real hinge friction, door inertia and retention forces are unknown. |
| A7 | Reuse the existing Arctos gripper initially. | Handle fit, wrist clearance, reach and transport retention are not established. |
| A8 | Use one printer per initial test scene. | Workcell position, support heights and tray location are configurable design choices, not a reconstruction of the user's room. |

None of these assumptions authorizes invented printer dimensions. Any later
geometric approximation needs a recorded source, uncertainty and approval.

## Controlled CAD checkpoint

Recommend starting with the official Voron assembly. Before writing a printer
URDF/SDF or exporting meshes:

1. Agree on the specialized CAD tools with the user. Inspect the existing STEP
   assembly first; a complete new model should not be necessary for stock Voron.
2. Confirm the user's door/handle modifications and hinge side, preferably with
   a front photograph. Resolve V0.2 versus V0.2r1 differences that affect access.
3. Identify separate chassis, door, handle, bed and parked-toolhead components.
   Record units, coordinate frames, hinge origin/axis and bed transforms.
4. Review a dimensioned view and proposed component split with the user.
5. With approval, export visual meshes and simplified collision geometry.
   Retain the actual opening: a single solid box for the chassis would prevent
   the robot entering. Door and handle must move together about the correct axis.
6. Record provenance, asset checksums, conversion settings, licenses, measured
   values and any estimated inertia/contact parameters alongside the assets.

For the P2S, first obtain and inspect a candidate CAD file under suitable terms.
If it lacks usable interiors, hinge or handle geometry, stop and ask the user
to arrange a controlled modeling/measurement session rather than fabricate it.

## Integration and acceptance after geometry approval

- Extend `arctos_gazebo` without changing the existing manipulation demo.
- Validate arm reach for the handle, door swing and bed approach before choosing
  a workcell layout. A collision-free endpoint alone is not a complete task.
- Mirror measured door and cube poses into MoveIt's planning scene. Maintain
  explicit ownership when the cube is attached for carried-object planning.
- First test cube retrieval with the door already open; then add robot-driven
  handle grasp and hinge-following opening. A commanded/animated door is only
  a debugging mode, not proof that the arm physically opened it.
- Test door clearance, cube insertion/grasp, lift, outward extraction, placement
  and retreat. Verify measured object motion and no prohibited contacts.
- Improve and repeat-test physical grasp retention before claiming reliability:
  `arctos_gazebo/README.md` already flags losses during cube transport.
- Keep initial operation simulation-only, with no connection to physical robot
  or printer control. Do not claim real-printer readiness from this demo.
