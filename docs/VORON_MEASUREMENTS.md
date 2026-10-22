# Voron V0.2 measurement report (inspection checkpoint)

Status: **approved by the user**; the Gazebo model is built (see `VORON_SIM_MODEL.md`).
Original status: measured, awaiting user approval of the component split before any
collision simplification, mesh export or SDF authoring. Follows
`CLAUDE_VORON_HANDOFF.md` §5 and `VORON_CAD_CHECKPOINT.md`.

## Source

| Item | Value |
| --- | --- |
| Upstream | `VoronDesign/Voron-0` @ `a53fc87562fd630c846af38d7de850c894dc3d85` (GPL-3.0) |
| STEP | `V0.2R1 Master Assembly v63.step`, SHA256 `d5a9d40d…b182d61f` (all four handoff hashes re-verified locally) |
| Runtime | `cadgen==0.7.26`, Python 3.13 via uvx, macOS arm64, telemetry off |
| Inventory | 2,478 occurrences, 1,691 leaves (matches the VM); ~147 s import, ~640 MB RSS |
| Selectors | All seven handoff selectors resolved to the expected labels |

Selectors below are revision-scoped. Bounding boxes are envelopes in saved CAD
placement, not clear openings.

## Coordinate convention

CAD frame, **millimetres**: +Z up, origin at the frame's XY centre, the front
(door) face at −Y, +X to the right when facing the door. Feet bottom at
z = −2.07. Scale check: the door panel measures **212.000 × 3.000 × 239.000**,
matching the drawing (212 × 239) and 3 mm panel spec, so the STEP is
normalised to mm. Proposed simulator frame: same axes, translated +2.07 mm in Z
so the feet sit at z = 0, scaled ×0.001 to metres (one conversion, applied once).

## Overall envelope

| Group | Selector | Min (mm) | Max (mm) |
| --- | --- | --- | --- |
| Frame extrusions | `#o1.1.1` | −115, −115, 49.2 | 115, 115, 297.2 |
| Printer without spool/banana/tools | — | ≈ −121.4, −143.6, −2.1 | ≈ 121.4, 132.0, 384.0 |
| Tophat | `#o1.6` | −121.4, −121.4, 292.2 | 121.4, 132.0, 384.0 |
| Spool + holder (excluded) | `#o1.8` | −4.9, −26.9, 26.7 | 207.5, 202.0, 375.3 |

`#o1.9 Banana` (scale prop) and `#o1.10 Swiss Army Jig` (tool lying on the bed)
are not printer geometry and are excluded.

## Door and hinge

| Part | Selector | Min (mm) | Max (mm) |
| --- | --- | --- | --- |
| Door panel | `#o1.5.4.2` | −106.0, −118.0, 58.55 | 106.0, −115.0, 297.55 |
| Door handle | `#o1.5.4.1.1.2` | −100.5, −143.5, 109.1 | −90.5, −118.0, 213.4 |
| Front Bottom/Top Right Hinge (door side) | `#o1.5.4.1.2.1/.2` | 84.7, −121.0, 49.55 | 119.0, −115.0, 298.7 |
| Front Bottom/Top Left Clip (door side) | `#o1.5.4.1.2.3/.4` | −117.5, −119.9, 49.55 | −84.7, −115.0, 297.55 |
| Upper/Lower Hinge Clip (frame side) | `#o1.5.1.2.2.1/.2` | 114.0, −120.0, 49.5 | 119.4, −84.6, 297.5 |
| Hinge pins M3×30, M3×40 BHCS | `#o1.5.4.5.1.2/.1` | 113.4, −120.5, 47.5 | 119.6, −114.5, 274.5 |

**Hinge axis (derived from geometry):** both pin screws have coaxial
cylindrical shanks (r = 1.50 mm, faces `#o1.5.4.5.1.1.f5`, `#o1.5.4.5.1.2.f5`)
and heads (r = 2.85 mm) on the line **x = 116.502, y = −117.500 mm, direction
+Z**. Both door-side hinges and frame-side hinge clips touch along that axis
(distance 0). The hinge is on the **right**; the handle is on the **left**.

**Joint sign and limits (sampled exact solid overlap, threshold 0.05 mm³):**
the door group rotated about +Z through the axis above opens outward for
**positive** angles. At −0.5° it overlaps the front frame extrusions (closed
stop confirmed). From 0° to 165° it is clear of all 129 fixed candidates near
the front. From ~166° the door hinges touch the frame-side hinge clips
(< 0.5 mm³ to 178°, then rapidly increasing: 8 mm³ at 185°, 1,490 mm³ at 250°).
Proposed simulator limits: **0 to 160° (2.79 rad)**, still a documented
approximation (samples at 0, 2, 5, 10, 20, 30, 45, 60, 75, 90, 105, 120, 135, 150,
160, 165–178 every 2°; static poses, not a swept volume).

**What moves:** door panel, handle, 4 handle magnets (`#o1.5.4.4`), both
door-side hinges and both left corner clips. Each clip/hinge sits 0.5 mm from
the panel and 0.002 mm in front of a vertical extrusion, so they are panel
holders that rest against the frame when closed. **Stationary:** both
`Door Latch` parts (x −100…−91, y −115…−100, behind the panel on the left
vertical extrusion) with their screws, nuts and magnets (`#o1.5.4.3`), the
frame-side hinge clips and the pins. Latch retention is magnetic (4 + 4 × 6×3 mm
magnets); no retention force is modelled yet.

## Handle grasp

D-shaped pull bar, 10.0 mm wide in X (x −100.5…−90.5), 104.3 mm tall, feet at
z ≈ 109–123 and 200–213 bonded to the panel, with lightening slots through the
web. At z = 161 the bar spans y −143.5…−133.0: a **15.0 mm finger gap** between
the panel (y = −118) and the bar, 10.5 mm bar depth. The bar's outer side is
5.5 mm from the panel's left edge (x = −106); beyond that is open air in front
of the left side panel. Arctos jaws (from `arctos_description/meshes`) are
≈ 32.5 mm across their closing axis, ≈ 45.7 mm long, with a 30 mm nominal
opening, so a pinch across the 10 mm bar width fits the stroke. Reach,
wrist clearance and approach path are **not** yet checked.

## Opening and interior

- Front frame window: x **−100…100** (200 mm clear between vertical extrusions),
  bottom extrusion top at **z = 64.2**; no front-top extrusion below the
  side rails. Overhead obstructions: Y rails and rail stops at z 233–250, gantry
  `#o1.2` from z 197 upward, parked toolhead `#o1.2.5` at x −29…29,
  y −47…34, z 223.1…328.3.
- Latch protrusions inside the opening: x −100…−91, z 108.7–126.7 and 195.7–213.7.
- Door handle sweeps a radius of ≈ 218 mm about the hinge (outer handle tip).

## Bed

| Item | Selector | Min (mm) | Max (mm) |
| --- | --- | --- | --- |
| Bed assembly | `#o1.3.1` | −65.1, −101.9, 49.1 | 82.1, 62.5, 228.6 |
| Build plate stack | `#o1.3.1.1` | −60.0, −101.9, 205.3 | 60.0, 28.1, 219.0 |
| Flex plate (print surface) | `#o1.3.1.1.4` | −60.0, −101.9, 217.9 | 60.0, 28.1, **219.0** |
| Z rail carriages | `#o1.1.3.2.*.1.1` | z 180.6 | z 212.2 |
| Z rails | `#o1.1.3.2.*.1.2` | z 66.2 | z 216.2 |

Saved pose: bed raised to the nozzle (flex plate top z = 219.0, toolhead bottom
z = 223.1). The bed carriage can descend ≈ 114.4 mm before reaching the rail
ends (rail limit, not a firmware or endstop limit), so the **lowest plate
surface ≈ z 104.6**. The space under the bed in front is clear to the bottom
extrusion; electronics are on the rear wall. Proposed retrieval pose: plate
lowered to its rail-limited bottom (≈ 104.6 mm) unless you prefer another
height.

## Proposed component split (review STEP)

`scripts/voron_review_split.py` writes `~/voron-inspection/voron_split_review.step`
(small hardware omitted, derived GPL-3.0 geometry, not committed):

| Group | Sources | Simulation treatment after approval |
| --- | --- | --- |
| `door` | panel, handle, handle magnets, door hinges, left clips | dynamic link, passive revolute about the hinge axis |
| `chassis` | extrusions, printed frame parts, skirts/feet, side/opaque panels, frame-side hinge clips, latches, pins, electronics | anchored; open front window preserved |
| `bed` | `#o1.3.1` | fixed at retrieval pose |
| `z_drive` | Z motor, Z rails | fixed |
| `gantry_toolhead` | `#o1.2`, Y rails | fixed parked obstacle |
| `tophat` | `#o1.6` | closed, fixed |

## Unresolved / needs the user

1. Does your printer match stock: hinge on the right, this D-handle on the left,
   magnetic latches, V0.2R1 frame? A front photo would settle it.
2. Door opening limit 0–160° acceptable, and the hinge friction/damping value to
   assume (no evidence yet; proposed small viscous damping as an estimate).
3. Bed retrieval height: rail-limited bottom (~104.6 mm) or another value.
4. Panel material (acrylic vs polycarbonate) and printed-part density for the
   door inertia; otherwise documented estimates will be used.
5. Snapshot of the full master STEP failed in this runtime (`Unextractable`
   surface on one face); the grouped review STEP renders cleanly. Not a CAD
   validity verdict.

Not done: collision simplification, mesh export, SDF, inertials, reach checks,
any Gazebo run (`gz` is not installed on this Mac).
