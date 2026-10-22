# Voron CAD inspection checkpoint

This checkpoint prepares a component split for review, not a Gazebo model.
Original CAD is kept unchanged. No collision simplification or printer redesign
is authorized by this inspection step.

## Reproduce acquisition and inspection

The official `VoronDesign/Voron-0` source is pinned to commit
`a53fc87562fd630c846af38d7de850c894dc3d85`. Use the CAD skill's isolated runtime:

```bash
uvx --no-config --managed-python --python 3.13 --from cadgen==0.7.26 python \
  scripts/inspect_voron_cad.py --fetch
```

Default download/report directory: `/tmp/opencode/voron-inspection`. Override it
with `--directory`. Downloads are checked against pinned SHA256 hashes; existing
files with different hashes are rejected, never replaced. Only the expected
STEP archive member is extracted. No ROS dependencies are installed or changed.

`--fetch-only` uses only Python's standard library and skips CAD loading. Use
`--measure '#o...'` (repeatable) to measure explicit selections after reviewing
the inventory. Reports include revision-scoped selectors, labels and hierarchy.
Do not reuse selectors against another upstream revision without reinspection.

The downloaded sources are GPL-3.0; the upstream `LICENSE` accompanies them.
No upstream geometry is vendored or relicensed by this checkpoint.

## Source evidence

| Input | Size | SHA256 |
| --- | --- | --- |
| STEP ZIP | 32,088,462 bytes | `a1cf81917dbeb55c13b23a490763297f85e4aeef5c0f2c45274a4969b5103b25` |
| Extracted `V0.2R1 Master Assembly v63.step` | 179,234,641 bytes | `d5a9d40db82c55eee20203608a5a4e784b54b87ae339861af577c5aeb182d61f` |
| `Front_Door_v0.2.PDF` | 19,699 bytes | `bf1134bacfae8655bad4520cf858acddb3b976d2fd1fb6be3b8dac729138f691` |

The official front-door drawing explicitly labels **212 × 239 mm**, agreeing
with the panel DXF. This is not the access-opening measurement. STEP text includes
named `Door Assembly`, `Door Panel`, `Door Handle`, `Door Latch Assembly`, hinge
components and `Bed Assembly`. Labels establish candidates, not joint axes or
physical clearances. The STEP includes both millimetre and metre unit contexts;
all normalized measurements must be checked before export, not inferred from
one unit declaration.

A lightweight read of the upstream STEP assembly relationships found 2,456
usage records. In the source hierarchy:

- `Door Assembly` contains `Printed Parts (4)`, `Door Panel`, `Latch Hardware`,
  `Handle Hardware` and `Hinge Hardware`.
- `Door Latch Assembly` includes the handle and two latch occurrences.
- `Door Clips` includes right-side hinge parts and left-side clips. This is
  evidence for the stock arrangement, not verification of the user's printer.
- `Bed Assembly` contains `Build Plate`, hardware, printed parts, extrusions and
  a drag chain. Keep these obstacles in scope rather than exporting only the
  top plate.

These are source relationships, not a geometry-engine validation or a claim
that all nested parts belong on the same moving link.

The CAD-kernel inventory subsequently completed successfully: **2,478
occurrences, 1,691 leaf occurrences**. Report:
`/tmp/opencode/voron-inspection/inventory.json` (VM-local, not committed).
Key selectors are `#o1.3.1` (bed assembly), `#o1.5.4` (door assembly),
`#o1.5.4.2` (door panel), and `#o1.5.4.1.1.2` (door handle).
This establishes selectable CAD structure, not measured clearances or completed
visual review. Full import needed more than the initial two-minute allowance.

## Proposed component ownership

| Group | Intended contents | Simulation treatment after approval |
| --- | --- | --- |
| Chassis | Frame, enclosure, stationary hinge/latch hardware, electronics covers | Anchored, with an open interior |
| Front door | Panel, handle, door-side clips and hardware | One rigid dynamic link, one passive hinge |
| Bed | Plate, heater, carriage/support obstacles | Fixed at a documented retrieval height initially |
| Parked motion system | Gantry, toolhead and nearby obstruction geometry | Stationary obstacles initially |
| Tophat | Top enclosure and its separate hinges | Closed/fixed initially; not the front-door joint |

The hinge's parent and child parts must be identified geometrically, not assigned
from names alone. In particular, the latch assembly may contain parts on both
the stationary frame and moving door. The final split depends on CAD inspection
and comparison with the user's printer.

## Approval boundary

Before exporting simulation geometry, review the intact CAD, component split,
door axis, handle fit and bed access. Record any differences from the user's
printer. Missing or ambiguous geometry triggers a controlled modeling discussion;
it is not replaced by invented dimensions. See `PRINTER_TENDING_RESEARCH.md` for
simulation assumptions and the later integration/validation sequence.

## Acquisition tests

```bash
python3 scripts/test_inspect_voron_cad.py
```

Offline tests cover checksum rejection without overwriting changed files,
exact-member archive extraction and reuse without network access. They do not
test CAD import, mesh export or Gazebo physics.

## Local measurement checkpoint

Measured selectors, hinge axis, door limits, handle, opening and bed datum are
in `VORON_MEASUREMENTS.md`. `scripts/voron_review_split.py` exports the proposed
component split as a review STEP outside the repository. Approval is still
required before simulator geometry.
