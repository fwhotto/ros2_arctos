#!/usr/bin/env python3
"""Export a review STEP of the proposed Voron V0.2 simulation component split.

Run with the CAD skill's pinned uvx Python. Reads the hash-verified master STEP
and writes a grouped, fastener-free copy for visual review only. The output is
derived GPL-3.0 Voron geometry; it stays outside the repository by default.
Selectors are scoped to upstream revision a53fc875 (see VORON_MEASUREMENTS.md).
"""

import argparse
import re
from pathlib import Path

from inspect_voron_cad import STEP_NAME, STEP_SHA256, verified_bytes

# Moving door link: panel, handle and its magnets, door-side hinges and corner clips.
DOOR = ("#o1.5.4.2", "#o1.5.4.1.1.2", "#o1.5.4.1.2", "#o1.5.4.4")
GROUPS = {
    "door": DOOR,
    # Frame-side hinge clips, latches and pivot screws stay with the chassis.
    "chassis": ("#o1.1.1", "#o1.1.4", "#o1.4", "#o1.5.1", "#o1.5.2",
                "#o1.5.4.1.1.1", "#o1.5.4.1.1.3", "#o1.5.4.3", "#o1.5.4.5", "#o1.7"),
    "bed": ("#o1.3.1",),
    "z_drive": ("#o1.3.2", "#o1.1.3.2"),
    "gantry_toolhead": ("#o1.2", "#o1.1.3.1"),
    "tophat": ("#o1.6",),
}
SMALL_HARDWARE = re.compile(r"HCS|Hex ?nut|Nut|Washer|Insert|Magnet|Spring", re.IGNORECASE)
# Face-heavy detail away from the door/bed task (electronics boards, belts, display);
# omitted from the review copy so the viewer stays responsive.
HEAVY_DETAIL = ("#o1.7", "#o1.2.1", "#o1.4.3")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, default=Path.home() / "voron-inspection")
    parser.add_argument("--output", type=Path, help="default DIRECTORY/voron_split_review.step")
    parser.add_argument("--keep-hardware", action="store_true",
                        help="keep screws, nuts, magnets and similar small parts")
    parser.add_argument("--keep-detail", action="store_true",
                        help="keep electronics boards, belts and the display")
    args = parser.parse_args()
    step = args.directory / STEP_NAME
    verified_bytes(step, STEP_SHA256)

    from cadgen import build123d as bd
    from cadgen import read_scene

    scene = read_scene(str(step))
    leaves = list(scene.leaves())
    groups = []
    for name, roots in GROUPS.items():
        parts = []
        for leaf in leaves:
            if not any(leaf.ref == r or leaf.ref.startswith(r + ".") for r in roots):
                continue
            if not args.keep_hardware and SMALL_HARDWARE.search(leaf.label or ""):
                continue
            if not args.keep_detail and any(
                    leaf.ref == r or leaf.ref.startswith(r + ".") for r in HEAVY_DETAIL):
                continue
            (selection,) = scene.resolve(leaf.ref)
            shape = selection.shape()
            shape.label = re.sub(r"[^A-Za-z0-9_]", "_", leaf.label or leaf.ref)
            parts.append(shape)
        group = bd.Compound(children=parts)
        group.label = name
        groups.append(group)
        print(f"{name}: {len(parts)} leaves")
    output = args.output or args.directory / "voron_split_review.step"
    bd.export_step(bd.Compound(children=groups, label="voron_v02_split"), str(output))
    print(f"Review STEP: {output}")


if __name__ == "__main__":
    main()
