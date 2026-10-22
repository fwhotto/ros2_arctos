#!/usr/bin/env python3
"""Fetch verified upstream CAD and inventory it without changing its geometry.

Run with the CAD skill's pinned uvx Python, not the ROS Python environment.
All downloaded assets and reports stay outside the repository by default.
"""

import argparse
import hashlib
import json
from pathlib import Path
from urllib.request import urlopen
import zipfile


REVISION = "a53fc87562fd630c846af38d7de850c894dc3d85"
BASE_URL = f"https://raw.githubusercontent.com/VoronDesign/Voron-0/{REVISION}/"
STEP_NAME = "V0.2R1 Master Assembly v63.step"
STEP_SHA256 = "d5a9d40db82c55eee20203608a5a4e784b54b87ae339861af577c5aeb182d61f"
ASSETS = (
    ("CAD/V0.2R1_Master_Assembly_STEP.zip", "V0.2R1_Master_Assembly_STEP.zip",
     "a1cf81917dbeb55c13b23a490763297f85e4aeef5c0f2c45274a4969b5103b25"),
    ("LICENSE", "LICENSE",
     "3972dc9744f6499f0f9b2dbf76696f2ae7ad8af9b23dde66d6af86c9dfb36986"),
    ("Drawings/Front_Door_v0.2.PDF", "Front_Door_v0.2.PDF",
     "bf1134bacfae8655bad4520cf858acddb3b976d2fd1fb6be3b8dac729138f691"),
)


def verified_bytes(path, expected):
    content = path.read_bytes()
    actual = hashlib.sha256(content).hexdigest()
    if actual != expected:
        raise ValueError(f"SHA256 mismatch for {path}: {actual}; expected {expected}")
    return content


def fetch_sources(directory):
    directory.mkdir(parents=True, exist_ok=True)
    for remote, filename, expected in ASSETS:
        path = directory / filename
        if not path.exists():
            with urlopen(BASE_URL + remote, timeout=120) as response:
                content = response.read()
            if hashlib.sha256(content).hexdigest() != expected:
                raise ValueError(f"Downloaded asset checksum mismatch: {remote}")
            # Never overwrite an existing local asset, even during a racing fetch.
            with path.open("xb") as stream:
                stream.write(content)
        verified_bytes(path, expected)
    step = directory / STEP_NAME
    if not step.exists():
        with zipfile.ZipFile(directory / ASSETS[0][1]) as archive:
            # Read exactly the approved member; never extract arbitrary paths.
            content = archive.read(STEP_NAME)
        if hashlib.sha256(content).hexdigest() != STEP_SHA256:
            raise ValueError("Archived STEP checksum mismatch")
        with step.open("xb") as stream:
            stream.write(content)
    verified_bytes(step, STEP_SHA256)
    return step


def inventory(step, refs):
    from cadgen import read_scene

    scene = read_scene(str(step))
    occurrences = []

    def walk(node, depth=0):
        occurrences.append({
            "ref": node.ref,
            "label": node.label,
            "prototype_id": node.prototype_id,
            "depth": depth,
            "children": len(node.children),
        })
        for child in node.children:
            walk(child, depth + 1)

    for root in scene.roots:
        walk(root)
    measurements = []
    for ref in refs:
        (selection,) = scene.resolve(ref)
        shape = selection.shape()
        bounds = shape.bounding_box()
        measurements.append({
            "ref": selection.ref,
            "kind": selection.kind,
            "bounds_min_mm": list(bounds.min),
            "bounds_max_mm": list(bounds.max),
            "bounds_size_mm": list(bounds.size),
            "solid_count": len(shape.solids()),
            "volume_mm3": shape.volume,
        })
    return {
        "upstream_revision": REVISION,
        "source": str(step.resolve()),
        "source_sha256": STEP_SHA256,
        "document_hash": scene.document_hash,
        "measurement_units": "mm; mm^3 for volume",
        "measurement_scope": "Saved CAD placements, not evaluated joint motion. Bounds are envelopes, not clear openings.",
        "occurrences": occurrences,
        "measurements": measurements,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path,
                        default=Path("/tmp/opencode/voron-inspection"))
    parser.add_argument("--fetch", action="store_true", help="Download pinned, checksum-verified sources")
    parser.add_argument("--fetch-only", action="store_true", help="Verify/download sources without loading CAD")
    parser.add_argument("--measure", action="append", default=[], metavar="REF",
                        help="Measure a CAD occurrence/entity ref from the inventory; repeatable")
    parser.add_argument("--output", type=Path, help="Report JSON; default DIRECTORY/inventory.json")
    args = parser.parse_args()
    step = args.directory / STEP_NAME
    if args.fetch or args.fetch_only:
        step = fetch_sources(args.directory)
    else:
        verified_bytes(step, STEP_SHA256)
    if args.fetch_only:
        print(f"Verified source: {step}")
        return
    report = inventory(step, args.measure)
    output = args.output or args.directory / "inventory.json"
    # The report is reproducible; preserve reports by selecting a different --output.
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(f"Report: {output}")
    print(f"Occurrences: {len(report['occurrences'])}")
    print(f"Leaf occurrences: {sum(not item['children'] for item in report['occurrences'])}")
    for measurement in report["measurements"]:
        print(json.dumps(measurement))


if __name__ == "__main__":
    main()
