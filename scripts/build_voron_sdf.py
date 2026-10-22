#!/usr/bin/env python3
"""Generate the Voron V0.2 Gazebo model (SDF 1.9) from the verified upstream STEP.

Run with the CAD skill's pinned uvx Python (see docs/VORON_SIM_MODEL.md).
Every number written to model.sdf is derived here from CAD geometry or from a
named constant below; measurements are documented in docs/VORON_MEASUREMENTS.md.

Visual meshes are derived GPL-3.0 Voron geometry. They are regenerated into
MODEL_DIR/meshes (git-ignored) rather than committed under Arctos' licence.
"""

import argparse
import math
from pathlib import Path
import re

from inspect_voron_cad import REVISION, STEP_NAME, STEP_SHA256, verified_bytes

MM = 0.001  # the one CAD (mm) -> simulator (m) conversion
FEET_Z_MM = 2.07  # CAD feet bottom is z = -2.07 mm; model frame puts them at z = 0

# Hinge axis: coaxial pin shanks #o1.5.4.5.1.1.f5 / #o1.5.4.5.1.2.f5, direction +Z.
HINGE_XY_MM = (116.502, -117.5)
# Sampled solid overlap: clear 0..165 deg, hinge clips touch from ~166 deg.
DOOR_LIMIT_RAD = math.radians(160.0)
# Estimates, not measured: small viscous damping and Coulomb friction at the hinge.
HINGE_DAMPING = 0.002  # N m s / rad
HINGE_FRICTION = 0.005  # N m
# Bed lowered from its saved pose (raised to the nozzle). At 114.4 mm the Z carriage
# touches the Z endstop microswitch; 110 mm leaves 4.4 mm margin.
BED_DROP_MM = 110.0

# Densities (kg/mm^3). Panels assumed acrylic; printed parts assumed ABS at an
# effective 60% fill (walls plus infill). All are documented estimates.
RHO = {"aluminium": 2.70e-6, "steel": 7.85e-6, "panel": 1.19e-6,
       "printed": 1.04e-6 * 0.6, "magnet": 7.5e-6}
COLOURS = {"aluminium": "0.62 0.64 0.67 1", "panel": "0.15 0.15 0.17 0.45",
           "printed": "0.12 0.12 0.13 1", "accent": "0.75 0.12 0.10 1",
           "steel": "0.35 0.36 0.38 1", "bed": "0.80 0.70 0.35 1", "electronics": "0.10 0.35 0.15 1"}

DOOR = ("#o1.5.4.2", "#o1.5.4.1.1.2", "#o1.5.4.1.2", "#o1.5.4.4")
BED_MOVING = ("#o1.3.1", "#o1.1.3.2.1.1.1", "#o1.1.3.2.2.1.1")
DRAG_CHAIN = ("#o1.3.1.5",)
EXCLUDED = ("#o1.8", "#o1.9", "#o1.10",  # spool, scale banana, tool lying on the bed
            "#o1.7", "#o1.2.1")  # electronics boards and belts: boxes only, no meshes
SMALL_HARDWARE = re.compile(r"HCS|Hex ?nut|Nut|Washer|Insert|Spring|Screw", re.IGNORECASE)
MAGNET = re.compile(r"Magnet", re.IGNORECASE)
ALUMINIUM = re.compile(r"1515|MGN|MIC6|o1\.1\.3", re.IGNORECASE)
STEEL = re.compile(r"Flex Plate|Leadscrew|Stepper|Magnetic Sheet", re.IGNORECASE)
PANEL = re.compile(r"Panel(?!.*Clip)", re.IGNORECASE)


def under(ref, roots):
    return any(ref == r or ref.startswith(r + ".") for r in roots)


def material(ref, label):
    if MAGNET.search(label):
        return "magnet"
    if PANEL.search(label):
        return "panel"
    if ALUMINIUM.search(label) or ALUMINIUM.search(ref):
        return "aluminium"
    if STEEL.search(label) or SMALL_HARDWARE.search(label):
        return "steel"
    return "printed"


class Boxes:
    """Axis-aligned collision boxes in CAD millimetres, optionally rotated about X."""

    def __init__(self):
        self.items = []

    def add(self, name, lo, hi, roll=0.0):
        size = [h - l for l, h in zip(lo, hi)]
        if min(size) <= 0:
            raise ValueError(f"Degenerate collision box {name}: {size}")
        centre = [(l + h) / 2 for l, h in zip(lo, hi)]
        self.items.append((name, centre, size, roll))

    def add_bbox(self, name, bbox, dz=0.0):
        self.add(name, (bbox.min.X, bbox.min.Y, bbox.min.Z + dz),
                 (bbox.max.X, bbox.max.Y, bbox.max.Z + dz))

    def add_oriented(self, name, centre, size, roll):
        self.items.append((name, list(centre), list(size), roll))


def fmt(values, digits=6):
    return " ".join(f"{v:.{digits}f}".rstrip("0").rstrip(".") if v else "0" for v in values)


def box_xml(kind, name, centre, size, roll, link, colour=None, surface=None):
    lines = [f'      <{kind} name="{name}">',
             f'        <pose relative_to="{link}">{fmt([c * MM for c in centre])} {fmt([roll, 0, 0])}</pose>',
             f'        <geometry><box><size>{fmt([s * MM for s in size])}</size></box></geometry>']
    if colour:
        lines.append(f"        <material><ambient>{colour}</ambient><diffuse>{colour}</diffuse></material>")
    if surface:
        lines.append(surface)
    lines.append(f"      </{kind}>")
    return "\n".join(lines)


def mesh_visual(name, uri, link, colour):
    return "\n".join([
        f'      <visual name="{name}">',
        f'        <pose relative_to="{link}">0 0 0 0 0 0</pose>',
        f"        <geometry><mesh><uri>{uri}</uri><scale>{MM} {MM} {MM}</scale></mesh></geometry>",
        f"        <material><ambient>{colour}</ambient><diffuse>{colour}</diffuse></material>",
        *(["        <transparency>0.55</transparency>"] if name.endswith("_panel") else []),
        "      </visual>"])


def inertial_xml(props, link):
    com = [c * MM for c in props.center_of_mass]
    i = [[v * MM * MM for v in row] for row in props.inertia]  # kg mm^2 -> kg m^2
    return "\n".join([
        "      <inertial>",
        f'        <pose relative_to="{link}">{fmt(com)} 0 0 0</pose>',
        f"        <mass>{props.mass:.6f}</mass>",
        f"        <inertia><ixx>{i[0][0]:.9g}</ixx><ixy>{i[0][1]:.9g}</ixy><ixz>{i[0][2]:.9g}</ixz>"
        f"<iyy>{i[1][1]:.9g}</iyy><iyz>{i[1][2]:.9g}</iyz><izz>{i[2][2]:.9g}</izz></inertia>",
        "      </inertial>"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, default=Path.home() / "voron-inspection")
    parser.add_argument("--model-dir", type=Path,
                        default=Path(__file__).resolve().parents[1] / "arctos_gazebo/models/voron_v02")
    parser.add_argument("--mesh-tolerance", type=float, default=0.25, help="chord tolerance, mm")
    parser.add_argument("--meshes-only", action="store_true",
                        help="write meshes but leave the committed model.sdf untouched")
    args = parser.parse_args()
    step = args.directory / STEP_NAME
    verified_bytes(step, STEP_SHA256)

    from cadgen import build123d as bd
    from cadgen import read_scene
    from cadgen.geometry import mass_properties

    scene = read_scene(str(step))
    bbox = lambda ref: scene.resolve(ref)[0].shape().bounding_box(optimal=False)  # noqa: E731

    # --- Classify leaves into (link, visual group) and collect mass bodies --------
    meshes = {}  # (link, group) -> [shapes]
    mass_bodies = {"chassis": [], "door": []}
    drop = bd.Location((0, 0, -BED_DROP_MM))
    for leaf in scene.leaves():
        ref, label = leaf.ref, leaf.label or ""
        if under(ref, EXCLUDED) or under(ref, DRAG_CHAIN):
            continue
        link = "door" if under(ref, DOOR) else "chassis"
        mat = material(ref, label)
        shape = scene.resolve(ref)[0].shape()
        if under(ref, BED_MOVING):
            shape = shape.moved(drop)
        for solid in shape.solids():
            mass_bodies[link].append((solid, RHO[mat]))
        if SMALL_HARDWARE.search(label) or mat == "magnet":
            continue  # mass only; too small to be worth tessellating
        group = "bed" if under(ref, BED_MOVING) else mat
        meshes.setdefault((link, group), []).append(shape)

    mesh_dir = args.model_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    visuals = {"chassis": [], "door": []}
    for (link, group), shapes in sorted(meshes.items()):
        name = f"{link}_{group}"
        path = mesh_dir / f"{name}.stl"
        bd.export_stl(bd.Compound(children=shapes), str(path),
                      tolerance=args.mesh_tolerance, angular_tolerance=0.5)
        colour = COLOURS["accent" if (link == "door" and group == "printed") else group]
        visuals[link].append(mesh_visual(name, f"meshes/{name}.stl", link, colour))
        print(f"mesh {path.name}: {len(shapes)} parts, {path.stat().st_size / 1e6:.1f} MB")
    if args.meshes_only:
        return

    props = {link: mass_properties(bodies) for link, bodies in mass_bodies.items()}

    # --- Collision primitives (CAD mm) --------------------------------------------
    door, chassis = Boxes(), Boxes()
    panel = bbox("#o1.5.4.2")
    door.add_bbox("panel", panel)
    for ref, name in (("#o1.5.4.1.2.1", "hinge_bottom"), ("#o1.5.4.1.2.2", "hinge_top"),
                      ("#o1.5.4.1.2.3", "clip_bottom_left"), ("#o1.5.4.1.2.4", "clip_top_left")):
        door.add_bbox(name, bbox(ref))
    # Handle: two feet, the straight bar and two 45-degree legs, leaving the finger gap
    # (panel face y = -118 to bar inner face y = -133) open. Profile sampled at x = -95.5.
    h = bbox("#o1.5.4.1.1.2")
    x0, x1 = h.min.X, h.max.X
    door.add("handle_foot_bottom", (x0, -123.0, h.min.Z), (x1, panel.min.Y, 122.0))
    door.add("handle_foot_top", (x0, -123.0, 200.0), (x1, panel.min.Y, h.max.Z))
    door.add("handle_bar", (x0, h.min.Y, 139.0), (x1, -133.0, 183.5))
    leg_len, leg_thick = 24.0 * math.sqrt(2), 14.5 / math.sqrt(2)
    door.add_oriented("handle_leg_bottom", ((x0 + x1) / 2, -129.85, 131.0), (x1 - x0, leg_thick, leg_len), math.pi / 4)
    door.add_oriented("handle_leg_top", ((x0 + x1) / 2, -130.3, 191.0), (x1 - x0, leg_thick, leg_len), -math.pi / 4)

    for leaf in scene.leaves():
        ref, label = leaf.ref, leaf.label or ""
        if under(ref, ("#o1.1.1",)):
            chassis.add_bbox(f"extrusion_{ref[4:].replace('.', '_')}", bbox(ref))
        elif under(ref, ("#o1.5.1", "#o1.5.2")) and PANEL.search(label):
            b = bbox(ref)
            if min(b.size) > 3.5:
                raise ValueError(f"Panel {ref} is not a thin plate: {b.size}")
            chassis.add_bbox(f"panel_{ref[4:].replace('.', '_')}", b)
    for ref, name in (("#o1.5.1.2.2.1", "hinge_clip_upper"), ("#o1.5.1.2.2.2", "hinge_clip_lower"),
                      ("#o1.5.4.1.1.1", "latch_upper"), ("#o1.5.4.1.1.3", "latch_lower"),
                      ("#o1.4.3", "display"), ("#o1.2.5", "toolhead"), ("#o1.2.4", "x_axis"),
                      ("#o1.2.2.1", "ab_drive_1"), ("#o1.2.2.2", "ab_drive_2"),
                      ("#o1.2.3.1", "idler_1"), ("#o1.2.3.2", "idler_2"),
                      ("#o1.1.3.1.1", "y_rail_left"), ("#o1.1.3.1.2", "y_rail_right"),
                      ("#o1.1.3.2.1.1.2", "z_rail_left"), ("#o1.1.3.2.2.1.2", "z_rail_right"),
                      ("#o1.7.1", "electronics_fan"), ("#o1.7.2", "electronics_skr_pico"),
                      ("#o1.7.3", "electronics_raspberry_pi"), ("#o1.7.4", "electronics_psu")):
        chassis.add_bbox(name, bbox(ref))
    # Base: skirts and feet below the deck; tophat closed, as one envelope each.
    s1, s2 = bbox("#o1.4.1"), bbox("#o1.4.2")
    chassis.add("base", (min(s1.min.X, s2.min.X), min(s1.min.Y, s2.min.Y), min(s1.min.Z, s2.min.Z)),
                (max(s1.max.X, s2.max.X), max(s1.max.Y, s2.max.Y), max(s1.max.Z, s2.max.Z)))
    t = bbox("#o1.6.3")
    chassis.add("tophat", (-119.4, -119.4, t.min.Z), (119.4, 119.4, 382.0))  # incl. corner clips
    # Bed at the retrieval pose.
    for ref, name in (("#o1.3.1.1", "bed_plate"), ("#o1.3.1.4.1", "bed_frame_rear"),
                      ("#o1.3.1.4.2", "bed_frame_left"), ("#o1.3.1.4.3", "bed_frame_right"),
                      ("#o1.3.1.3.1", "bed_nut_block"), ("#o1.3.1.3.2", "bed_mount_left"),
                      ("#o1.3.1.3.3", "bed_mount_right"), ("#o1.3.1.3.6", "bed_front"),
                      ("#o1.1.3.2.1.1.1", "z_carriage_left"), ("#o1.1.3.2.2.1.1", "z_carriage_right")):
        chassis.add_bbox(name, bbox(ref), dz=-BED_DROP_MM)
    # Drag chain re-routes when the bed moves; approximated by its lowered envelope.
    c = bbox("#o1.3.1.5")
    chassis.add("drag_chain_envelope", (c.min.X, c.min.Y, c.min.Z), (c.max.X, c.max.Y, c.max.Z - BED_DROP_MM))
    m = bbox("#o1.3.2.1")
    nut = bbox("#o1.3.1.2.2.1")
    chassis.add("z_motor", (m.min.X, m.min.Y, m.min.Z), (m.max.X, m.max.Y, 64.2))
    lead_xy = ((nut.min.X + nut.max.X) / 2, (nut.min.Y + nut.max.Y) / 2)
    chassis.add("z_leadscrew", (lead_xy[0] - 4, lead_xy[1] - 4, 64.2), (lead_xy[0] + 4, lead_xy[1] + 4, m.max.Z))

    # --- Frames -------------------------------------------------------------------
    flex = bbox("#o1.3.1.1.4")
    mic6 = bbox("#o1.3.1.1.1")
    bed_surface = ((mic6.min.X + mic6.max.X) / 2, (mic6.min.Y + mic6.max.Y) / 2, flex.max.Z - BED_DROP_MM)
    hinge = (*HINGE_XY_MM, (panel.min.Z + panel.max.Z) / 2)
    grasp = ((x0 + x1) / 2, (h.min.Y - 133.0) / 2, (139.0 + 183.5) / 2)
    opening = (0.0, -115.0, bed_surface[2])

    surface = ("        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>")
    collisions = {
        "door": [box_xml("collision", n, c, s, r, "door", surface=surface if n.startswith("handle") else None)
                 for n, c, s, r in door.items],
        "chassis": [box_xml("collision", n, c, s, r, "chassis") for n, c, s, r in chassis.items],
    }
    extra_visuals = [box_xml("visual", f"{n}_visual", c, s, r, "chassis", COLOURS["electronics"])
                     for n, c, s, r in chassis.items if n.startswith("electronics") or n == "drag_chain_envelope"]

    p_d, p_c = props["door"], props["chassis"]
    ledger = f"""<!--
  Voron V0.2R1 printer for Arctos printer-tending simulation. GENERATED by
  scripts/build_voron_sdf.py; edit the generator, not this file.

  Source: VoronDesign/Voron-0 @ {REVISION} (GPL-3.0),
          {STEP_NAME} sha256 {STEP_SHA256}
  Consumer: Gazebo Harmonic / ROS 2 Jazzy, Bullet Featherstone, SDF 1.9 (project baseline).
  Units: m, kg, s, rad. Mesh units mm, <scale> {MM} (single conversion, MM in generator).
  Frames: chassis and door link frames = CAD frame (+Z up, door at -Y, origin at the
          frame's XY centre) translated +{FEET_Z_MM} mm in Z so the feet sit at model z = 0.

  Links
    chassis  anchored to world by fixed joint "anchor". Frame, panels, skirts, closed
             tophat, parked gantry/toolhead, Z drive and bed (all fixed). Mass
             {p_c.mass:.3f} kg from CAD solid volumes x material estimates (electronics excluded).
    door     dynamic. Panel, handle, handle magnets, door-side hinges and left clips.
             Mass {p_d.mass:.4f} kg, COM/inertia from CAD solids (mass_properties):
             panel acrylic {RHO['panel']:.3g}, printed ABS x0.6 {RHO['printed']:.3g},
             magnets {RHO['magnet']:.3g} kg/mm^3 (estimates).
  Joints
    anchor      fixed, world -> chassis.
    door_hinge  revolute, chassis -> door. Axis +Z through CAD (x, y) = {HINGE_XY_MM} mm,
                derived from the coaxial hinge-pin shanks. Positive = door opens outward
                (toward -Y). Limits 0..{DOOR_LIMIT_RAD:.4f} rad (0..160 deg); sampled CAD
                overlap is clear to 165 deg. Damping {HINGE_DAMPING} N m s/rad and friction
                {HINGE_FRICTION} N m are estimates. No magnetic latch force.
  Frames
    hinge          on the hinge axis at door mid-height.
    handle_grasp   centre of the straight handle bar; pinch axis X (bar 10 mm wide),
                   approach +Y toward the panel; finger gap 15 mm (y -118..-133 CAD).
    bed_surface    flex-plate top centre at the retrieval pose: bed lowered
                   {BED_DROP_MM} mm from its saved pose (4.4 mm above the Z endstop contact).
    front_opening  front frame plane (CAD y = -115) at bed height; clear width x -100..100.
  Collisions: boxes only. Door-to-chassis contact is not simulated (no self-collision);
  the joint limits stop the door. Handle boxes keep the finger gap open.
  Visual meshes: derived GPL-3.0 geometry in meshes/, regenerated, not committed.
-->"""

    sdf = "\n".join([
        '<?xml version="1.0"?>', ledger, '<sdf version="1.9">', '  <model name="voron_v02">',
        '    <link name="chassis">',
        f'      <pose relative_to="__model__">0 0 {FEET_Z_MM * MM:.5f} 0 0 0</pose>',
        inertial_xml(p_c, "chassis"), *visuals["chassis"], *extra_visuals, *collisions["chassis"],
        "    </link>",
        '    <link name="door">',
        f'      <pose relative_to="__model__">0 0 {FEET_Z_MM * MM:.5f} 0 0 0</pose>',
        inertial_xml(p_d, "door"), *visuals["door"], *collisions["door"],
        "    </link>",
        '    <joint name="anchor" type="fixed"><parent>world</parent><child>chassis</child></joint>',
        '    <joint name="door_hinge" type="revolute">',
        f'      <pose relative_to="door">{fmt([v * MM for v in hinge])} 0 0 0</pose>',
        "      <parent>chassis</parent>", "      <child>door</child>",
        "      <axis>", '        <xyz expressed_in="chassis">0 0 1</xyz>',
        f"        <limit><lower>0</lower><upper>{DOOR_LIMIT_RAD:.6f}</upper></limit>",
        f"        <dynamics><damping>{HINGE_DAMPING}</damping><friction>{HINGE_FRICTION}</friction></dynamics>",
        "      </axis>", "    </joint>",
        f'    <frame name="hinge" attached_to="chassis"><pose relative_to="chassis">{fmt([v * MM for v in hinge])} 0 0 0</pose></frame>',
        f'    <frame name="handle_grasp" attached_to="door"><pose relative_to="door">{fmt([v * MM for v in grasp])} 0 0 0</pose></frame>',
        f'    <frame name="bed_surface" attached_to="chassis"><pose relative_to="chassis">{fmt([v * MM for v in bed_surface])} 0 0 0</pose></frame>',
        f'    <frame name="front_opening" attached_to="chassis"><pose relative_to="chassis">{fmt([v * MM for v in opening])} 0 0 0</pose></frame>',
        "  </model>", "</sdf>", ""])
    out = args.model_dir / "model.sdf"
    out.write_text(sdf)
    print(f"door mass {p_d.mass:.4f} kg, COM mm {[round(v, 2) for v in p_d.center_of_mass]}")
    print(f"chassis mass {p_c.mass:.3f} kg; bed surface CAD z {bed_surface[2]:.2f} mm")
    print(f"wrote {out}: {len(door.items)} door + {len(chassis.items)} chassis collision boxes")


if __name__ == "__main__":
    main()
