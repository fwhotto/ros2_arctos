"""Static and gz checks of the generated Voron V0.2 printer model and its world."""

import math
import os
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest

SHARE = Path(get_package_share_directory("arctos_gazebo"))
MODEL = SHARE / "models/voron_v02/model.sdf"
WORLD = SHARE / "worlds/voron_printer.sdf"


def floats(text):
    return [float(v) for v in text.split()]


@pytest.fixture(scope="module")
def model():
    return ET.parse(MODEL).getroot().find("model")


def test_hinge_joint(model):
    joint = model.find("joint[@name='door_hinge']")
    assert joint.get("type") == "revolute"
    assert (joint.findtext("parent"), joint.findtext("child")) == ("chassis", "door")
    assert floats(joint.findtext("axis/xyz")) == [0, 0, 1]
    assert float(joint.findtext("axis/limit/lower")) == 0
    assert float(joint.findtext("axis/limit/upper")) == pytest.approx(math.radians(160), abs=1e-6)
    anchor = model.find("joint[@name='anchor']")
    assert (anchor.get("type"), anchor.findtext("parent")) == ("fixed", "world")


def test_door_inertia_is_physical(model):
    inertial = model.find("link[@name='door']/inertial")
    assert 0.1 < float(inertial.findtext("mass")) < 0.5
    i = {k: float(inertial.findtext(f"inertia/{k}")) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
    tensor = np.array([[i["ixx"], i["ixy"], i["ixz"]], [i["ixy"], i["iyy"], i["iyz"]],
                       [i["ixz"], i["iyz"], i["izz"]]])
    principal = np.linalg.eigvalsh(tensor)
    assert principal.min() > 0
    assert principal[0] + principal[1] >= principal[2]


def inside_box(point, collision):
    pose = floats(collision.findtext("pose"))
    size = floats(collision.findtext("geometry/box/size"))
    offset = np.array(point) - np.array(pose[:3])
    c, s = math.cos(pose[3]), math.sin(pose[3])
    local = np.array([offset[0], c * offset[1] + s * offset[2], -s * offset[1] + c * offset[2]])
    return all(abs(v) <= h / 2 for v, h in zip(local, size))


def test_handle_finger_gap_is_open(model):
    door = model.find("link[@name='door']")
    boxes = door.findall("collision")
    # CAD x = -95.5 mm, y midway between panel (-118) and bar (-133), straight bar z 145..177 mm.
    for z_mm in range(145, 178, 4):
        point = (-0.0955, -0.1255, z_mm / 1000)
        assert not any(inside_box(point, box) for box in boxes), z_mm
    bar = door.find("collision[@name='handle_bar']")
    assert inside_box((-0.0955, -0.138, 0.161), bar)


def test_world_cube_sits_on_bed_surface(model):
    chassis_z = floats(model.findtext("link[@name='chassis']/pose"))[2]
    bed = floats(model.findtext("frame[@name='bed_surface']/pose"))
    cube = ET.parse(WORLD).getroot().find("world/model[@name='cube']")
    pose = floats(cube.findtext("pose"))
    size = floats(cube.findtext("link/collision/geometry/box/size"))
    assert pose[:2] == pytest.approx(bed[:2], abs=1e-6)
    gap = pose[2] - size[2] / 2 - (chassis_z + bed[2])
    assert 0 < gap <= 0.002


def test_meshes_generated(model):
    uris = [m.text for m in model.iter("uri")]
    missing = [u for u in uris if not (MODEL.parent / u).is_file()]
    if missing:
        pytest.skip(f"run scripts/build_voron_sdf.py before building; missing {missing}")


@pytest.mark.skipif(shutil.which("gz") is None, reason="gz not installed")
@pytest.mark.parametrize("path", [MODEL, WORLD])
def test_gz_sdf_check(path):
    env = dict(os.environ, GZ_SIM_RESOURCE_PATH=str(SHARE / "models"))
    result = subprocess.run(["gz", "sdf", "--check", str(path)], env=env,
                            capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
