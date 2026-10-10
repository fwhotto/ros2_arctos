"""Physical/model invariants for the backend-independent installed description."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
import xacro
import yaml


SHARE = Path(get_package_share_directory("arctos_description"))
JOINTS = [f"{name}_joint" for name in "XYZABC"]


def expand(filename="arctos.xacro", **mappings):
    return ET.fromstring(xacro.process_file(
        str(SHARE / "urdf" / filename), mappings=mappings
    ).toxml())


def vertices(mesh):
    prefix = "package://arctos_description/"
    assert mesh.get("filename").startswith(prefix)
    path = SHARE / mesh.get("filename")[len(prefix):]
    raw = path.read_bytes()
    count = int.from_bytes(raw[80:84], "little")
    dtype = np.dtype([
        ("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")
    ])
    assert len(raw) == 84 + 50 * count
    points = np.frombuffer(raw, dtype=dtype, offset=84)["vertices"].reshape(-1, 3)
    return points.astype(float) * np.fromstring(mesh.get("scale"), sep=" ")


@pytest.mark.parametrize("options,expected", [
    ({}, JOINTS),
    ({"include_gripper": "true"}, JOINTS + ["Left_jaw_joint", "Right_jaw_joint"]),
    ({"prefix": "test_"}, ["test_" + joint for joint in JOINTS]),
])
def test_description_structure_and_urdf_parser(tmp_path, options, expected):
    robot = expand(**options)
    moving = [j.get("name") for j in robot.findall("joint") if j.get("type") != "fixed"]
    assert moving == expected
    assert robot.find("ros2_control") is None
    assert robot.find("transmission") is None
    assert robot.find("gazebo") is None
    if not options.get("include_gripper"):
        assert not any("jaw" in link.get("name") for link in robot.findall("link"))
    path = tmp_path / "robot.urdf"
    ET.ElementTree(robot).write(path)
    result = subprocess.run(["check_urdf", str(path)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Successfully Parsed" in result.stdout


def test_physical_limits_and_dynamics():
    limits = yaml.safe_load((SHARE / "config/physical_joint_limits.yaml").read_text())
    assert list(limits) == JOINTS
    for joint in expand().findall("joint"):
        if joint.get("type") == "fixed":
            continue
        values = {name: float(value) for name, value in joint.find("limit").attrib.items()}
        assert values == limits[joint.get("name")]
        assert values["lower"] < 0 < values["upper"]
        assert values["velocity"] > 0 and values["effort"] > 0
        axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
        assert np.linalg.norm(axis) == pytest.approx(1)
        dynamics = joint.find("dynamics")
        assert 0 < float(dynamics.get("damping")) <= 1
        assert float(dynamics.get("friction")) >= 0


@pytest.mark.parametrize("include_gripper", ["false", "true"])
def test_inertias_and_mesh_alignment(include_gripper):
    for link in expand(include_gripper=include_gripper).findall("link"):
        visual = link.find("visual")
        if visual is None:
            assert link.get("name") in {"world", "tool0"}
            continue
        assert ET.tostring(visual.find("geometry")) == ET.tostring(link.find("collision/geometry"))
        points = vertices(visual.find("geometry/mesh"))
        points += np.fromstring(visual.find("origin").get("xyz"), sep=" ")
        origin = link.find("inertial/origin")
        assert origin.get("rpy") == "0 0 0"
        com = np.fromstring(origin.get("xyz"), sep=" ")
        assert np.all(com >= points.min(0) - 0.001), link.get("name")
        assert np.all(com <= points.max(0) + 0.001), link.get("name")
        mass = float(link.find("inertial/mass").get("value"))
        assert np.isfinite(mass) and mass > 0
        entries = link.find("inertial/inertia").attrib
        inertia = np.array([
            [float(entries["i" + "".join(sorted(x + y))]) for y in "xyz"]
            for x in "xyz"
        ])
        eigenvalues = np.linalg.eigvalsh(inertia)
        assert np.isfinite(eigenvalues).all()
        assert eigenvalues[0] > 0, link.get("name")
        # Principal moments must satisfy the rigid-body triangle inequality.
        assert eigenvalues[0] + eigenvalues[1] >= eigenvalues[2] - 1e-10, link.get("name")
        # Radius of gyration cannot exceed the furthest mesh point from the COM.
        radius_squared = np.max(np.sum((points - com) ** 2, axis=1))
        assert eigenvalues[-1] / mass <= radius_squared, link.get("name")


def test_reduced_model_shares_geometry_axes_and_bounds():
    full = expand()
    reduced = expand("base_xyz/arctos.xacro")
    assert reduced.find("ros2_control") is None
    for tag in ("joint", "link"):
        for element in reduced.findall(tag):
            other = full.find(f"{tag}[@name='{element.get('name')}']")
            assert other is not None
            assert ET.tostring(element) == ET.tostring(other)
    assert [j.get("name") for j in reduced.findall("joint") if j.get("type") != "fixed"] == JOINTS[:3]


def test_zero_pose_matches_cad_assembly_and_clears_ground():
    robot = expand()
    transforms = {"world": np.zeros(3)}
    # All zero-pose joint origin rotations are identity in this model.
    for joint in robot.findall("joint"):
        origin = joint.find("origin")
        assert origin.get("rpy") == "0 0 0"
        transforms[joint.find("child").get("link")] = (
            transforms[joint.find("parent").get("link")]
            + np.fromstring(origin.get("xyz"), sep=" ")
        )
    for link in robot.findall("link"):
        visual = link.find("visual")
        if visual is None:
            continue
        mesh_offset = transforms[link.get("name")] + np.fromstring(
            visual.find("origin").get("xyz"), sep=" "
        )
        # STL coordinates are in the original assembly frame. Preserve the
        # assembled geometry to within 0.1 mm despite rounded joint origins.
        assert np.linalg.norm(mesh_offset) < 0.0001, link.get("name")
        minimum_z = (vertices(visual.find("geometry/mesh")) + mesh_offset)[:, 2].min()
        if link.get("name") == "base_link":
            assert minimum_z == pytest.approx(0.0, abs=1e-7)
        else:
            assert minimum_z > 0.1, link.get("name")


def test_gripper_pad_gap_matches_manufacturer_stroke():
    robot = expand(include_gripper="true")
    pads = []
    for name in ("Left_jaw_1", "Right_jaw_1"):
        visual = robot.find(f"link[@name='{name}']/visual")
        points = vertices(visual.find("geometry/mesh"))
        # Distal gripping pads, away from the overlapping rack/body geometry.
        pads.append(points[points[:, 1] < -0.39])
    gap = pads[0][:, 2].min() - pads[1][:, 2].max()
    assert gap == pytest.approx(0.03, abs=1e-6)
    stroke = float(robot.find("joint[@name='Left_jaw_joint']/limit").get("upper"))
    assert gap - 2 * stroke == pytest.approx(0, abs=1e-6)


@pytest.mark.parametrize("filename", ["arctos.xacro", "base_xyz/arctos.xacro"])
def test_legacy_control_bounds_match_geometry(filename):
    robot = expand(filename, use_hardware="true")
    control = robot.find("ros2_control")
    assert control is not None
    moving = {j.get("name") for j in robot.findall("joint") if j.get("type") != "fixed"}
    assert {j.get("name") for j in control.findall("joint")} == moving
    for joint in control.findall("joint"):
        limit = robot.find(f"joint[@name='{joint.get('name')}']/limit")
        assert [c.get("name") for c in joint.findall("command_interface")] == ["position"]
        command = joint.find("command_interface")
        assert float(command.find("param[@name='min']").text) == float(limit.get("lower"))
        assert float(command.find("param[@name='max']").text) == float(limit.get("upper"))


def test_macro_can_be_instantiated_twice(tmp_path):
    path = tmp_path / "two_arms.xacro"
    path.write_text(f'''<robot name="two_arms" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:include filename="{SHARE}/urdf/arctos_arm.xacro"/>
      <link name="world"/>
      <xacro:arctos_arm prefix="left_"/>
      <xacro:arctos_arm prefix="right_"/>
      <joint name="left_mount" type="fixed">
        <parent link="world"/><child link="left_base_link"/><origin xyz="-1 0 0"/>
      </joint>
      <joint name="right_mount" type="fixed">
        <parent link="world"/><child link="right_base_link"/><origin xyz="1 0 0"/>
      </joint>
    </robot>''')
    robot = ET.fromstring(xacro.process_file(str(path)).toxml())
    for tag in ("link", "joint"):
        names = [element.get("name") for element in robot.findall(tag)]
        assert len(names) == len(set(names))
    assert robot.find("link[@name='left_tool0']") is not None
    assert robot.find("link[@name='right_tool0']") is not None


def test_reduced_legacy_include_keeps_three_joints(tmp_path):
    path = tmp_path / "legacy.xacro"
    path.write_text(f'''<robot name="arctos" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:include filename="{SHARE}/urdf/base_xyz/ros2_control.xacro"/>
      <xacro:arctos_ros2_control name="legacy"/>
    </robot>''')
    robot = ET.fromstring(xacro.process_file(str(path)).toxml())
    assert [j.get("name") for j in robot.findall("ros2_control/joint")] == JOINTS[:3]
