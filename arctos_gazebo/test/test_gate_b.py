"""Gate B: SDF conversion and actual MoveIt/FCL mesh collision checks."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit.core.collision_detection import CollisionRequest, CollisionResult
from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_model import RobotModel
from moveit_msgs.msg import CollisionObject
import numpy as np
import pytest
from shape_msgs.msg import SolidPrimitive
import xacro
import yaml


@pytest.fixture(scope="module")
def model_files(tmp_path_factory):
    directory = tmp_path_factory.mktemp("gate_b")
    share = Path(get_package_share_directory("arctos_description"))
    urdf = xacro.process_file(str(share / "urdf/arctos.xacro")).toxml()
    (directory / "arm.urdf").write_text(urdf)
    root = ET.fromstring(urdf)
    srdf = ET.Element("robot", name="arctos")
    group = ET.SubElement(srdf, "group", name="arm")
    ET.SubElement(group, "chain", base_link="base_link", tip_link="tool0")
    # Do not use the legacy SRDF's broad "Never" exclusions: test every
    # non-adjacent pair, only excluding mechanically connected neighbours.
    for joint in root.findall("joint"):
        ET.SubElement(srdf, "disable_collisions",
                      link1=joint.find("parent").get("link"),
                      link2=joint.find("child").get("link"), reason="Adjacent")
    ET.ElementTree(srdf).write(directory / "arm.srdf")
    return directory


@pytest.fixture(scope="module")
def robot_model(model_files):
    return RobotModel(str(model_files / "arm.urdf"), str(model_files / "arm.srdf"))


def initial_state(scene):
    control = Path(get_package_share_directory("arctos_control"))
    initial = yaml.safe_load((control / "config/initial_positions.yaml").read_text())["initial_positions"]
    assert set(initial) == {f"{axis}_joint" for axis in "XYZABC"}
    state = scene.current_state
    state.set_to_default_values()
    state.joint_positions = initial
    state.update()
    return state


def box(name, dimensions, position):
    obj = CollisionObject()
    obj.id = name
    obj.header.frame_id = "world"
    obj.operation = CollisionObject.ADD
    shape = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=dimensions)
    pose = Pose()
    pose.orientation.w = 1.0
    pose.position.x, pose.position.y, pose.position.z = position
    obj.primitives = [shape]
    obj.primitive_poses = [pose]
    return obj


def test_sdf_preserves_articulation_and_world_anchor(model_files):
    result = subprocess.run(["gz", "sdf", "-p", str(model_files / "arm.urdf")],
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    assert not result.stderr.strip(), result.stderr
    (model_files / "arm.sdf").write_text(result.stdout)
    model = ET.fromstring(result.stdout).find("model")
    assert model is not None
    assert model.findtext("static", "false") == "false"
    joints = model.findall("joint")
    assert {j.get("name") for j in joints if j.get("type") == "revolute"} == {
        f"{axis}_joint" for axis in "XYZABC"
    }
    anchor = model.find("joint[@name='world_joint']")
    assert anchor.get("type") == "fixed"
    assert anchor.findtext("parent") == "world"
    assert anchor.findtext("child") == "base_link"
    for joint in joints:
        if joint.get("type") == "revolute":
            assert model.find(f"link[@name='{joint.findtext('child')}']/inertial") is not None
            assert float(joint.findtext("axis/limit/lower")) < float(joint.findtext("axis/limit/upper"))
            assert float(joint.findtext("axis/dynamics/damping")) > 0
    # A massless fixed tool frame may be lumped into its parent link, but must
    # remain represented as a named SDF frame.
    assert model.find("frame[@name='tool0']") is not None


def test_initial_pose_is_self_collision_free(robot_model):
    scene = PlanningScene(robot_model)
    state = initial_state(scene)
    result = CollisionResult()
    scene.check_self_collision(CollisionRequest(), result, state)
    assert not result.collision
    tool = state.get_global_link_transform("tool0")
    np.testing.assert_allclose(tool[:3, 3], [0, -0.323031, 0.5685601], atol=1e-7)
    np.testing.assert_allclose(tool[:3, :3], np.eye(3), atol=1e-10)


def test_ground_support_and_obstacle_detection(robot_model):
    scene = PlanningScene(robot_model)
    state = initial_state(scene)
    scene.apply_collision_object(box("support", [2.0, 2.0, 0.1], [0.0, 0.0, -0.05]))
    assert any(obj.id == "support" for obj in scene.planning_scene_message.world.collision_objects)
    # The bolted base touches the support by design; no moving link is exempted.
    scene.allowed_collision_matrix.set_entry("base_link", "support", True)
    result = CollisionResult()
    scene.check_collision(CollisionRequest(), result, state)
    assert not result.collision
    # Negative control: the same checker must detect an obstructed arm.
    scene.apply_collision_object(box("obstruction", [1.0, 1.0, 1.0], [0.0, 0.0, 0.5]))
    assert any(obj.id == "obstruction" for obj in scene.planning_scene_message.world.collision_objects)
    result = CollisionResult()
    scene.check_collision(CollisionRequest(), result, state)
    assert result.collision


def test_known_folded_pose_is_rejected(robot_model):
    scene = PlanningScene(robot_model)
    state = initial_state(scene)
    state.set_joint_group_positions("arm", [
        -1.470336642, 2.083213825, 0.604038899,
        1.362070388, -0.156979343, -1.430322992,
    ])
    state.update()
    result = CollisionResult()
    scene.check_self_collision(CollisionRequest(), result, state)
    assert result.collision
