"""Validate the optional gripper against the actual MoveIt semantics and meshes."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from moveit.core.collision_detection import CollisionRequest, CollisionResult
from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_model import RobotModel
import numpy as np
import pytest
import xacro
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


@pytest.fixture(scope="module")
def model(tmp_path_factory):
    directory = tmp_path_factory.mktemp("gripper")
    control = Path(get_package_share_directory("arctos_control"))
    config = Path(get_package_share_directory("arctos_moveit_config"))
    urdf = xacro.process_file(str(control / "urdf/mock.urdf.xacro")).toxml()
    srdf = xacro.process_file(str(config / "config/mock.srdf.xacro")).toxml()
    (directory / "robot.urdf").write_text(urdf)
    (directory / "robot.srdf").write_text(srdf)
    return (RobotModel(str(directory / "robot.urdf"), str(directory / "robot.srdf")),
            ET.fromstring(urdf), ET.fromstring(srdf))


def test_one_gripper_actuator_and_mimic(model):
    _, urdf, _ = model
    mimic = urdf.find("joint[@name='Right_jaw_joint']/mimic")
    assert mimic.attrib == {"joint": "Left_jaw_joint", "multiplier": "1", "offset": "0"}
    control = urdf.find("ros2_control")
    assert control.find("joint[@name='Right_jaw_joint']/command_interface") is None
    assert control.find("joint[@name='Left_jaw_joint']/command_interface").get("name") == "position"


@pytest.mark.parametrize("arm_pose", ["home", "demo"])
def test_gripper_sweep_and_symmetric_motion(model, arm_pose):
    robot, _, srdf = model
    scene = PlanningScene(robot)
    state = scene.current_state
    state.set_to_default_values()
    joints = {j.get("name"): float(j.get("value"))
              for j in srdf.find(f"group_state[@name='{arm_pose}']")}
    for closing in np.linspace(0, 0.015, 16):
        state.joint_positions = {**joints, "Left_jaw_joint": float(closing)}
        state.update()
        result = CollisionResult()
        scene.check_self_collision(CollisionRequest(), result, state)
        assert not result.collision, (arm_pose, closing)
        # Express both moving frames in the body frame to verify the axis
        # signs and mimic behavior independently of the arm's orientation.
        inverse = np.linalg.inv(state.get_global_link_transform("Gripper_1"))
        left = inverse @ state.get_global_link_transform("Left_jaw_1")
        right = inverse @ state.get_global_link_transform("Right_jaw_1")
        np.testing.assert_allclose(left[:3, 3], [0, 0, -closing], atol=1e-9)
        np.testing.assert_allclose(right[:3, 3], [0, 0, closing], atol=1e-9)


def test_manufacturer_travel_with_elbow_sign_conversion(model):
    _, urdf, _ = model
    expected = [(-200, 200), (-70, 120), (-110, 40), (-180, 180), (-100, 100), (-180, 180)]
    for axis, bounds in zip("XYZABC", expected):
        limit = urdf.find(f"joint[@name='{axis}_joint']/limit")
        np.testing.assert_allclose(np.rad2deg([float(limit.get("lower")), float(limit.get("upper"))]),
                                   bounds, atol=1e-8)


def test_cube_fits_open_pads_but_contacts_closed_pads(model):
    robot, urdf, _ = model
    scene = PlanningScene(robot)
    state = scene.current_state
    state.set_to_default_values()
    state.update()
    center = state.get_global_link_transform("grasp_frame")[:3, 3]
    cube = CollisionObject(id="cube", operation=CollisionObject.ADD)
    cube.header.frame_id = "world"
    cube.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.02]*3)]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = center
    pose.orientation.w = 1.0
    cube.primitive_poses = [pose]
    scene.apply_collision_object(cube)
    result = CollisionResult()
    scene.check_collision(CollisionRequest(), result, state)
    assert not result.collision
    state.joint_positions = {"Left_jaw_joint": 0.0075}
    state.update()
    result = CollisionResult()
    scene.check_collision(CollisionRequest(), result, state)
    assert result.collision
    for name in ("Left_jaw_1", "Right_jaw_1"):
        link = urdf.find(f"link[@name='{name}']")
        assert {c.get("name") for c in link.findall("collision")} == {"pad", "rail"}
        assert all(c.find("geometry/mesh") is None for c in link.findall("collision"))
