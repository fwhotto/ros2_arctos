"""Shared scene generation and provisional grasp-frame contract."""

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
import xacro
import yaml


SHARE = Path(get_package_share_directory("arctos_gazebo"))
spec = importlib.util.spec_from_file_location("scene_objects", SHARE / "scripts/scene_objects.py")
scene_objects = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scene_objects)


def test_generated_scene_and_support():
    config = scene_objects.load_scene(SHARE / "config/manipulation.yaml")
    for name in ("table", "cube"):
        model = ET.fromstring(scene_objects.model_sdf(name, config[name])).find("model")
        assert model.findtext("static") == ("true" if name == "table" else "false")
        np.testing.assert_allclose(np.fromstring(model.findtext("pose"), sep=" ")[:3], config[name]["position"])
        for kind in ("visual", "collision"):
            np.testing.assert_allclose(np.fromstring(model.findtext(f"link/{kind}/geometry/box/size"), sep=" "), config[name]["size"])
        if name == "cube":
            assert float(model.findtext("link/inertial/mass")) == config[name]["mass"]
            assert model.findtext("plugin/topic") == "/cube/pose"
            assert float(model.findtext("link/inertial/inertia/ixx")) > 0
    top = config["table"]["position"][2] + config["table"]["size"][2]/2
    bottom = config["cube"]["position"][2] - config["cube"]["size"][2]/2
    assert 0 <= bottom - top < 0.01
    assert max(config["cube"]["size"]) < 0.03


@pytest.mark.parametrize("field,value", [("size", [0, 1, 1]), ("position", [0, float("nan"), 0])])
def test_bad_config_rejected(tmp_path, field, value):
    config = scene_objects.load_scene(SHARE / "config/manipulation.yaml")
    config["cube"][field] = value
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError):
        scene_objects.load_scene(path)


def test_grasp_frame_matches_approach_transform():
    description = Path(get_package_share_directory("arctos_description"))
    robot = ET.fromstring(xacro.process_file(str(description / "urdf/arctos.xacro"),
                                            mappings={"include_gripper": "true"}).toxml())
    joint = robot.find("joint[@name='grasp_frame_joint']")
    assert joint.find("parent").get("link") == "tool0"
    np.testing.assert_allclose(np.fromstring(joint.find("origin").get("xyz"), sep=" "),
                               [-0.00129, -0.066, 0.001367])
    np.testing.assert_allclose(np.fromstring(joint.find("origin").get("rpy"), sep=" "),
                                [np.pi/2, 0, 0])


def test_stacking_objects_are_dynamic_and_independently_observable():
    config = scene_objects.load_scene(SHARE / "config/stacking.yaml")
    assert scene_objects.object_names(config) == ["table", "cube", "support_cube"]
    for name in ("cube", "support_cube"):
        model = ET.fromstring(scene_objects.model_sdf(name, config[name])).find("model")
        assert model.findtext("static") == "false"
        assert model.findtext("plugin/topic") == f"/{name}/pose"
        assert float(model.findtext("link/inertial/mass")) > 0
        assert float(model.findtext("link/collision/surface/friction/bullet/friction")) == config[name]["friction"]
        assert float(model.findtext("link/collision/surface/friction/ode/mu")) == config[name]["friction"]
        assert model.find("joint") is None
        top = config["table"]["position"][2] + config["table"]["size"][2]/2
        assert 0 <= config[name]["position"][2] - config[name]["size"][2]/2 - top < 0.01


@pytest.mark.parametrize("mass", [0, -1, float("nan")])
def test_bad_support_mass_rejected(tmp_path, mass):
    config = scene_objects.load_scene(SHARE / "config/stacking.yaml")
    config["support_cube"]["mass"] = mass
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError):
        scene_objects.load_scene(path)


@pytest.mark.parametrize("friction", [-1, float("nan")])
def test_bad_friction_rejected(tmp_path, friction):
    config = scene_objects.load_scene(SHARE / "config/stacking.yaml")
    config["cube"]["friction"] = friction
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError):
        scene_objects.load_scene(path)


def test_push_allows_only_cube_jaw_contact():
    from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix

    script = Path(__file__).resolve().parents[2] / "scripts/test_manipulation.py"
    spec = importlib.util.spec_from_file_location("manipulation_checkpoint", script)
    checkpoint = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checkpoint)
    names = ["table", "support_cube", "Left_jaw_1", "Right_jaw_1"]
    original = AllowedCollisionMatrix(entry_names=names, entry_values=[
        AllowedCollisionEntry(enabled=[False]*len(names)) for _ in names])
    original.entry_values[2].enabled[3] = original.entry_values[3].enabled[2] = True
    updated = checkpoint.push_collision_matrix(original)
    assert original.entry_names == names
    assert all(len(row.enabled) == len(updated.entry_names) for row in updated.entry_values)
    for i, left in enumerate(updated.entry_names):
        for j, right in enumerate(updated.entry_names):
            pair = {left, right}
            allowed = pair in ({"cube", "Left_jaw_1"}, {"cube", "Right_jaw_1"}, {"Left_jaw_1", "Right_jaw_1"})
            assert updated.entry_values[i].enabled[j] == allowed


def test_gazebo_defaults_keep_planning_without_rviz():
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions

    spec = importlib.util.spec_from_file_location("gazebo_demo", SHARE / "launch/demo.launch.py")
    demo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(demo)
    defaults = {action.name: perform_substitutions(LaunchContext(), action.default_value)
                for action in demo.generate_launch_description().entities
                if isinstance(action, DeclareLaunchArgument)}
    assert defaults["rviz"] == "false"
    assert defaults["gui"] == defaults["moveit"] == "true"


@pytest.mark.parametrize("stage", ["transfer", "stack", "push"])
def test_physics_stages_require_explicit_execution(stage):
    import subprocess
    import sys

    script = Path(__file__).resolve().parents[2] / "scripts/test_manipulation.py"
    result = subprocess.run([sys.executable, str(script), f"--{stage}"],
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 2
    assert "requires --execute" in result.stderr
