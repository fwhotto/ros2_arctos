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
