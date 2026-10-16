"""Check conversion of the actual simulator wrapper, including mimic joints."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro


@pytest.mark.parametrize("gripper", ["true", "false"])
def test_gazebo_articulation(tmp_path, gripper):
    share = Path(get_package_share_directory("arctos_gazebo"))
    xml = xacro.process_file(str(share / "urdf/gazebo.urdf.xacro"),
                             mappings={"include_gripper": gripper}).toxml()
    urdf = ET.fromstring(xml)
    assert urdf.findtext("ros2_control/hardware/plugin") == "gz_ros2_control/GazeboSimSystem"
    path = tmp_path / "robot.urdf"
    path.write_text(xml)
    result = subprocess.run(["gz", "sdf", "-p", str(path)],
                            capture_output=True, text=True, timeout=30, check=True)
    model = ET.fromstring(result.stdout).find("model")
    assert model.findtext("joint[@name='world_joint']/parent") == "world"
    assert len(model.findall("joint[@type='revolute']")) == 6
    assert len(model.findall("joint[@type='prismatic']")) == (2 if gripper == "true" else 0)
    if gripper == "true":
        mimic = model.find("joint[@name='Right_jaw_joint']/axis/mimic")
        assert mimic is not None
        assert mimic.get("joint") == "Left_jaw_joint"
    assert model.find("plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']") is not None
