"""Gate A checks dependency boundaries and installed assets, not robot motion."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_share_directory


ROOT = Path(__file__).resolve().parents[2]
PACKAGES = {
    "arctos_description", "arctos_control", "arctos_moveit_config", "arctos_gazebo"
}
FORBIDDEN = {
    "arctos_bringup", "arctos_hardware_interface", "arctos_motor_driver",
    "gazebo_ros", "gazebo_ros_control", "gazebo_ros2_control", "warehouse_ros_mongo",
}


def dependencies(package):
    manifest = ET.parse(ROOT / package / "package.xml").getroot()
    return {
        element.text.strip() for element in manifest
        if element.tag.endswith("depend")
    }


def test_simulation_dependency_closure():
    pending = ["arctos_gazebo"]
    visited = set()
    while pending:
        package = pending.pop()
        if package in visited:
            continue
        visited.add(package)
        deps = dependencies(package)
        assert not deps & FORBIDDEN, (package, deps & FORBIDDEN)
        for dependency in deps:
            if (ROOT / dependency / "package.xml").exists():
                pending.append(dependency)
    assert visited == PACKAGES


@pytest.mark.parametrize("package", sorted(PACKAGES))
def test_ros_dependencies_are_available(package):
    for dependency in dependencies(package):
        if dependency == "python3-pytest":
            continue  # rosdep system dependency, not an ament package
        assert Path(get_package_share_directory(dependency)).is_dir()


@pytest.mark.parametrize("package", ["arctos_description", "arctos_control"])
def test_core_has_no_simulator_dependency(package):
    assert not any(
        dependency.startswith(("gz_", "ros_gz", "gazebo", "ignition"))
        for dependency in dependencies(package)
    )


@pytest.mark.parametrize("package", sorted(PACKAGES))
def test_resources_are_installed(package):
    source = ROOT / package
    installed = Path(get_package_share_directory(package))
    assert (installed / "package.xml").is_file()
    # Cover existing resources and future additions to these conventional dirs.
    for directory in ("urdf", "meshes", "rviz", "launch", "config", "worlds"):
        for resource in (source / directory).rglob("*"):
            if resource.is_file() and "__pycache__" not in resource.parts:
                target = installed / resource.relative_to(source)
                assert target.is_file(), str(target)
                assert target.read_bytes() == resource.read_bytes(), str(target)
    if (source / ".setup_assistant").exists():
        assert (installed / ".setup_assistant").is_file()
