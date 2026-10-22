"""Two-cube physics demo; run scripts/test_manipulation.py separately to move."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("arctos_gazebo"))
    return LaunchDescription([
        DeclareLaunchArgument("scene_config", default_value=str(share / "config/stacking.yaml")),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / "launch/demo.launch.py")),
                                 launch_arguments={"scene_config": LaunchConfiguration("scene_config")}.items()),
    ])
