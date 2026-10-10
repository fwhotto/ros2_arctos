"""Interactive MoveIt planning and execution on mock hardware (no Gazebo)."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    control = Path(get_package_share_directory("arctos_control"))
    config = Path(get_package_share_directory("arctos_moveit_config"))
    moveit = (
        MoveItConfigsBuilder("arctos", package_name="arctos_moveit_config")
        .robot_description(file_path=str(control / "urdf/mock.urdf.xacro"))
        .robot_description_semantic(file_path="config/arctos.srdf")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Open the interactive RViz window"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen", parameters=[moveit.robot_description]),
        Node(package="controller_manager", executable="ros2_control_node",
             output="screen", parameters=[str(control / "config/ros2_controllers.yaml")],
             remappings=[("~/robot_description", "/robot_description")]),
        # The spawner waits for controller_manager; joint-state publication comes
        # from ros2_control alone, not a competing joint_state_publisher.
        Node(package="controller_manager", executable="spawner", output="screen",
             arguments=["joint_state_broadcaster", "arm_controller",
                        "--controller-manager-timeout", "60"]),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[moveit.to_dict(), {"use_sim_time": False}]),
        Node(package="rviz2", executable="rviz2", output="screen",
             condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", str(config / "config/moveit.rviz")],
             parameters=[moveit.robot_description, moveit.robot_description_semantic,
                         moveit.robot_description_kinematics, moveit.joint_limits,
                         moveit.planning_pipelines, {"use_sim_time": False}]),
    ])
