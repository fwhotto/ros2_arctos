"""Interactive MoveIt planning and execution on mock hardware (no Gazebo)."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context):
    include_gripper = LaunchConfiguration("include_gripper").perform(context).lower()
    if include_gripper not in ("true", "false"):
        raise ValueError("include_gripper must be true or false")
    mappings = {"include_gripper": include_gripper}
    control = Path(get_package_share_directory("arctos_control"))
    config = Path(get_package_share_directory("arctos_moveit_config"))
    moveit = (
        MoveItConfigsBuilder("arctos", package_name="arctos_moveit_config")
        .robot_description(file_path=str(control / "urdf/mock.urdf.xacro"), mappings=mappings)
        .robot_description_semantic(file_path="config/mock.srdf.xacro", mappings=mappings)
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    controllers = ["joint_state_broadcaster", "arm_controller"]
    if include_gripper == "true":
        controllers.append("gripper_controller")
    else:
        manager = moveit.trajectory_execution["moveit_simple_controller_manager"]
        manager["controller_names"].remove("gripper_controller")
        del manager["gripper_controller"]
        for joint in ("Left_jaw_joint", "Right_jaw_joint"):
            moveit.joint_limits["robot_description_planning"]["joint_limits"].pop(joint)
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen", parameters=[moveit.robot_description]),
        Node(package="controller_manager", executable="ros2_control_node",
             output="screen", parameters=[str(control / "config/ros2_controllers.yaml")],
             remappings=[("~/robot_description", "/robot_description")]),
        # The spawner waits for controller_manager; joint-state publication comes
        # from ros2_control alone, not a competing joint_state_publisher.
        Node(package="controller_manager", executable="spawner", output="screen",
             arguments=controllers + [
                        "--controller-manager-timeout", "60"]),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[moveit.to_dict(), {"use_sim_time": False}]),
        Node(package="rviz2", executable="rviz2", output="screen",
             condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", str(config / "config/moveit.rviz")],
             parameters=[moveit.robot_description, moveit.robot_description_semantic,
                         moveit.robot_description_kinematics, moveit.joint_limits,
                         moveit.planning_pipelines, {"use_sim_time": False}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Open the interactive RViz window"),
        DeclareLaunchArgument("include_gripper", default_value="true",
                              description="Include the actuated mimic-jaw gripper"),
        OpaqueFunction(function=launch_setup),
    ])
