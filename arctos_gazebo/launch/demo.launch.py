"""Gazebo Harmonic, ros2_control and optional MoveIt/RViz."""

from pathlib import Path
import importlib.util

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def setup(context):
    share = Path(get_package_share_directory("arctos_gazebo"))
    config = Path(get_package_share_directory("arctos_moveit_config"))
    gripper = LaunchConfiguration("include_gripper").perform(context).lower()
    gui = LaunchConfiguration("gui").perform(context).lower()
    render_engine = LaunchConfiguration("render_engine").perform(context)
    if render_engine not in ("ogre", "ogre2"):
        raise ValueError("render_engine must be ogre or ogre2")
    if gripper not in ("true", "false") or gui not in ("true", "false"):
        raise ValueError("include_gripper and gui must be true or false")
    mappings = {"include_gripper": gripper}
    moveit = (
        MoveItConfigsBuilder("arctos", package_name="arctos_moveit_config")
        .robot_description(file_path=str(share / "urdf/gazebo.urdf.xacro"), mappings=mappings)
        .robot_description_semantic(file_path="config/mock.srdf.xacro", mappings=mappings)
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    controllers = ["joint_state_broadcaster", "arm_controller"]
    if gripper == "true":
        controllers.append("gripper_controller")
        # Physics can report endpoint noise (~1e-14 m). Jazzy's bounds adapter
        # rejects even that; allow 1 micrometre in planning only. Physical and
        # ros2_control travel bounds remain 0..0.015 m.
        for name in ("Left_jaw_joint", "Right_jaw_joint"):
            moveit.joint_limits["robot_description_planning"]["joint_limits"][name].update(
                has_position_limits=True, min_position=-0.000001, max_position=0.015001)
    else:
        manager = moveit.trajectory_execution["moveit_simple_controller_manager"]
        manager["controller_names"].remove("gripper_controller")
        del manager["gripper_controller"]
        for name in ("Left_jaw_joint", "Right_jaw_joint"):
            moveit.joint_limits["robot_description_planning"]["joint_limits"].pop(name)
    sim_time = {"use_sim_time": True}
    scene_file = LaunchConfiguration("scene_config").perform(context)
    objects = []
    if scene_file:
        spec = importlib.util.spec_from_file_location("scene_objects", share / "scripts/scene_objects.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        scene = module.load_scene(scene_file)
        for name in module.object_names(scene):
            objects.append(Node(package="ros_gz_sim", executable="create", output="screen",
                                arguments=["-world", "arctos_world", "-name", name,
                                           "-string", module.model_sdf(name, scene[name]),
                                           "-x", str(scene[name]["position"][0]),
                                           "-y", str(scene[name]["position"][1]),
                                           "-z", str(scene[name]["position"][2])]))
    return objects + [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory("ros_gz_sim")) / "launch/gz_sim.launch.py")),
            launch_arguments={"gz_args": f'-r {"" if gui == "true" else "-s"} --render-engine-gui {render_engine} {share / "worlds/empty.sdf"}',
                              "on_exit_shutdown": "true"}.items()),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                        "/arctos/sim_joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                         "/cube/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                         "/support_cube/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
             remappings=[("/arctos/sim_joint_states", "/gazebo/joint_states")]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[moveit.robot_description, sim_time], output="screen"),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-world", "arctos_world", "-name", "arctos",
                        "-topic", "/robot_description"]),
        # Gazebo owns controller_manager. The spawner waits for its service.
        Node(package="controller_manager", executable="spawner", output="screen",
             arguments=controllers + ["--controller-manager-timeout", "90"],
             parameters=[sim_time]),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             condition=IfCondition(LaunchConfiguration("moveit")),
             parameters=[moveit.to_dict(), sim_time]),
        Node(package="arctos_gazebo", executable="planning_ground.py", output="screen",
             condition=IfCondition(LaunchConfiguration("moveit")),
             parameters=[sim_time, {"scene_config": scene_file}]),
        Node(package="rviz2", executable="rviz2", output="screen",
             condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", str(config / "config/moveit.rviz")],
             parameters=[moveit.to_dict(), sim_time]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("render_engine", default_value="ogre",
                              description="Gazebo GUI renderer: ogre for OpenGL 2.1 VMs, ogre2 for OpenGL 3.3+"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("moveit", default_value="true"),
        DeclareLaunchArgument("include_gripper", default_value="true"),
        DeclareLaunchArgument("scene_config", default_value="",
                              description="Optional manipulation YAML (empty for the original scene)"),
        OpaqueFunction(function=setup),
    ])
