#!/usr/bin/env python3
"""Plan/execute the arm and open/close the gripper on a running mock_demo launch.

Run on an isolated ROS_DOMAIN_ID shared only with that mock launch.
"""

import argparse
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, PlanningSceneComponents
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
import xacro


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bare", action="store_true", help="Test include_gripper:=false")
    parser.add_argument("--sim-time", action="store_true", help="Use Gazebo /clock")
    parser.add_argument("--hold-seconds", type=float, default=0,
                        help="Verify each target remains stable for this many seconds")
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("moveit_smoke_test", parameter_overrides=[
        rclpy.parameter.Parameter("use_sim_time", value=args.sim_time)])
    latest = {}
    gazebo = {}
    stamps = {}
    def receive(msg, destination, key):
        destination.update(zip(msg.name, msg.position))
        stamps[key] = time.monotonic()
    node.create_subscription(JointState, "/joint_states",
                             lambda msg: receive(msg, latest, "control"), 10)
    if args.sim_time:
        node.create_subscription(JointState, "/gazebo/joint_states",
                                 lambda msg: receive(msg, gazebo, "gazebo"), 10)
    planner = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    executor = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")

    def wait(future, timeout=60):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        assert future.done(), "Timed out waiting for MoveIt"
        return future.result()

    try:
        assert planner.wait_for_service(timeout_sec=45), "MoveIt planner unavailable"
        assert executor.wait_for_server(timeout_sec=30), "Execution action unavailable"
        expected = {f"{axis}_joint" for axis in "XYZABC"}
        if not args.bare:
            expected.update(("Left_jaw_joint", "Right_jaw_joint"))
        deadline = time.monotonic() + 15
        while not expected.issubset(latest) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert set(latest) == expected, f"Unexpected joint states: {latest}"
        if args.sim_time:
            deadline = time.monotonic() + 15
            while (not expected.issubset(gazebo) or node.get_clock().now().nanoseconds == 0) and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            assert expected.issubset(gazebo), "Gazebo joint feedback unavailable"
            assert node.get_clock().now().nanoseconds > 0, "Simulation clock unavailable"
            scene_client = node.create_client(GetPlanningScene, "/get_planning_scene")
            assert scene_client.wait_for_service(timeout_sec=15)
            deadline = time.monotonic() + 15
            while True:
                scene_request = GetPlanningScene.Request()
                scene_request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                scene = wait(scene_client.call_async(scene_request)).scene
                if any(obj.id == "ground" for obj in scene.world.collision_objects):
                    break
                assert time.monotonic() < deadline, "Planning ground missing"
                rclpy.spin_once(node, timeout_sec=0.1)
        share = Path(get_package_share_directory("arctos_moveit_config"))
        srdf = ET.fromstring(xacro.process_file(
            str(share / "config/mock.srdf.xacro"),
            mappings={"include_gripper": "false" if args.bare else "true"},
        ).toxml())
        targets = ("demo", "home") if args.bare else ("demo", "closed", "open", "home")
        for target in targets:
            state = srdf.find(f"group_state[@name='{target}']")
            positions = {j.get("name"): float(j.get("value")) for j in state}
            request = GetMotionPlan.Request()
            motion = request.motion_plan_request
            motion.group_name = state.get("group")
            motion.pipeline_id = "ompl"
            motion.allowed_planning_time = 5.0
            motion.num_planning_attempts = 3
            motion.max_velocity_scaling_factor = 0.2
            motion.max_acceleration_scaling_factor = 0.2
            motion.start_state.is_diff = True
            goal_tolerance = 0.00001 if motion.group_name == "gripper" else 0.001
            motion.goal_constraints = [Constraints(joint_constraints=[
                JointConstraint(joint_name=name, position=value,
                                tolerance_above=goal_tolerance,
                                tolerance_below=goal_tolerance, weight=1.0)
                for name, value in positions.items()
            ])]
            response = wait(planner.call_async(request)).motion_plan_response
            assert response.error_code.val == MoveItErrorCodes.SUCCESS, response.error_code
            assert len(response.trajectory.joint_trajectory.points) > 1
            goal = wait(executor.send_goal_async(
                ExecuteTrajectory.Goal(trajectory=response.trajectory)))
            assert goal.accepted
            result = wait(goal.get_result_async()).result
            assert result.error_code.val == MoveItErrorCodes.SUCCESS, result.error_code
            if "Left_jaw_joint" in positions:
                positions["Right_jaw_joint"] = positions["Left_jaw_joint"]
            tolerance = 0.0001 if motion.group_name == "gripper" else 0.01
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if all(abs(latest[name] - value) < tolerance for name, value in positions.items()):
                    break
            assert all(abs(latest[name] - value) < tolerance for name, value in positions.items()), latest
            print(f"PASS: planned and executed {target}; joint feedback matches", flush=True)
            held = dict(latest)
            start_clock = node.get_clock().now().nanoseconds
            deadline = time.monotonic() + args.hold_seconds
            samples = 0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                assert time.monotonic() - stamps["control"] < 1, "Stale controller feedback"
                for name in expected:
                    allowed = 0.0001 if "jaw" in name else 0.01
                    assert math.isfinite(latest[name])
                    assert abs(latest[name] - held[name]) < allowed, ("Pose drift", name)
                    if args.sim_time:
                        assert time.monotonic() - stamps["gazebo"] < 1, "Stale Gazebo feedback"
                        assert abs(gazebo[name] - latest[name]) < allowed, ("Gazebo mismatch", name)
                samples += 1
            if args.hold_seconds:
                assert samples > 1
                assert node.get_clock().now().nanoseconds > start_clock, "Clock stopped"
                print(f"PASS: held {target} for {args.hold_seconds:g}s; fresh feedback agrees", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
