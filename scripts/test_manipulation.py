#!/usr/bin/env python3
"""Validate the live scene and plan an open-jaw approach; --execute moves the arm."""

import argparse
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, PlanningSceneComponents, RobotState, PositionConstraint, BoundingVolume
from moveit_msgs.srv import GetPlanningScene, GetPositionIK, GetPositionFK, GetMotionPlan, GetCartesianPath, GetStateValidity
import numpy as np
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from shape_msgs.msg import SolidPrimitive
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--scene-config", default=str(Path(get_package_share_directory("arctos_gazebo")) / "config/manipulation.yaml"))
    args = parser.parse_args()
    config = yaml.safe_load(Path(args.scene_config).read_text())
    rclpy.init()
    node = rclpy.create_node("manipulation_checkpoint", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    latest = {}
    gazebo_cube = {}
    node.create_subscription(JointState, "/joint_states", lambda msg: latest.update(zip(msg.name, msg.position)), 10)
    def receive_cube(msg):
        for transform in msg.transforms:
            if transform.child_frame_id == "cube":
                assert transform.header.frame_id == "arctos_world", "Unexpected cube reference frame"
                p = transform.transform.translation
                gazebo_cube["position"] = [p.x, p.y, p.z]
                gazebo_cube["received"] = time.monotonic()
    node.create_subscription(TFMessage, "/cube/pose", receive_cube, 10)

    def call(kind, name, request):
        client = node.create_client(kind, name)
        try:
            assert client.wait_for_service(timeout_sec=60), name
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=40)
            assert future.done(), name
            return future.result()
        finally:
            node.destroy_client(client)

    def state(values):
        result = RobotState()
        result.joint_state.name = list(values)
        result.joint_state.position = list(values.values())
        return result

    def ik(center, seed):
        # grasp_frame local translation (-.00129,-.074,.001367), Rx(pi/2).
        # Desired grasp rotation Rx(pi) points +Z down; tool0 is Rx(pi/2).
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = center[0] + 0.00129
        pose.pose.position.y = center[1] + 0.001367
        pose.pose.position.z = center[2] + 0.074
        pose.pose.orientation.x = float(np.sqrt(0.5))
        pose.pose.orientation.w = float(np.sqrt(0.5))
        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm"
        request.ik_request.ik_link_name = "tool0"
        request.ik_request.pose_stamped = pose
        request.ik_request.robot_state = state(seed)
        request.ik_request.timeout.sec = 5
        request.ik_request.avoid_collisions = False
        response = call(GetPositionIK, "/compute_ik", request)
        assert response.error_code.val == MoveItErrorCodes.SUCCESS, ("IK", center, response.error_code)
        return dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))

    def plan(target, start, clearance=None):
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = "arm"
        motion.pipeline_id = "ompl"
        motion.start_state = state(start)
        motion.allowed_planning_time = 5.0
        motion.max_velocity_scaling_factor = 0.1
        motion.max_acceleration_scaling_factor = 0.1
        if clearance is not None:
            # Keep the wrist high during free-space transit. Mesh checks alone
            # can miss contact with Gazebo's conservative convex hulls.
            region_pose = Pose()
            region_pose.position.z = clearance + 1.0
            region_pose.orientation.w = 1.0
            constraint = PositionConstraint(link_name="tool0", weight=1.0)
            constraint.header.frame_id = "world"
            constraint.constraint_region = BoundingVolume(
                primitives=[SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[2.0, 2.0, 2.0])],
                primitive_poses=[region_pose])
            motion.path_constraints.position_constraints = [constraint]
        motion.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=name, position=float(target[name]), tolerance_above=0.0001,
                            tolerance_below=0.0001, weight=1.0) for name in [f"{a}_joint" for a in "XYZABC"]])]
        return call(GetMotionPlan, "/plan_kinematic_path", request).motion_plan_response

    def execute(trajectory):
        action = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
        assert action.wait_for_server(timeout_sec=30)
        future = action.send_goal_async(ExecuteTrajectory.Goal(trajectory=trajectory))
        rclpy.spin_until_future_complete(node, future, timeout_sec=15)
        assert future.done() and future.result().accepted
        result = future.result().get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=90)
        assert result.done() and result.result().result.error_code.val == MoveItErrorCodes.SUCCESS
        action.destroy()

    try:
        deadline = time.monotonic() + 90
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        while True:
            scene = call(GetPlanningScene, "/get_planning_scene", request).scene
            objects = {obj.id: obj for obj in scene.world.collision_objects}
            if {"table", "cube", "ground"}.issubset(objects) and len(latest) == 8:
                break
            assert time.monotonic() < deadline, "Scene/joints not ready"
            rclpy.spin_once(node, timeout_sec=0.2)
        assert abs(latest["Left_jaw_joint"]) < 0.0001, "Start with open jaws"
        table = objects["table"]
        np.testing.assert_allclose(table.primitives[0].dimensions, config["table"]["size"])
        p = table.pose.position
        np.testing.assert_allclose([p.x, p.y, p.z], config["table"]["position"])
        top = config["table"]["position"][2] + config["table"]["size"][2]/2
        # Check stable, synchronized cube height for two seconds after settling.
        centers = []
        for _ in range(12):
            until = time.monotonic() + 0.2
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.05)
            scene = call(GetPlanningScene, "/get_planning_scene", request).scene
            cube = next(obj for obj in scene.world.collision_objects if obj.id == "cube")
            np.testing.assert_allclose(cube.primitives[0].dimensions, config["cube"]["size"])
            p = cube.pose.position
            centers.append([p.x, p.y, p.z])
            assert time.monotonic() - gazebo_cube.get("received", 0) < 1, "Cube feedback stale"
            np.testing.assert_allclose(centers[-1], gazebo_cube["position"], atol=0.001)
        center = np.array(centers[-1])
        np.testing.assert_allclose(center[:2], config["cube"]["position"][:2], atol=0.002)
        assert abs(center[2] - (top + config["cube"]["size"][2]/2)) < 0.002, centers
        assert np.max(np.ptp(centers[-5:], axis=0)) < 0.001, "Cube not settled"
        print("PASS: cube settled on table; planning scene dimensions/pose consistent", flush=True)
        pre = center + [0, 0, config["approach_clearance"]]
        approach = center + [0, 0, config["cube"]["size"][2]/2 + config["surface_clearance"]]
        target = ik(pre, latest)
        response = plan(target, latest, clearance=float(top + 0.1))
        assert response.error_code.val == MoveItErrorCodes.SUCCESS, response.error_code
        if args.execute:
            execute(response.trajectory)
            np.testing.assert_allclose(gazebo_cube["position"], center, atol=0.001,
                                       err_msg="Cube displaced during pre-grasp transit")
        print("PASS: pre-grasp trajectory" + (" executed" if args.execute else " planned"), flush=True)
        # A target inside the solid tabletop must be rejected, without execution.
        bad = ik(center - [0, 0, 0.04], target)
        validity = call(GetStateValidity, "/check_state_validity", GetStateValidity.Request(robot_state=state(bad), group_name="arm"))
        assert not validity.valid and any("table" in (c.contact_body_1, c.contact_body_2) for c in validity.contacts), "Negative target did not collide with table"
        assert plan(bad, target).error_code.val != MoveItErrorCodes.SUCCESS, "Planner accepted colliding target"
        print("PASS: target intersecting table rejected", flush=True)
        cart = GetCartesianPath.Request()
        cart.header.frame_id = "world"
        cart.group_name = "arm"
        cart.link_name = "tool0"
        cart.start_state = state(target)
        cart.max_step = 0.002
        cart.avoid_collisions = True
        cart.max_velocity_scaling_factor = 0.1
        cart.max_acceleration_scaling_factor = 0.1
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, approach + [0.00129, 0.001367, 0.074])
        pose.orientation.x = float(np.sqrt(0.5))
        pose.orientation.w = float(np.sqrt(0.5))
        cart.waypoints = [pose]
        result = call(GetCartesianPath, "/compute_cartesian_path", cart)
        if result.fraction < 0.999:
            final = ik(approach, target)
            validity = call(GetStateValidity, "/check_state_validity", GetStateValidity.Request(robot_state=state(final), group_name="arm"))
            print("Approach diagnostic:", [(c.contact_body_1, c.contact_body_2) for c in validity.contacts], flush=True)
        assert result.error_code.val == MoveItErrorCodes.SUCCESS and result.fraction > 0.999, (result.error_code, result.fraction)
        if args.execute:
            execute(result.solution)
            until = time.monotonic() + 2
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.05)
            fk = GetPositionFK.Request()
            fk.header.frame_id = "world"
            fk.fk_link_names = ["grasp_frame"]
            fk.robot_state = state(latest)
            actual = call(GetPositionFK, "/compute_fk", fk)
            assert actual.error_code.val == MoveItErrorCodes.SUCCESS
            p = actual.pose_stamped[0].pose.position
            endpoint = dict(zip(result.solution.joint_trajectory.joint_names, result.solution.joint_trajectory.points[-1].positions))
            assert all(abs(latest[name]-value) < 0.01 for name, value in endpoint.items()), ("Arm did not reach approach endpoint", {name: latest[name]-value for name, value in endpoint.items()}, latest)
            np.testing.assert_allclose([p.x, p.y, p.z], approach, atol=0.003)
            actual_q = actual.pose_stamped[0].pose.orientation
            assert abs(actual_q.x) > 0.999, "Grasp frame is not pointing down"
            assert abs(latest["Left_jaw_joint"]) < 0.0001, latest
            assert abs(latest["Right_jaw_joint"]) < 0.0001, latest
            assert time.monotonic() - gazebo_cube["received"] < 1
            np.testing.assert_allclose(gazebo_cube["position"], center, atol=0.001,
                                       err_msg="Cube displaced during approach")
        print("PASS: collision-free straight approach above cube" + (" executed; jaws open and cube undisturbed" if args.execute else " planned"), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
