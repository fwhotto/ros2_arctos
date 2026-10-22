#!/usr/bin/env python3
"""Gazebo-only approach, grasp, transfer, stack and push checkpoints.

Only use with the Gazebo manipulation demo, not physical hardware. A failed
grasp leaves the simulation for inspection; restart it before another attempt.
"""

import argparse
import copy
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import ExecuteTrajectory
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, PlanningSceneComponents, RobotState, PositionConstraint, BoundingVolume
from moveit_msgs.srv import GetPlanningScene, GetPositionIK, GetPositionFK, GetMotionPlan, GetCartesianPath, GetStateValidity
import numpy as np
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import SetBool
from moveit_msgs.msg import PlanningScene, AttachedCollisionObject, CollisionObject, AllowedCollisionEntry
from moveit_msgs.srv import ApplyPlanningScene
import yaml


def push_collision_matrix(original):
    """Permit only intentional upper-cube/jaw contact, preserving other pairs."""
    acm = copy.deepcopy(original)
    for name in ("cube", "Left_jaw_1", "Right_jaw_1"):
        if name not in acm.entry_names:
            acm.entry_names.append(name)
            for row in acm.entry_values:
                row.enabled.append(False)
            acm.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(acm.entry_names)))
    for jaw in ("Left_jaw_1", "Right_jaw_1"):
        i, j = acm.entry_names.index("cube"), acm.entry_names.index(jaw)
        acm.entry_values[i].enabled[j] = acm.entry_values[j].enabled[i] = True
    return acm


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--grasp", action="store_true", help="With --execute: close, lift, set down and release")
    stages = parser.add_mutually_exclusive_group()
    stages.add_argument("--transfer", action="store_true", help="With --execute: carry 40 mm sideways and release on table")
    stages.add_argument("--stack", action="store_true", help="With --execute: stack the red cube on the blue cube")
    stages.add_argument("--push", action="store_true", help="With --execute: stack, check stability, then push the upper cube off")
    parser.add_argument("--scene-config", help="Must match the running launch; defaults to stacking.yaml for the new stages")
    args = parser.parse_args()
    extended = args.transfer or args.stack or args.push
    args.grasp = args.grasp or extended
    if args.grasp and not args.execute:
        parser.error("--grasp/--transfer/--stack/--push requires --execute")
    if not args.scene_config:
        args.scene_config = str(Path(get_package_share_directory("arctos_gazebo")) / "config" /
                                ("stacking.yaml" if extended else "manipulation.yaml"))
    config = yaml.safe_load(Path(args.scene_config).read_text())
    if (args.stack or args.push) and "support_cube" not in config:
        parser.error("Stacking requires a support_cube in the scene configuration")
    if extended:
        for name in ("cube", "support_cube") if args.stack or args.push else ("cube",):
            if not np.allclose(config[name]["size"], [0.02, 0.02, 0.02]):
                parser.error("Transfer/stack/push currently require 20 mm cubes")
    # This is a positional simulation squeeze, not calibrated force control.
    squeeze = 0.0005
    closing = (0.03 - config["cube"]["size"][1])/2 + squeeze
    if args.grasp and not 0.001 < closing < 0.014:
        parser.error("Cube width must leave usable jaw travel for the grasp")
    rclpy.init()
    node = rclpy.create_node("manipulation_checkpoint", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    latest = {}
    sync_paused = False
    gazebo_cube = {}
    gazebo_support = {}
    saved_acm = None
    node.create_subscription(JointState, "/joint_states", lambda msg: latest.update(zip(msg.name, msg.position)), 10)
    def receive_cube(msg):
        for transform in msg.transforms:
            if transform.child_frame_id == "cube":
                assert transform.header.frame_id == "arctos_world", "Unexpected cube reference frame"
                p = transform.transform.translation
                gazebo_cube["position"] = [p.x, p.y, p.z]
                gazebo_cube["rotation"] = transform.transform.rotation
                gazebo_cube["received"] = time.monotonic()
    node.create_subscription(TFMessage, "/cube/pose", receive_cube, 10)
    def receive_support(msg):
        for transform in msg.transforms:
            if transform.child_frame_id == "support_cube":
                assert transform.header.frame_id == "arctos_world", "Unexpected support reference frame"
                p = transform.transform.translation
                gazebo_support["position"] = [p.x, p.y, p.z]
                gazebo_support["rotation"] = transform.transform.rotation
                gazebo_support["received"] = time.monotonic()
    node.create_subscription(TFMessage, "/support_cube/pose", receive_support, 10)

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
        # grasp_frame local translation (-.00129,-.066,.001367), Rx(pi/2).
        # Desired grasp rotation Rx(pi) points +Z down; tool0 is Rx(pi/2).
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = center[0] + 0.00129
        pose.pose.position.y = center[1] + 0.001367
        pose.pose.position.z = center[2] + 0.066
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

    def spin_seconds(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

    def gripper(position):
        action = ActionClient(node, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
        assert action.wait_for_server(timeout_sec=15)
        trajectory = JointTrajectory(joint_names=["Left_jaw_joint"], points=[
            JointTrajectoryPoint(positions=[position], time_from_start=Duration(sec=2))])
        future = action.send_goal_async(FollowJointTrajectory.Goal(trajectory=trajectory))
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        assert future.done() and future.result().accepted
        done = future.result().get_result_async()
        rclpy.spin_until_future_complete(node, done, timeout_sec=20)
        assert done.done() and done.result().result.error_code == 0, "Gripper trajectory failed"
        action.destroy()
        spin_seconds(1)

    def grasp_position():
        fk = GetPositionFK.Request()
        fk.header.frame_id = "world"
        fk.fk_link_names = ["grasp_frame"]
        fk.robot_state = state(latest)
        actual = call(GetPositionFK, "/compute_fk", fk)
        assert actual.error_code.val == MoveItErrorCodes.SUCCESS
        assert abs(actual.pose_stamped[0].pose.orientation.x) > 0.999, "Gripper tilted during Cartesian motion"
        p = actual.pose_stamped[0].pose.position
        return np.array([p.x, p.y, p.z])

    def cartesian(center):
        # Short, straight motions at fixed downward orientation, including push.
        cart = GetCartesianPath.Request()
        cart.header.frame_id = "world"
        cart.group_name = "arm"
        cart.link_name = "tool0"
        cart.start_state = state(latest)
        cart.max_step = 0.002
        cart.avoid_collisions = True
        cart.max_velocity_scaling_factor = 0.05
        cart.max_acceleration_scaling_factor = 0.05
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, center + [0.00129, 0.001367, 0.066])
        pose.orientation.x = float(np.sqrt(0.5))
        pose.orientation.w = float(np.sqrt(0.5))
        cart.waypoints = [pose]
        response = call(GetCartesianPath, "/compute_cartesian_path", cart)
        assert response.error_code.val == MoveItErrorCodes.SUCCESS and response.fraction > 0.999, ("Cartesian path", center, response.error_code, response.fraction)
        execute(response.solution)
        spin_seconds(1)
        np.testing.assert_allclose(grasp_position(), center, atol=0.003,
                                   err_msg="Gripper did not reach Cartesian endpoint")

    def apply(scene):
        assert call(ApplyPlanningScene, "/apply_planning_scene", ApplyPlanningScene.Request(scene=scene)).success

    def check_position(feedback, expected, tolerance=0.003):
        assert time.monotonic() - feedback.get("received", 0) < 1, "Gazebo object feedback stale"
        np.testing.assert_allclose(feedback["position"], expected, atol=tolerance)

    def stable(expected, seconds=3, support=None):
        finish = node.get_clock().now().nanoseconds + int(seconds * 1e9)
        deadline = time.monotonic() + max(30, seconds * 10)
        while node.get_clock().now().nanoseconds < finish:
            assert time.monotonic() < deadline, "Simulation clock stopped during stability check"
            rclpy.spin_once(node, timeout_sec=0.05)
            check_position(gazebo_cube, expected)
            if support is not None:
                check_position(gazebo_support, support)
                for feedback in (gazebo_cube, gazebo_support):
                    q = feedback["rotation"]
                    assert 1 - 2*(q.x*q.x + q.y*q.y) > 0.98, "Tower cube tilted"

    try:
        deadline = time.monotonic() + 90
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        while True:
            scene = call(GetPlanningScene, "/get_planning_scene", request).scene
            objects = {obj.id: obj for obj in scene.world.collision_objects}
            required = {"table", "cube", "ground"} | ({"support_cube"} if args.stack or args.push else set())
            if required.issubset(objects) and len(latest) == 8:
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
        support = None
        destination = center.copy()
        if args.transfer:
            destination += [-0.04, 0, 0]
        elif args.stack or args.push:
            support = np.array(gazebo_support["position"])
            expected_support = np.array(config["support_cube"]["position"])
            expected_support[2] = top + config["support_cube"]["size"][2]/2
            check_position(gazebo_support, expected_support)
            support_scene = call(GetPlanningScene, "/get_planning_scene", request).scene
            support_object = next(obj for obj in support_scene.world.collision_objects if obj.id == "support_cube")
            np.testing.assert_allclose(support_object.primitives[0].dimensions, config["support_cube"]["size"])
            p = support_object.pose.position
            np.testing.assert_allclose([p.x, p.y, p.z], support, atol=0.001)
            destination = support + [0, 0, (config["support_cube"]["size"][2] + config["cube"]["size"][2])/2]
        pre = center + [0, 0, config["approach_clearance"]]
        approach = center if args.grasp else center + [0, 0, config["cube"]["size"][2]/2 + config["surface_clearance"]]
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
        pose.position.x, pose.position.y, pose.position.z = map(float, approach + [0.00129, 0.001367, 0.066])
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
        approach_description = "around cube" if args.grasp else "above cube"
        print(f"PASS: collision-free straight approach {approach_description}" + (" executed; jaws open and cube undisturbed" if args.execute else " planned"), flush=True)
        if args.grasp:
            gripper(closing)
            assert abs(latest["Left_jaw_joint"] - closing) < 0.002, latest
            assert abs(latest["Left_jaw_joint"] - latest["Right_jaw_joint"]) < 0.001, "Follower jaw did not close symmetrically"
            if extended:
                print(f"INFO: positional squeeze {squeeze*1000:.1f} mm; jaw feedback "
                      f"{latest['Left_jaw_joint']*1000:.2f}/{latest['Right_jaw_joint']*1000:.2f} mm", flush=True)
            assert call(SetBool, "/manipulation/pause_cube_sync", SetBool.Request(data=True)).success
            sync_paused = True
            scene = call(GetPlanningScene, "/get_planning_scene", request).scene
            cube = next(obj for obj in scene.world.collision_objects if obj.id == "cube")
            attach = AttachedCollisionObject(link_name="grasp_frame", object=cube,
                                             touch_links=["Left_jaw_1", "Right_jaw_1"])
            update = PlanningScene(is_diff=True)
            update.robot_state.is_diff = True
            update.robot_state.attached_collision_objects = [attach]
            apply(update)
            # MoveIt attachment only updates planning geometry. No Gazebo fixed
            # joint, teleport or attachment service is used: physics must hold it.
            cartesian(center + [0, 0, 0.06])
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                assert time.monotonic() - gazebo_cube["received"] < 1
                np.testing.assert_allclose(gazebo_cube["position"], center + [0, 0, 0.06], atol=0.005,
                                           err_msg="Cube slipped or was not lifted")
            print("PASS: physical grasp lifted cube 60 mm and held for 3 seconds", flush=True)
            if extended:
                carried = destination + [0, 0, 0.06]
                cartesian(carried)
                check_position(gazebo_cube, carried, tolerance=0.005)
                print("PASS: physical grasp survived sideways transfer", flush=True)
            # Leave a sub-millimetre gap over the support to avoid planning a
            # collision. Gravity closes it after release; no Gazebo pose edits.
            release = destination + [0, 0, 0.0005] if args.stack or args.push else destination
            cartesian(release)
            # Remove attached geometry and resume measured world-object updates.
            update = PlanningScene(is_diff=True)
            update.robot_state.is_diff = True
            update.robot_state.attached_collision_objects = [AttachedCollisionObject(
                link_name="grasp_frame", object=CollisionObject(id="cube", operation=CollisionObject.REMOVE))]
            apply(update)
            gripper(0.0)
            assert call(SetBool, "/manipulation/pause_cube_sync", SetBool.Request(data=False)).success
            sync_paused = False
            spin_seconds(1)
            cartesian(destination + [0, 0, config["approach_clearance"]])
            spin_seconds(2)
            stable(destination, support=support)
            assert abs(latest["Left_jaw_joint"]) < 0.0001
            assert abs(latest["Right_jaw_joint"]) < 0.0001
            request.components.components |= PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            final_scene = call(GetPlanningScene, "/get_planning_scene", request).scene
            assert not any(obj.object.id == "cube" for obj in final_scene.robot_state.attached_collision_objects)
            cube = next(obj for obj in final_scene.world.collision_objects if obj.id == "cube")
            p = cube.pose.position
            np.testing.assert_allclose([p.x, p.y, p.z], gazebo_cube["position"], atol=0.001)
            print("PASS: cube released and stable after retreat; planning-scene ownership restored", flush=True)
            if args.stack or args.push:
                print("PASS: two-cube tower stood unsupported for 3 seconds after retreat", flush=True)
            if args.push:
                # The leading open jaw contacts the upper cube along +Y. Its
                # pad bottom clears the lower cube. Only cube/jaw collisions
                # are allowed, temporarily; table and lower cube stay protected.
                request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
                saved_acm = call(GetPlanningScene, "/get_planning_scene", request).scene.allowed_collision_matrix
                acm = push_collision_matrix(saved_acm)
                start_push = destination + [0, -0.045, 0]
                cartesian(start_push + [0, 0, 0.06])
                cartesian(start_push)
                apply(PlanningScene(is_diff=True, allowed_collision_matrix=acm))
                cartesian(destination + [0, 0.005, 0])
                cartesian(destination + [0, 0.005, 0.06])
                apply(PlanningScene(is_diff=True, allowed_collision_matrix=saved_acm))
                saved_acm = None
                spin_seconds(3)
                fallen = np.array(gazebo_cube["position"])
                assert fallen[2] < destination[2] - 0.01, ("Upper cube did not fall", fallen)
                assert np.linalg.norm(fallen[:2] - support[:2]) > 0.015, "Upper cube still over support"
                assert abs(fallen[2] - (top + config["cube"]["size"][2]/2)) < 0.004, "Cube did not land on table"
                stable(fallen, seconds=2)
                print("PASS: physical push displaced upper cube; it fell and settled on the table", flush=True)
    finally:
        try:
            if saved_acm is not None:
                apply(PlanningScene(is_diff=True, allowed_collision_matrix=saved_acm))
        finally:
            if sync_paused:
                node.get_logger().error("Grasp failed while cube was attached in MoveIt. No automatic release attempted; restart the simulation before retrying.")
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
