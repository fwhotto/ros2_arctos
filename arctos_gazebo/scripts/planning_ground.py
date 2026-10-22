#!/usr/bin/env python3
"""Maintain ground/table and live Gazebo objects in MoveIt's collision world.

The grasp script pauses cube updates while MoveIt owns attached geometry and
resumes them after detachment. This never attaches the cube in Gazebo.
"""

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import rclpy
from shape_msgs.msg import Plane
from shape_msgs.msg import SolidPrimitive
from tf2_msgs.msg import TFMessage
from scene_objects import load_scene, object_names
import time
from std_srvs.srv import SetBool


def main():
    rclpy.init()
    node = rclpy.create_node("planning_ground")
    path = node.declare_parameter("scene_config", "").value
    config = load_scene(path) if path else None
    dynamic_names = [name for name in object_names(config) if name != "table"] if config else []
    latest = {}
    paused = False
    def pause_sync(request, response):
        nonlocal paused
        paused = request.data
        response.success = True
        response.message = "Cube synchronization paused" if paused else "Cube synchronization resumed"
        return response
    if config:
        node.create_service(SetBool, "/manipulation/pause_cube_sync", pause_sync)
    def receive(msg):
        for transform in msg.transforms:
            if transform.child_frame_id in dynamic_names:
                if transform.header.frame_id != "arctos_world":
                    raise RuntimeError("Object pose is not expressed in Gazebo world")
                latest[transform.child_frame_id] = (transform.transform, time.monotonic())
    if config:
        for name in dynamic_names:
            node.create_subscription(TFMessage, f"/{name}/pose", receive, 10)
    try:
        client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        if not client.wait_for_service(timeout_sec=90):
            raise RuntimeError("MoveIt planning scene unavailable")
        ground = CollisionObject(id="ground", operation=CollisionObject.ADD)
        ground.header.frame_id = "world"
        ground.planes = [Plane(coef=[0.0, 0.0, 1.0, 0.001])]
        pose = Pose()
        pose.orientation.w = 1.0
        ground.plane_poses = [pose]
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [ground]
        if config:
            table = CollisionObject(id="table", operation=CollisionObject.ADD)
            table.header.frame_id = "world"
            table.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=config["table"]["size"])]
            table_pose = Pose()
            table_pose.orientation.w = 1.0
            table_pose.position.x, table_pose.position.y, table_pose.position.z = config["table"]["position"]
            table.primitive_poses = [table_pose]
            scene.world.collision_objects.append(table)
        future = client.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(node, future, timeout_sec=20)
        if not future.done() or not future.result().success:
            raise RuntimeError("Failed to install planning ground")
        node.get_logger().info("Planning ground ready at z=-0.001 m")
        if config:
            deadline = time.monotonic() + 30
            ready = False
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                if not all(name in latest for name in dynamic_names):
                    if time.monotonic() > deadline:
                        raise RuntimeError("Missing Gazebo object pose: " + str(set(dynamic_names) - latest.keys()))
                    continue
                if any(time.monotonic() - latest[name][1] > 2 for name in dynamic_names):
                    raise RuntimeError("Gazebo object pose stream became stale")
                update = PlanningScene(is_diff=True)
                for name in dynamic_names:
                    if paused and name == "cube":
                        continue
                    transform = latest[name][0]
                    cube = CollisionObject(id=name, operation=CollisionObject.ADD)
                    cube.header.frame_id = "world"
                    cube.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=config[name]["size"])]
                    pose = Pose()
                    pose.position.x = transform.translation.x
                    pose.position.y = transform.translation.y
                    pose.position.z = transform.translation.z
                    pose.orientation = transform.rotation
                    cube.primitive_poses = [pose]
                    update.world.collision_objects.append(cube)
                future = client.call_async(ApplyPlanningScene.Request(scene=update))
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                if not future.done() or not future.result().success:
                    raise RuntimeError("Object planning-scene update failed")
                if not ready:
                    node.get_logger().info("Manipulation scene ready: object poses follow Gazebo")
                    ready = True
                # Keep receiving poses while limiting scene updates to 5 Hz.
                until = time.monotonic() + 0.2
                while time.monotonic() < until and rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
