#!/usr/bin/env python3
"""Mirror ground/table and the live Gazebo cube in MoveIt's collision world.

Cube updates are for this pre-contact checkpoint only; pause/remove this
updater before implementing MoveIt attached-object ownership during grasping.
"""

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import rclpy
from shape_msgs.msg import Plane
from shape_msgs.msg import SolidPrimitive
from tf2_msgs.msg import TFMessage
from scene_objects import load_scene
import time


def main():
    rclpy.init()
    node = rclpy.create_node("planning_ground")
    path = node.declare_parameter("scene_config", "").value
    config = load_scene(path) if path else None
    latest = {}
    def receive(msg):
        for transform in msg.transforms:
            if transform.child_frame_id == "cube":
                if transform.header.frame_id != "arctos_world":
                    raise RuntimeError("Cube pose is not expressed in Gazebo world")
                latest["pose"] = transform.transform
                latest["time"] = time.monotonic()
    if config:
        node.create_subscription(TFMessage, "/cube/pose", receive, 10)
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
                if "pose" not in latest:
                    if time.monotonic() > deadline:
                        raise RuntimeError("No Gazebo cube pose received")
                    continue
                if time.monotonic() - latest["time"] > 2:
                    raise RuntimeError("Gazebo cube pose stream became stale")
                cube = CollisionObject(id="cube", operation=CollisionObject.ADD)
                cube.header.frame_id = "world"
                cube.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=config["cube"]["size"])]
                pose = Pose()
                pose.position.x = latest["pose"].translation.x
                pose.position.y = latest["pose"].translation.y
                pose.position.z = latest["pose"].translation.z
                pose.orientation = latest["pose"].rotation
                cube.primitive_poses = [pose]
                update = PlanningScene(is_diff=True)
                update.world.collision_objects = [cube]
                future = client.call_async(ApplyPlanningScene.Request(scene=update))
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                if not future.done() or not future.result().success:
                    raise RuntimeError("Cube planning-scene update failed")
                if not ready:
                    node.get_logger().info("Manipulation scene ready: cube pose follows Gazebo")
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
