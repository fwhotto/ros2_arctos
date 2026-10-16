#!/usr/bin/env python3
"""Mirror the simulation ground in MoveIt's collision world."""

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import rclpy
from shape_msgs.msg import Plane


def main():
    rclpy.init()
    node = rclpy.create_node("planning_ground")
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
        future = client.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(node, future, timeout_sec=20)
        if not future.done() or not future.result().success:
            raise RuntimeError("Failed to install planning ground")
        node.get_logger().info("Planning ground ready at z=-0.001 m")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
