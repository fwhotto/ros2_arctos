#!/usr/bin/env python3
"""Plan/execute home -> demo -> home on a running mock_demo launch.

Run on an isolated ROS_DOMAIN_ID shared only with that mock launch.
"""

import time
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState


def main():
    rclpy.init()
    node = rclpy.create_node("mock_moveit_smoke_test")
    latest = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda msg: latest.update(zip(msg.name, msg.position)), 10)
    planner = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    executor = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")

    def wait(future, timeout=60):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        assert future.done(), "Timed out waiting for MoveIt"
        return future.result()

    try:
        assert planner.wait_for_service(timeout_sec=45), "MoveIt planner unavailable"
        assert executor.wait_for_server(timeout_sec=30), "Execution action unavailable"
        deadline = time.monotonic() + 15
        while len(latest) < 6 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert len(latest) == 6, f"Missing joint states: {latest}"
        share = Path(get_package_share_directory("arctos_moveit_config"))
        srdf = ET.parse(share / "config/arctos.srdf")
        for target in ("demo", "home"):
            state = srdf.find(f"group_state[@name='{target}']")
            positions = {j.get("name"): float(j.get("value")) for j in state}
            request = GetMotionPlan.Request()
            motion = request.motion_plan_request
            motion.group_name = "arm"
            motion.pipeline_id = "ompl"
            motion.allowed_planning_time = 5.0
            motion.num_planning_attempts = 3
            motion.max_velocity_scaling_factor = 0.2
            motion.max_acceleration_scaling_factor = 0.2
            motion.start_state.is_diff = True
            motion.goal_constraints = [Constraints(joint_constraints=[
                JointConstraint(joint_name=name, position=value,
                                tolerance_above=0.001, tolerance_below=0.001, weight=1.0)
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
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if all(abs(latest[name] - value) < 0.01 for name, value in positions.items()):
                    break
            assert all(abs(latest[name] - value) < 0.01 for name, value in positions.items()), latest
            print(f"PASS: planned and executed {target}; joint feedback matches", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
