# arctos_moveit_config

## Description

This package provides the MoveIt configuration for the Arctos robot.

## RViz planning demo (Jazzy)

From the repository root, in a terminal sourced only with Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to arctos_moveit_config
source install/setup.bash
ros2 launch arctos_moveit_config mock_demo.launch.py
```

This starts RViz, MoveIt with OMPL/KDL, robot_state_publisher, and ros2_control
with `mock_components/GenericSystem`. The six-axis bare arm starts at the shared
initial pose. `arm_controller` executes position trajectories and the joint
state broadcaster updates the displayed robot. No Gazebo or physical hardware
driver is started. All nodes use wall time.

### Try a motion

1. Wait for **You can start planning now!** in the terminal.
2. In RViz's **MotionPlanning** panel, select planning group **arm**.
3. In **Planning**, set Start State to **<current>** and Goal State to **demo**
   (click **Update** if shown next to the state selector).
4. Click **Plan** to see the animated trajectory preview.
5. Click **Execute** to move the mock robot, or use **Plan & Execute**.
6. Select **home** as the goal and repeat to return.

You can also enable **Query Goal State** under the MotionPlanning display's
**Planning Request** properties and drag the tool's interactive marker. Use the
**Interact** toolbar tool for dragging. Named poses are a convenient first test;
arbitrary dragged targets may be unreachable or in collision.

The RViz grid is visual only; this demo starts with an empty collision world.
Self-collision checking is enabled. Planning acceleration limits are conservative
estimates, and mock execution does not model gravity or contact physics.

### Headless regression check

In one sourced terminal:

```bash
export ROS_DOMAIN_ID=81
ros2 launch arctos_moveit_config mock_demo.launch.py rviz:=false
```

In another terminal with the same workspace sourced, from the repository root:

```bash
export ROS_DOMAIN_ID=81
python3 scripts/test_mock_moveit.py
```

This sends real MoveIt planning requests and executes `demo` then `home`, checking
action success and final `/joint_states` feedback. Use this domain only for the
mock stack. Stop the launch with Ctrl+C when finished.

Validated on 2026-10-10: fresh Jazzy build, all 27 existing pytest cases passing,
and both end-to-end plan/execute requests successful. Evidence is in
`/tmp/opencode/arctos-moveit-mock/runtime.log`. The RViz window was not visually
verified because the validation session had no graphical display.

Use `mock_demo.launch.py` for this workflow; older generated launch files are
legacy entry points pending migration.
