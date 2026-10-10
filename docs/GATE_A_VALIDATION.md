# Gate A validation report

Date: 2026-10-10.

## Environment

- Host: Ubuntu 24.04.5, aarch64 Linux (architecture corrected during Gate B review).
- ROS underlay: `/opt/ros/jazzy` only; validator clears inherited overlays.
- Gazebo Sim: 8.15.0 (Harmonic).
- Installed MoveIt: 2.12.4; `gz_ros2_control`: 1.2.20.
- No package installation or system configuration changes were required.

## Executed validation

Command: `bash scripts/validate_jazzy.sh`.

Result directory: `/tmp/opencode/arctos-jazzy.5W61mi`.
This path is local validation evidence, not a required repository resource.

1. `rosdep check` for the four simulation packages: all dependencies satisfied.
2. Gazebo version assertion: major version 8 passed.
3. Fresh, non-symlink colcon build: all four packages succeeded:
   `arctos_description`, `arctos_control`, `arctos_moveit_config`, `arctos_gazebo`.
4. Colcon test: 11 pytest cases passed. The aggregate colcon result counts the
   CTest wrapper as well and reports 12 tests, zero errors/failures/skips.
5. Tests verified local dependency closure, exclusion of hardware/CAN/Classic,
   simulator-independent core packages, installed ROS dependency availability,
   and matching installed model/mesh/launch/configuration assets.
6. `git diff --check`: passed.
7. `bash -n scripts/validate_jazzy.sh`: passed. Reusing the populated result
   directory was correctly rejected with exit status 1, validating the
   fresh-directory guard without overwriting prior results.

## Evidence boundaries

This is a clean build directory and cleared process environment on an existing
Jazzy host, not a fresh operating-system/container installation. The manifest
dependency closure was resolved with rosdep; tests additionally check that
declared ROS dependencies are discoverable.

Gate A does not validate Xacro model correctness, collision freedom, simulated
controller activation, MoveIt execution, or physical hardware compatibility.
The initial all-zero pose is provisional. These checks belong to subsequent
gates and are not implied by successful packaging tests.
