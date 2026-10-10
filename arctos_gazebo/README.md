# Arctos Gazebo

Target: ROS 2 Jazzy and Gazebo Harmonic via `gz_ros2_control`.

Gate A provides dependency declarations and packaging tests. Worlds, robot
spawning, the simulator plugin wrapper and launch files follow in Gate D.
There is no runnable Arctos Gazebo launch yet.

Gate B adds SDF conversion and MoveIt/FCL collision tests for the canonical bare
arm, including support clearance and deliberately colliding negative controls.

See `docs/JAZZY_SETUP.md` at repository root for the isolated build and test path.
