# Jazzy simulation development: Gates A and B

## Supported environment

- Ubuntu 24.04 (Noble), ROS 2 Jazzy, Gazebo Harmonic (Gazebo Sim 8).
- MoveIt 2 and `gz_ros2_control` from the Jazzy binary packages.
- Source only `/opt/ros/jazzy/setup.bash` before building this package set.

Gate A checks dependencies, packaging and installation. Gate B adds a canonical
bare-arm model with physical-property, SDF conversion and collision tests. There
is not yet a runnable Gazebo arm. Existing demo/controller/MoveIt configuration
will be migrated in later gates.

## Dependency installation

Install ROS 2 Jazzy using the official Ubuntu deb instructions first:
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Then, from the repository root:

```bash
sudo apt install ros-dev-tools python3-pytest
# Initialize rosdep once per machine if it has not already been initialized:
sudo rosdep init
rosdep update --rosdistro jazzy
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths arctos_description arctos_control \
  arctos_moveit_config arctos_gazebo --ignore-src --rosdistro jazzy -y
```

The explicit package paths exclude `arctos_bringup`, CAN drivers and the legacy
three-joint configuration. Do not use the repository root as `--from-paths` for
this simulation-only dependency installation.

Warehouse functionality is optional. To use the legacy warehouse launch,
install the Jazzy `moveit_ros_warehouse` and `warehouse_ros_mongo` packages
separately where available. The Gate A package closure does not require MongoDB.

## Isolated build and tests

```bash
bash scripts/validate_jazzy.sh
# Or select a fresh output directory:
bash scripts/validate_jazzy.sh /tmp/opencode/arctos-gate-b
```

The script clears inherited overlays, sources only Jazzy, checks rosdep and
Gazebo's major version, and builds the four selected packages into a fresh
directory. It uses a normal install (not symlink-install) so missing install
rules cannot be hidden by source-tree symlinks. It runs colcon tests and reports
their results. Existing output directories containing build/install/log are
rejected to avoid stale-build success.

Tests cover:

- The local simulation dependency closure excludes CAN/hardware and Classic.
- Description/control dependencies do not require a simulator.
- Declared ROS dependencies are discoverable in the sourced install.
- All current model, mesh, launch and configuration resources are installed
  with matching contents; the same check covers future world files.
- URDF parsing, shared reduced/full geometry, prefixes, backend-independent
  model expansion and optional mimic-jaw gripper attachment.
- Physical bounds, inertial plausibility, CAD assembly alignment and clearance.
- SDF articulation/anchoring and tool frame preservation.
- Actual MoveIt/FCL self-collision/support tests, with obstructed and folded
  poses as negative controls. The legacy MoveIt SRDF is not used for these tests.

The output directory contains colcon logs/results, the selected package list,
installed Jazzy package versions and Gazebo version. This supports repeating the
validation on a clean Noble/Jazzy machine. Binary repository updates may change
versions; retain the version inventory alongside validation results.

To source the validated packages, use the reported directory:

```bash
source /tmp/opencode/arctos-gate-b/install/setup.bash
```

## Scope of evidence

An isolated underlay-only build on an existing Noble host verifies that no
workspace overlay supplies missing Arctos packages. It is not a fresh-container
installation test. Dependency resolution is checked with rosdep. Model and
collision tests run headlessly using the installed description. Controller
activation, gravity-loaded pose holding and motion execution are later gates.
