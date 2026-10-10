# Manufacturer motion references and RViz gripper

Reviewed 2026-10-10. These findings supersede the inherited travel bounds and
fixed-gripper preview recorded at Gate B.

## Sources and their limits

1. [Official specifications, work envelope and DH diagrams](https://arctosrobotics.com/docs/)
   describe a six-axis arm with 600 mm maximum reach, axis names and gear ratios.
   The specifications section does not give a numeric joint-stop table. The
   [work-envelope drawing](https://arctosrobotics.com/wp-content/uploads/2024/05/Screenshot_2.jpg)
   gives dimensions/radii, not calibrated angular limits.
2. [Official Arctos Studio Web](https://arctosrobotics.com/studio/) defines both
   degree-valued UI limits and radian-valued IK limits in its publicly served
   [application bundle](https://arctosrobotics.com/studio/assets/index-Dzvz2M_O.js).
   These are the best explicit manufacturer **software defaults** found, not a
   certified mechanical-stop specification for every hardware revision.
   Retrieved bundle SHA-256:
   `40b52a906205e3cbc8cb671a80b78286876b013bc9c2d747f2eebd2657f3c694`.
   HTTP Last-Modified: 2026-09-26. The degree defaults are transcribed below;
   the IK values agree to rounding.
3. [Manufacturer ROS model](https://github.com/Arctos-Robotics/ROS/blob/405dc5faf9c529b229f21b30e05e8197ef38096e/arctos_urdf_description/urdf/arctos_urdf.xacro),
   pinned at `405dc5faf9c529b229f21b30e05e8197ef38096e`, defines all six arm joints
   as continuous, so it is **not** a source of realistic bounded arm travel.
   It does define opposing prismatic jaws with 0–0.015 m travel. All ten meshes
   in that repository are byte-identical to ours (Git blob hashes checked).
4. The official docs' **Gripper Setup** section identifies a DS3225 servo and
   gives commands: closed-loop `0763` open / `0700` close; open-loop
   `M97 B120 T0.2` open / `M97 B40 T0.2` close. These servo command values are
   not jaw distances or a force specification.

The current homepage advertises a 2 kg payload while the docs say 1 kg. This
illustrates why revision-specific mechanical ratings should not be inferred
from the marketing overview. Neither payload value is used to tune this mock.

## Arm travel and sign conventions

| Axis | Previously in our model (degrees) | Studio defaults (degrees) | Now in our model (degrees) |
| --- | --- | --- | --- |
| X / base | −180 … 180 | −200 … 200 | −200 … 200 |
| Y / shoulder | −55.00 … 124.91 | −70 … 120 | −70 … 120 |
| Z / elbow | −90 … 70 | −40 … 110 | **−110 … 40** |
| A / wrist roll | −180 … 180 | −180 … 180 | −180 … 180 |
| B / wrist bend | −88.81 … 88.81 | −100 … 100 | −100 … 100 |
| C / tool roll | −180 … 180 | −180 … 180 | −180 … 180 |

Studio and the manufacturer ROS model rotate Z about **−X**. Our full-arm
convention rotates it about **+X**. Thus `q_local_Z = -q_manufacturer_Z`, and
`[-40, 110]` becomes `[-110, 40]`. Copying the manufacturer's numeric elbow limits
without this conversion would allow the wrong side of travel. The other five
axis directions agree. The zero poses have the same general CAD orientation;
the reference uses slightly different frame/pivot positions, already discussed
in [MODEL_REVIEW.md](MODEL_REVIEW.md). This is not a physical calibration.

The canonical bounds are in
`arctos_description/config/physical_joint_limits.yaml`. Both full and reduced
models, mock position command limits, and the legacy adapter consume them.
MoveIt inherits velocity and position bounds from the URDF. Existing effort and
velocity estimates are retained; the public travel defaults do not establish
motor torque, loaded speed or acceleration ratings. RViz defaults to 10% speed
and acceleration scaling, which also makes the demo deliberately slow.

Joint limits and collision feasibility are different: an angle can be within
its allowed range yet collide with another link at a particular arm pose.
The two wrist motor commands are also coupled in the manufacturer's CAN code
(`C-B`, `C+B`); our joints describe output-link motion, not raw motor angles.
Physical motor calibration, cable constraints and endstop offsets remain
hardware-revision-specific.

## Implemented gripper convention

- The description keeps `include_gripper:=false` by default for bare-arm users.
  The **mock RViz launch enables it by default**.
- `Left_jaw_joint` is the single commanded prismatic joint. `Right_jaw_joint`
  mimics it 1:1 along the opposing axis; it has state interfaces only.
- `q=0` is **open**, preserving the mesh CAD pose. `q=0.015 m` is **closed**.
  The distal pad separation is approximately `0.030 - 2*q` metres. A mesh-based
  test verifies the 30 mm opening and the 15 mm per-jaw closing stroke.
- Gripper velocity `0.02 m/s`, acceleration `0.05 m/s²` and effort `20 N` are
  mock/planning estimates, not manufacturer force or speed ratings.
- The `gripper` MoveIt group has named states `open` and `closed`, and is an end
  effector attached to `tool0`. `gripper_controller/follow_joint_trajectory`
  controls the independent jaw. `tool0` remains the wrist attachment frame;
  it is not a newly calibrated grasp TCP.
- The wrist/body mount, jaw/body sliding contacts and jaw-to-jaw closure contact
  are allowed collision pairs. Other gripper-to-arm and object collisions remain
  checked. There are no broad new `Never` exclusions.

This implements planning, display and mock execution. Object attachment,
force-controlled grasping, Gazebo contacts and physical servo control are later
work; closing on an object is not yet a simulated grasp.

## Validation

The tests cover one gripper actuator, mimic direction, manufacturer-limit sign
conversion, pad geometry and 16 opening samples at each of `home` and `demo`.
The latter use MoveIt/FCL with the actual mock SRDF.

The headless runtime smoke test plans and executes `arm:demo`, `gripper:closed`,
`gripper:open`, `arm:home`, checking successful actions and both jaw feedback
positions (within 0.1 mm). Bare-arm launch/execution is also checked separately.
Run `bash scripts/validate_jazzy.sh` for the build/model checks and follow
[the MoveIt README](../arctos_moveit_config/README.md) for the runtime test.

Final evidence: `/tmp/opencode/arctos-gripper-final`, with 32 pytest cases
passing (colcon reports 36 including four CTest wrappers), zero errors/failures/
skips, and successful headless runtime checks for both variants. Runtime logs
are `gripper-runtime.log` and `bare-runtime.log` in that directory. RViz was not
visually inspected in this display-less session.
