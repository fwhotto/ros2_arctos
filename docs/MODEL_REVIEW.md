# Gate B: canonical Arctos model review

Follow-up: [manufacturer motion specifications](MOTION_SPECIFICATIONS.md) now
provide the arm travel defaults and actuated gripper convention. The Gate B
validation report remains a historical record of the bare-arm milestone.

## Decisions and supported model

The canonical description is the **bare six-axis arm**, without a control
backend. `arctos_description/urdf/arctos_arm.xacro` exports the reusable
`arctos_arm` macro. The `arctos.xacro` entry point adds a fixed world mount.
The three-joint entry point instantiates the same macro with the wrist omitted.

Parameters:

- `prefix`: applied to all robot link/joint names, excluding the shared `world`.
- `include_gripper`: false by default; true adds the gripper with opposing
  prismatic jaws and a mimic follower. The mock RViz launch enables it by default.
- `use_hardware`: false by default; explicit opt-in to the legacy hardware
  adapter. Jazzy hardware execution remains unvalidated.

The geometry macro also accepts `include_wrist`; its default is true. It does
not create `world` or its own anchor, so a caller can mount it in a larger cell.
Prefixing is model-level preparation, not a claim of multi-arm controller or
MoveIt support.

`tool0` is fixed at the origin of `Link_6_1`, with identical axes. At zero joint
angles its world position is `(0, -0.323031, 0.5685601)` metres and its orientation
is identity. A future gripper/tool defines its own TCP relative to this frame.

## Comparison with coen132/Arctos

Reference commit: `603bdcb079340604b5da2b364c8c03169d0ea96c`.
Reference file: `arctos_description/urdf/arctos_urdf.xacro`.
Local baseline before this task: `158bd8c`.

All ten STL meshes are byte-identical between the repositories, verified by
Git blob hashes. Representative hashes:

| Mesh | Git blob SHA-1 |
| --- | --- |
| `base_link.stl` | `5511707872fec2d3482132214f170615a578a73e` |
| `Link_1_1.stl` | `6805f1faf55bb4c9119b5755ee18b09671200645` |
| `Link_6_1.stl` | `af360e27c27004ab13dbd15623ad64ddc850afa8` |

The meshes use millimetres and assembly coordinates; URDF scales them by 0.001.
The local joint/visual origins assemble the bare meshes at zero angles within
0.1 mm of that original CAD assembly. A regression test checks this explicitly.

### Joint-frame comparison

The table gives local joint origins in metres. All origin rotations are zero.
Names map X/Y/Z/A/B/C to reference joint_1/2/3/4/5/6 respectively.

| Joint | Retained local origin | Reference origin | Retained local axis | Reference axis |
| --- | --- | --- | --- | --- |
| X | `0 0 0.1126` | `0 0.000529 0.111` | `0 0 1` | `0 0 1` |
| Y | `0 -0.020356 0.175` | `-0.05713 -0.020885 0.178378` | `1 0 0` | `1 0 0` |
| Z | `0 0.001365 0.261035` | `0.10013 0.001311 0.261007` | `1 0 0` | `-1 0 0` |
| A | `0 -0.13881 0.0217651` | `-0.042999 -0.139695 0.021705` | `0 1 0` | `0 1 0` |
| B | `0 -0.12543 -0.00184` | `-0.039012 -0.121187 -0.001774` | `-1 0 0` | `-1 0 0` |
| C | `0 -0.0398 0` | `0.039204 -0.033835 0` | `0 1 0` | `0 1 0` |

Retain the local full-arm kinematic convention and bounded revolute joints.
The reference declares all six joints continuous, so it is not a source of
mechanical travel limits. Some reference frame shifts lie along rotation axes;
others change joint locations. Replacing origins independently of visual/COM
origins would be incorrect. Physical calibration is still required before
transferring simulation results to motors.

The reduced local model previously reversed Y and used different Z limits.
It now shares exactly the first three links/joints with the full model. This
changes the reduced model's Y convention; physical calibration files must not
be assumed compatible.

### Canonical limits

`arctos_description/config/physical_joint_limits.yaml` is the sole source for
URDF bounds and the legacy control adapter. Future mock/Gazebo adapters should
consume it rather than duplicate numbers.

| Joint | Lower / upper (rad) | Velocity (rad/s) | Effort (N m) |
| --- | --- | --- | --- |
| X | `-200*pi/180 / 200*pi/180` | 1.4 | 24.3 |
| Y | `-70*pi/180 / 120*pi/180` | 1.4 | 270 |
| Z | `-110*pi/180 / 40*pi/180` | 1.4 | 97 |
| A | `-pi / pi` | 4.4 | 22 |
| B | `-100*pi/180 / 100*pi/180` | 3.0 | 16 |
| C | `-pi / pi` | 3.0 | 16 |

Travel now follows manufacturer Studio software defaults with the local elbow
sign conversion; see the follow-up review. Effort values remain estimates from
the existing full-arm description. X velocity uses
the conservative existing reduced-model value of 1.4 rather than 15 rad/s.
These are project estimates, not newly measured actuator specifications.
Planner acceleration limits and controller settings are later-gate work.

Malformed literal expressions and the reversed Y control bounds have been
removed. The old MoveIt control include now delegates to the same adapter.

## Mass and inertia decisions

Keep the existing local masses/COM/tensors for the base and links 1–5. They
pass positive-definiteness, rigid-body principal-moment triangle inequalities,
COM-in-mesh-bounds and radius-of-gyration plausibility checks. These checks
establish numerical/geometric plausibility, not measurement accuracy.

| Link | Retained mass (kg) |
| --- | --- |
| Base | 2.3 |
| 1 | 3.0396 |
| 2 | 2.527 |
| 3 | 0.87 |
| 4 | 1.006 |
| 5 | 0.3344 |

Do not adopt the reference's much larger arm masses wholesale (for example,
11.558 kg for link 2). The source does not establish material densities or
whether motor/fastener masses are included.

### Wrist correction

The old wrist had mass 0.001 kg and principal inertias 0.001 kg m², implying a
one-metre radius of gyration for a mesh only about 73 mm across. Replace it with
the reference's geometrically plausible estimate:

- Mass: `0.103434780908908 kg`.
- COM in local link frame: `(0.000193065679464, 0.006195601622124,
  0.001815696749104) m`.
- Inertia about that COM: diagonal `(0.000052, 0.000103, 0.000052) kg m²`.

Because the meshes are identical and both inertial frames have identity
rotation, transform the reference COM as:

```text
COM_local = COM_reference + visual_origin_local - visual_origin_reference
```

The tensor is already about the COM with the same axis orientation, so it does
not require a parallel-axis shift. This reference-derived estimate should be
replaced if measured/CAD-material properties become available.

### Optional gripper

The gripper body retains its local estimate. Both jaws now have nonzero
reference-derived mass/tensors and translated COMs, using the same formula.
The follow-up now adds opposing jaw axes, a mimic relation and mock actuation,
with tests for both open/closed poses and intermediate openings. This adds one
independent DOF to the six-axis arm. Gazebo contact and physical servo behavior
remain unvalidated; see the follow-up review for limits and source provenance.

## Collision, support and dynamics

Keep the original triangle meshes for collision geometry at this gate. They
preserve the CAD shape, and the measured headless model/collision tests complete
in seconds. There is no demonstrated need for simplification yet. Gazebo contact
performance and any engine-specific convex approximation must be reviewed in
Gate D; visual fidelity alone does not prove contact fidelity.

The shared initial pose in `arctos_control/config/initial_positions.yaml` sets
all six joints to zero. MoveIt/FCL checks it using only adjacent-link collision
exclusions, **not** the old SRDF's broad `Never` exclusions. It is free of
non-adjacent self-collision.

The support test uses a 2 m × 2 m × 0.1 m box with its top at world z=0. Contact
between this support and the bolted base is permitted; every moving link is
checked. The CAD base bottom is z≈0 (floating-point mesh roundoff of ~1e-9 m);
all moving links have more than 0.1 m ground clearance in this pose.

Negative controls verify that FCL detects both a deliberately obstructing box
and a known self-colliding folded pose. The later world and planning scene must
use consistent support geometry; changing the mount or initial angles requires
revalidating clearance.

Each revolute joint specifies damping `0.1 N m s/rad` and friction `0`. Damping
provides modest numerical regularization; friction is zero because no measured
value is available. These are initial simulation parameters, not a gear/stepper
model. Tune them only with the later controller/physics execution tests.

## SDF and anchoring

`gz sdf -p` converts the bare URDF without diagnostics and preserves six
revolute joints with inertial child links, limits and damping. It retains the
fixed `world_joint` from world to base. The model is not made static. The
massless fixed tool link is represented as a named SDF frame after fixed-joint
lumping. No ROS 1 transmissions or hardware/Gazebo plugins appear in the
default model.

Gate B validates conversion and collision geometry, not gravity-loaded pose
holding or controller execution. Those require Gates C/D.
