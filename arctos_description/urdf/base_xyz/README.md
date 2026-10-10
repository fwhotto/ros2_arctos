# arctos_description/urdf/base_xyz

This folder contains the URDF files for the base of the robot. The base is composed of the following parts:

This entry point now instantiates the canonical `arctos_arm` macro without its
wrist. Geometry, axis signs and limits are shared with the full arm. In
particular, Y now uses the full-arm +X axis and Z uses the full-arm travel bounds.
The default model contains no control backend; `use_hardware:=true` explicitly
adds the legacy adapter. Existing physical calibration is not validated against
this convention.

**Joints**:
- `world_joint`: The joint that connects the world frame to the base link. (`world` to `base_link`) (fixed joint)
- `X_joint`: The joint that connects the base link to the first link of the robot. (`base_link` to `Link_1_1`)
- `Y_joint`: The joint that connects the first link of the robot to the second link. (`Link_1_1` to `Link_2_1`)
- `Z_joint`: The joint that connects the second link of the robot to the third link. (`Link_2_1` to `Link_3_1`)
