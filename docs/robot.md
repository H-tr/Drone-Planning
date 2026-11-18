# Robot Models

The package has two robots. Both have 6 DOF in the same order: the body
`(x, y, z, yaw)`, then the camera pan and tilt.

| Robot name | Model | Collision model | Limits |
|---|---|---|---|
| `x500` (default) | Holybro X500 V2 with PX4's gimbal | 126 spheres | PX4 v1.17 defaults |
| `drone` | [PHANTOM's drone](#phantoms-drone) | 1 sphere, radius 0.30 m | PHANTOM's |

Pick a robot by its name: `create_planner("drone")`,
`create_planner("drone_base")` or `OmplVampPlanner("drone")`. Without a
name, `OmplVampPlanner()` is the X500, as before.

## Holybro X500 V2

The X500 is the **Holybro X500 V2** quadrotor carrying a pan-tilt gimbal
camera, as modelled by PX4 for Gazebo:
[`PX4-gazebo-models`](https://github.com/PX4/PX4-gazebo-models)
`x500_gimbal` = `x500` (which merges `x500_base`) + `gimbal`, pinned to
commit `e5997f4` (BSD-3-Clause; the license ships next to the URDF).

The flight controller holds roll and pitch, so the body is a
velocity-commanded point with a heading, and the configuration is 6-DOF:

| Index | Joint | Group | Range | Notes |
|------:|-------|-------|-------|-------|
| 0 | `base_x_joint` | base | −10 … 10 m | virtual, world frame |
| 1 | `base_y_joint` | base | −10 … 10 m | virtual, world frame |
| 2 | `base_z_joint` | base | 0 … 10 m | virtual, z up |
| 3 | `base_yaw_joint` | base | −π … π | virtual, counter-clockwise from +x |
| 4 | `cgo3_vertical_arm_joint` | gimbal | −π … π | pan: PX4 `MNT_RANGE_YAW` = 360° |
| 5 | `cgo3_camera_joint` | gimbal | −135° … 45° | tilt: SDF joint limits |

The gimbal keeps the PX4 model's joint names and axes, so its joint
values are PX4's gimbal angles in the body frame: **positive pan turns
the camera clockwise seen from above (to the right), positive tilt raises
it**, zero looks straight ahead. Gimbal roll (`cgo3_horizontal_arm_joint`)
is held level. The base ranges are the default planning workspace; set
your own with `PlannerConfig(joint_limits=...)` or
`MotionPlanner.set_joint_limits`.

### SDF → URDF conversion

`pixi run build-robot` (`scripts/build_robot.sh`) fetches the pinned PX4
models into `build/` and runs `scripts/px4_sdf_to_urdf.py`, which writes
`drone_planning/resources/robot/x500/x500.urdf` and `meshes/`. The
conversion keeps every link, joint, pose, mesh and inertial of the SDFs
and changes only what a planning model needs:

* **Model composition.** The `<include merge="true">` chain is resolved
  with SDF's rules: an include's `<pose>` replaces the included model's
  own pose (`x500_base` sits 0.24 m up in its model; the gimbal is
  included 0.26 m up and yawed 3.14 rad), link poses are in the model
  frame, joint poses in the child link frame and joint axes in the joint
  frame. URDF puts each child link's frame at its parent joint, so link
  contents are re-expressed in that frame: the gimbal mount sits 2 cm
  above `base_link`, the camera 0.142 m below it and 4.1 cm ahead.
* **Virtual base joints.** `world → base_x_joint → base_y_joint →
  base_z_joint → base_yaw_joint → base_link`, the same convention as
  Fetch's virtual base.
* **Fixed joints.** The four rotor joints (spinning propellers) and the
  gimbal roll joint become fixed. The gimbal pan joint is unlimited in
  the SDF; it gets PX4's `MNT_RANGE_YAW` (360°).
* **Meshes.** Written as binary STL in their own frame, in metres: the
  SDF `<scale>` (0.001 for the gimbal's millimetre meshes, 0.846 for the
  propellers) is applied to the vertices, because foam applies a
  collision origin *before* a mesh scale and would misplace the spheres.
  COLLADA meshes are flattened with their node transforms. The frame
  mesh (`NXP-HGD-CF.dae`, 179k faces, 22 MB) is decimated by quadric
  edge collapse to 21k faces to fit the repository's 1 MiB file limit —
  largest deviation 4.0 mm, mean 0.25 mm (two-sided Hausdorff on 200k
  samples). Three gimbal meshes and the propellers are wound inwards
  (negative enclosed volume); their faces are flipped. Textures and the
  four flat decal planes on the flight controller are dropped.
* **Collision geometry** is the visual meshes, which foam spherizes —
  except on the rotors: a spinning rotor sweeps a solid of revolution, so
  its collision is the cylinder that bounds its propeller and motor-bell
  meshes about the rotor axis (radius 0.147 m, 4.1 cm tall). The SDF's
  own collision primitives (boxes for frame and skids, a blade-shaped box
  per rotor, a sphere for the camera) are coarser stand-ins and are not
  used.
* **Camera frame.** A fixed `camera_sensor_link` marks the SDF camera
  sensor pose (x along the optical axis, Gazebo's convention; 1280×720,
  horizontal field of view 2.0 rad).
* **Limits.** Velocity / acceleration limits are not in the URDF (its
  `<limit velocity>` is 0); they live in `drone_planning/x500.py`.

Cross-check with Holybro's X500 V2 specification: the rotors sit on a
0.348 m square (0.49 m diagonal; Holybro: 500 mm wheelbase) and the skids
hang 0.23 m below the frame (Holybro: 215 mm landing gear). The PX4 model
carries 11.5-inch propeller meshes; Holybro ships 10-inch (1045)
propellers, so the model's rotor discs are 2 cm larger in radius.

### Spherization and the VAMP model

The pipeline is the template's, unchanged:

1. **foam** (`pixi run spherize-robot`, `scripts/spherize_robot.sh`):
   `third_party/foam/scripts/generate_sphere_urdf.py` with its sphere
   database turns every collision mesh into a medial-axis sphere tree →
   `x500_spherized.urdf`. foam gives each geometry
   `min(0.7 × bounding-sphere volume / volume, --branch)` spheres:
   `--branch 16` (default 8) and a ×4 factor on the four rotor cylinders,
   which the volume heuristic gives only 3 spheres each.
2. **SRDF** (`x500.srdf`, self-collision exclusions): only the gimbal
   moves relative to the body. Checking the mesh model on a 73 × 61 grid
   over the pan and tilt ranges, the only contacts are between links
   joined by a joint (the pan yoke in its mount, the camera against its
   tilt yoke at the ends of its range); every other pair never touches.
   The spheres overreach the meshes and overlap in most gimbal poses, so
   every pair is excluded: the X500 cannot collide with itself inside its
   joint limits.
3. **cricket** (`pixi run generate-fk`, `scripts/generate_fk.sh`):
   `fkcc_gen` with `scripts/cricket_configs/x500.json` traces the
   forward kinematics and collision checks into
   `ext/ompl_vamp/include/vamp/robots/x500.hh` (`vamp::robots::X500`).
   Its motion-check `resolution` is 64: one check every 1.6 cm of
   translation, below the 1.9 cm diameter of the smallest sphere.

The model has **126 spheres** (radius 9.5–79.5 mm), covering the original
PX4 meshes (the propeller blades at every spin angle) except for thin
slivers between sphere-tree sample points:

| Link | Spheres |
|------|--------:|
| `base_link` (frame, skids, motor mounts) | 24 |
| `rotor_0` … `rotor_3` (propeller disc + bell) | 4 × 12 |
| `cgo3_mount_link` | 14 |
| `cgo3_vertical_arm_link` | 16 |
| `cgo3_horizontal_arm_link` | 16 |
| `camera_link` | 8 |

`tests/test_collision_model.py` checks the coverage through the compiled
VAMP model at four configurations. Point obstacles are checked as points
(`point_radius` 0): the spheres are the margin.

!!! note "Point clouds and link bounding spheres"

    The generated check tests each link's bounding sphere (the smallest
    sphere around its spheres) against the environment before the link's
    spheres, and `add_pointcloud` indexes a point cloud in VAMP's CAPT for
    queries up to `Robot::max_radius`. Known upstream limitation:
    `max_radius` is the largest single sphere (79.5 mm), not the largest
    bounding sphere (base_link's, 286 mm), so a bounding-sphere query can
    miss a point the link's spheres touch and a colliding configuration
    passes. `tests/test_collision_model.py` checks VAMP's verdict against
    a brute-force sphere–point check and is an expected failure until
    upstream fixes this.

### Limits

`drone_planning/x500.py` takes every limit from PX4 v1.17.0 default
parameters, which none of the X500 airframe files override:

| Quantity | Bound | PX4 parameter |
|----------|-------|---------------|
| horizontal speed, norm of `(ẋ, ẏ)` | 12 m/s | `MPC_XY_VEL_MAX` |
| horizontal acceleration, norm of `(ẍ, ÿ)` | 5 m/s² | `MPC_ACC_HOR_MAX` |
| climb / descent rate | 3 / 1.5 m/s | `MPC_Z_VEL_MAX_UP` / `MPC_Z_VEL_MAX_DN` |
| upward / downward acceleration | 4 / 3 m/s² | `MPC_ACC_UP_MAX` / `MPC_ACC_DOWN_MAX` |
| yaw rate / acceleration | 60 °/s / 20 °/s² | `MPC_YAWRAUTO_MAX` / `MPC_YAWRAUTO_ACC` |
| gimbal pan / tilt rate | 30 °/s | `MNT_RATE_YAW` / `MNT_RATE_PITCH` |
| gimbal pan / tilt acceleration | 97 / 300 rad/s² | — |

PX4 has no gimbal acceleration parameter: the bound is the torque cap of
the gimbal model's joint controllers (`cmd_max` = 0.3 N m) over the SDF
inertia about each joint axis — effectively the gimbal is rate-limited.
TOPP-RA takes symmetric per-joint bounds: pass the tighter side of each
range (see [Time parameterization](planning/trajectory.md)).

## PHANTOM's drone

The drone of the PHANTOM planner
([H-tr/PHANTOM-Planner](https://github.com/H-tr/PHANTOM-Planner),
`phantom/robot/drone.urdf`). It is a simple model of a stabilized
multirotor: the flight controller holds roll and pitch, so the body is
position and yaw. A camera hangs 6 cm below the body centre on a pan-tilt
gimbal. The configuration is 6-DOF, in PHANTOM's order:

| Index | Joint | Group | Range | Notes |
|------:|-------|-------|-------|-------|
| 0 | `base_x_joint` | base | −10 … 10 m | virtual, world frame |
| 1 | `base_y_joint` | base | −10 … 10 m | virtual, world frame |
| 2 | `base_z_joint` | base | 0 … 10 m | virtual, z up |
| 3 | `base_yaw_joint` | base | −π … π | virtual, counter-clockwise from +x |
| 4 | `camera_pan` | gimbal | −90° … 90° | PHANTOM's limits |
| 5 | `camera_tilt` | gimbal | −90° … 30° | PHANTOM's limits; −90° looks straight down |

**Positive pan turns the camera left** (counter-clockwise seen from
above; the X500's pan turns the other way), positive tilt raises it, and
zero looks straight ahead. So the camera looks along azimuth
`yaw + pan` at elevation `tilt`. The base ranges are the same as the
X500's; set your own with `PlannerConfig(joint_limits=...)` or
`set_joint_limits`.

The files are in `drone_planning/resources/robot/drone/`:

* `drone.urdf` — PHANTOM's URDF. PHANTOM's floating base joint becomes the
  package's virtual base joints (`world → base_x_joint → base_y_joint →
  base_z_joint → base_yaw_joint → base_link`), as for the X500. The body
  link is `base_link`; the gimbal joints and links keep PHANTOM's names
  (`camera_pan`, `gimbal`, `camera_tilt`, `camera`). The visual boxes are
  kept. The camera's optical axis is the x axis of `camera`.
* `drone_spherized.urdf` — the source of the VAMP model. **It is written
  by hand, without foam:** the collision geometry of `drone.urdf` already
  is one sphere (radius 0.30 m at the origin of `base_link`), so this file
  keeps the joints and that sphere and drops the visual boxes.
* `drone.srdf` — the planning groups. Only `base_link` has collision
  geometry, so there is no pair of links to check against each other:
  the drone cannot collide with itself.

cricket (`pixi run generate-fk-drone`, which runs
`scripts/generate_fk.sh drone`) turns `drone_spherized.urdf` and
`drone.srdf` with `scripts/cricket_configs/drone.json` into
`ext/ompl_vamp/include/vamp/robots/drone.hh` (`vamp::robots::Drone`). Its
motion-check `resolution` is 64 (one check every 1.6 cm of translation),
the value of Autolife-Planning's model. The gimbal angles do not enter
the collision check: only the body position does.

### Limits

`drone_planning/drone.py` takes the limits from PHANTOM's URDF. They are
simulated capabilities, not hardware ratings:

| Quantity | Bound |
|----------|-------|
| speed along x, y and z, each | 1.5 m/s |
| acceleration along x, y and z, each | 1.0 m/s² |
| horizontal speed, norm of `(ẋ, ẏ)` | 1.5 m/s |
| horizontal acceleration, norm of `(ẍ, ÿ)` | 1.0 m/s² |
| yaw rate / acceleration | 1.5 rad/s / 3.0 rad/s² |
| camera pan and tilt rate / acceleration | 1.0 rad/s / 2.0 rad/s² |

PHANTOM gives one limit per axis. FLASK also limits the length (norm) of
the horizontal velocity and acceleration, so those limits take the
per-axis values. This is a little stricter than PHANTOM for diagonal
flight: the horizontal speed is then at most 1.5 m/s in all.

### Camera

`drone_robot_config.camera` follows PHANTOM's sensor: a view cone of
1.0821 rad (62°) that sees up to 5 m. PHANTOM gives no image size; the
square 640 × 640 image is this package's choice.
