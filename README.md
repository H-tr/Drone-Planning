# Drone-Planning

A motion planning library for drones with a pan-tilt gimbal camera: the
**Holybro X500 V2 quadrotor** and **PHANTOM's drone** (a simple model: a
box body and a gimbal camera, 10 collision spheres). Same public API and module layout as its sister
packages [Fetch-Planning](https://github.com/H-tr/Fetch-Planning) and
Autolife-Planning — OMPL planners over a VAMP SIMD collision backend,
FLASK kinodynamic planning and TOPP-RA time parameterization — so code
can switch robots by changing a single import.

```bash
pip install drone-planning
```

Documentation: <https://h-tr.github.io/Drone-Planning/>

---

## The robots

| Robot name | Model | Collision model | Limits |
|---|---|---|---|
| `x500` (default) | Holybro X500 V2 with PX4's gimbal | 126 spheres | PX4 v1.17 defaults (`drone_planning/x500.py`) |
| `drone` | PHANTOM's drone | 10 spheres (foam) | PHANTOM's (`drone_planning/drone.py`) |

Both robots have 6 DOF in the same order: the body `(x, y, z, yaw)`, then
the camera pan and tilt. Pick a robot by its name, for example
`create_planner("drone")` or `OmplVampPlanner("drone")`.

### X500

The model is PX4's official Gazebo `x500_gimbal`
([PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models): `x500` +
`gimbal`), converted to URDF with its meshes and dimensions
(`scripts/px4_sdf_to_urdf.py`, see [docs/robot.md](docs/robot.md)). The
flight controller holds roll and pitch, so the body is a
velocity-commanded point with a heading: 6 DOF.

| Index | Joint | Group | Range |
|------:|-------|-------|-------|
| 0 | `base_x_joint` | base | −10 … 10 m (virtual, world frame) |
| 1 | `base_y_joint` | base | −10 … 10 m |
| 2 | `base_z_joint` | base | 0 … 10 m (z up) |
| 3 | `base_yaw_joint` | base | −π … π (counter-clockwise from +x) |
| 4 | `cgo3_vertical_arm_joint` | gimbal | pan, −π … π (positive turns right) |
| 5 | `cgo3_camera_joint` | gimbal | tilt, −135° … 45° (positive looks up) |

Planning subgroups (`drone_planning.x500.PLANNING_SUBGROUPS`):

- `x500_base` — 4 DOF: `(x, y, z, yaw)`, gimbal held
- `x500_whole_body` — 6 DOF: body + gimbal pan and tilt (alias `x500`)

The collision model is **126 spheres** (foam → SRDF → cricket, the
template's pipeline), covering every link's original PX4 mesh (the rotor
discs as swept by the spinning propellers) except for thin slivers;
`tests/test_collision_model.py` checks the coverage. Point obstacles are checked as points: the spheres
are the margin.

Kinodynamic limits are PX4 v1.17 default parameters (`drone_planning/x500.py`):

| | Velocity | Acceleration |
|---|---|---|
| horizontal `(x, y)`, norm | 12 m/s `MPC_XY_VEL_MAX` | 5 m/s² `MPC_ACC_HOR_MAX` |
| vertical `z` | −1.5 … 3 m/s `MPC_Z_VEL_MAX_DN/UP` | −3 … 4 m/s² `MPC_ACC_DOWN/UP_MAX` |
| yaw | 60 °/s `MPC_YAWRAUTO_MAX` | 20 °/s² `MPC_YAWRAUTO_ACC` |
| gimbal pan / tilt | 30 °/s `MNT_RATE_YAW/PITCH` | 97 / 300 rad/s² (model torque cap) |

### PHANTOM's drone

The drone of the PHANTOM planner (`phantom/robot/drone.urdf` in
H-tr/PHANTOM-Planner). The flight controller holds roll and pitch, so the
body is position and yaw. The body is a box of 0.46 × 0.46 × 0.12 m, and
the camera hangs 6 cm below the body centre. The collision model is **10
spheres** on the body: foam's 9 spheres of the box (radius 117–131 mm, a
flat 3 × 3 layer) and one sphere of 87.8 mm at the gimbal pivot, which
holds the camera at every pan and tilt. So the camera angles never change
whether the drone collides.

| Index | Joint | Group | Range |
|------:|-------|-------|-------|
| 0 | `base_x_joint` | base | −10 … 10 m (virtual, world frame) |
| 1 | `base_y_joint` | base | −10 … 10 m |
| 2 | `base_z_joint` | base | 0 … 10 m (z up) |
| 3 | `base_yaw_joint` | base | −π … π (counter-clockwise from +x) |
| 4 | `camera_pan` | gimbal | −90° … 90° (positive turns **left**) |
| 5 | `camera_tilt` | gimbal | −90° … 30° (positive looks up) |

Planning subgroups (`drone_planning.drone.PLANNING_SUBGROUPS`):

- `drone_base` — 4 DOF: `(x, y, z, yaw)`, gimbal held
- `drone_whole_body` — 6 DOF: body + gimbal pan and tilt (alias `drone`)

Kinodynamic limits are PHANTOM's (simulated values, not hardware ratings):

| | Velocity | Acceleration |
|---|---|---|
| `x`, `y`, `z`, each | 1.5 m/s | 1.0 m/s² |
| horizontal `(x, y)`, norm | 1.5 m/s | 1.0 m/s² |
| yaw | 1.5 rad/s | 3.0 rad/s² |
| camera pan / tilt | 1.0 rad/s | 2.0 rad/s² |

PHANTOM gives one limit per axis. FLASK also limits the horizontal norm,
so that limit takes the per-axis value: a diagonal flight is a little
slower than PHANTOM allows.

---

## Usage

```python
import numpy as np
from drone_planning.planning import create_planner
from drone_planning.types import KinodynamicConfig, PlannerConfig

planner = create_planner(
    "x500_whole_body",
    config=PlannerConfig(
        planner_name="rrtc",
        time_limit=1.0,
        joint_limits={"base_x_joint": (-0.5, 5.5), "base_y_joint": (-2.0, 2.0),
                      "base_z_joint": (0.3, 2.7)},       # the workspace
    ),
    pointcloud=cloud,                                     # (N, 3) float32
)

start = np.array([0.0, 1.5, 2.2, -1.57, 0.0, 0.0])        # x, y, z, yaw, pan, tilt
goal = np.array([5.0, -1.5, 0.6, 3.14, 1.0, -1.2])

result = planner.plan(start, goal)                        # geometric path
if result.success:
    print(result.path.shape)

kino = planner.plan_kinodynamic(start, goal, config=KinodynamicConfig(time_limit=1.0))
times, q, qd, qdd = kino.trajectory.sample_uniform(dt=0.02)
# qd[:, :4] = (x_dot, y_dot, z_dot, yaw_rate): PX4 offboard velocity setpoints
```

### Low-level API

The `_ompl_vamp` extension has the low-level API of the sister packages,
for callers (e.g. PHANTOM) that drive it directly:

```python
from drone_planning._ompl_vamp import KinodynamicSettings, OmplVampPlanner

planner = OmplVampPlanner("drone")                        # full body, 6 DOF
# OmplVampPlanner() and OmplVampPlanner("x500") are the X500.
planner.dimension()                                       # 6
lower, upper = planner.default_joint_limits()             # this robot's, full-body order
planner.set_joint_limits(lower, upper)
planner.set_subgroup([0, 1, 2, 3], full_configuration)    # body; gimbal frozen
planner.set_distance_grid(esdf, origin, spacing)          # replaces the grid
planner.add_pointcloud(points_float32_nx3, 0.0)           # replaces the cloud
result = planner.plan(start, goal, planner_name="rrtc", time_limit=1.0,
                      simplify=True, interpolate=False)
result.solved, result.path                                 # bool, (N, 4) array
result = planner.plan_kinodynamic(start, start_velocity, goal, 1.0, settings)
```

`OmplVampPlanner(robot, active_indices, frozen_config)` makes the
subgroup planner in one step. `solved` is true only for an exact solution
(an approximate OMPL solution, which stops short of the goal, is not
solved). `MotionPlanner._kinodynamic_settings` shows how
`KinodynamicSettings` is filled from the robot's limits.

**Distance grid (ESDF).** `set_distance_grid(distances, origin, spacing)`
takes a signed distance field: the distance (m) from each point of a
regular 3-D lattice to the nearest obstacle, positive in free space.
`distances` is a float32 `(X, Y, Z)` array in C order, `origin` is the
world position of lattice point `(0, 0, 0)` and `spacing` gives the
lattice step along x, y and z. A robot sphere is free when the
interpolated distance at its centre is at least its radius **and** the
sphere lies wholly inside the lattice's box. The planner reads the array
in place, without a copy, and keeps it alive, so change its values only
between queries. Other dtypes or orders raise `TypeError` (a converted
array would be a copy); `MotionPlanner.set_distance_grid` converts for
you. `remove_distance_grid()` and `has_distance_grid()` work like their
point-cloud versions.

---

## Results

Planning times and success rates of RRT-Connect (`plan()`) and FLASK on
the demo room are in
[docs/planning/kinodynamic.md](docs/planning/kinodynamic.md#results-on-the-demo-room),
and the cost of a collision check in
[docs/planning/index.md](docs/planning/index.md#standalone-collision-checking).

---

## Architecture

```
┌───────────────────────────────────────────────────────────────┐
│                   drone_planning/ (Python)                     │
│   create_planner(...) → MotionPlanner                          │
│     plan()  ·  plan_kinodynamic()  ·  validate_batch()         │
└────────┬───────────────────────────────────────────────────────┘
         │ one nanobind call
┌────────▼───────────────────────────────────────────────────────┐
│              _ompl_vamp  (C++/nanobind extension)              │
│  OMPL SimpleSetup, RealVectorStateSpace over the active joints │
│    RRT-Connect, RRT*, BIT*, AIT*, KPIECE, PRM, FMT, … (30)     │
│  FLASK: kinodynamic RRT-Connect, closed-form cubic edges       │
└────────┬───────────────────────────────────────────────────────┘
         │
┌────────▼───────────────────────────────────────────────────────┐
│              VAMP backend  (header-only, SIMD)                 │
│   vamp::robots::X500 (126 spheres), vamp::robots::Drone (1),   │
│   generated by cricket                                         │
│   point clouds in a CAPT built for Robot::max_radius;          │
│   distance grids read by trilinear interpolation               │
└────────────────────────────────────────────────────────────────┘
```

---

## Project layout

```
Drone-Planning/
├── CMakeLists.txt              # two extensions: _ompl_vamp, _time_parameterization
├── pyproject.toml              # scikit-build-core
├── pixi.toml                   # conda env + build tasks
├── .gitmodules                 # vamp, ompl, cricket, foam
│
├── drone_planning/             # Python package (mirrors fetch_planning/)
│   ├── x500.py                 # 6-DOF X500 layout (joint groups, HOME, PX4 limits, …)
│   ├── drone.py                # 6-DOF layout of PHANTOM's drone (PHANTOM's limits)
│   ├── types/                  # SE3Pose, PlannerConfig, KinodynamicConfig, …
│   ├── trajectory/             # TOPP-RA + FLASK trajectories
│   ├── kinematics/             # Pinocchio FK (e.g. the camera pose)
│   ├── planning/               # MotionPlanner, create_planner
│   ├── envs/                   # PyBullet scene wrapper
│   ├── utils/                  # rotations, PyBullet helpers, video recorder
│   └── resources/robot/        # x500/ (URDFs, SRDF, meshes), drone/ (URDFs, SRDF)
│
├── ext/
│   ├── ompl_vamp/              # nanobind OMPL↔VAMP bridge + FLASK
│   │   └── include/vamp/robots/  # x500.hh, drone.hh: generated by cricket
│   └── time_parameterization/  # vendored TOPP-RA C++ core (nanobind)
│
├── scripts/                    # build_robot.sh (PX4 SDF → URDF), spherize_robot.sh,
│                               # generate_fk.sh, cricket_configs/, render_videos/
├── examples/planning/          # motion, subgroup, kinodynamic, time parameterization
├── tests/                      # pytest suite
└── third_party/                # vamp, ompl, cricket, foam
```

Rebuild the X500 model end to end with `pixi run generate-fk` (PX4 SDF →
URDF → foam → cricket); it reproduces the committed files byte for byte.
The drone's model needs only the cricket step: `pixi run
generate-fk-drone` (`scripts/generate_fk.sh drone`).

---

## Development

```bash
git clone --recursive https://github.com/H-tr/Drone-Planning.git
cd Drone-Planning
bash scripts/setup.sh        # pixi install, build cricket + foam
pixi run test                # pytest suite
```

Dependencies: cmake, ninja, a C++17 compiler, nanobind, Boost
(serialization, filesystem), Eigen; pinocchio, CGAL, CppAD, inja for
cricket; trimesh, pycollada, pymeshlab for the model conversion. Pixi
installs all of them from conda-forge. Runtime (wheel): `numpy`, `pin`.

Wheels: `pipx run cibuildwheel --platform linux` (manylinux_2_34 and
manylinux_2_28, CPython 3.10–3.14). Releases: pushing a `v*` tag runs
`.github/workflows/release.yml`, which publishes to PyPI with Trusted
Publishing (environment `pypi`).

---

## License & attribution

- The X500 model (meshes, poses, inertials) is from
  [PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models)
  (BSD-3-Clause, © Rudis Laboratories; license in
  `drone_planning/resources/robot/x500/LICENSE`).
- VAMP (Apache-2.0, KavrakiLab), OMPL (BSD-3-Clause), foam and cricket
  (MIT, CoMMALab).
- The time parameterizer vendors the C++ core of
  [toppra](https://github.com/hungpham2511/toppra) (MIT, © Hung Pham)
  unmodified under `ext/time_parameterization/toppra/`.
- FLASK: Duong et al., "Ultrafast Sampling-based Kinodynamic Planning via
  Differential Flatness", T-RO 2026.
