# Motion Planning

<div class="grid cards" markdown>

-   __OMPL frontend__

    ---

    All of [OMPL](https://ompl.kavrakilab.org/)'s mature sampling-based
    planners (30 exposed), driven by one uniform Python call. No glue
    code to swap planners — change a string.

-   __VAMP backend__

    ---

    Collision checking runs through [VAMP](https://github.com/KavrakiLab/vamp)'s
    SIMD-vectorised sphere pipeline. Point-cloud obstacles are
    broadphase-indexed; a check takes microseconds (see
    [below](#standalone-collision-checking)).

-   __C++ hot path__

    ---

    Python calls `plan(start, goal)` once. Everything inside —
    sampling, NN queries, collision checks, path simplification —
    runs in C++ behind a single nanobind call.

</div>

## Architecture

```
         Python
           │  planner.plan(start, goal)
           ▼
    ┌─────────────────────────────────────────┐
    │     OMPL SimpleSetup  (C++)              │
    │  ┌────────────┐  ┌──────────────────┐    │
    │  │  Planner   │→ │ MotionValidator  │    │
    │  │  RRT-C,    │  │  checkMotion     │    │
    │  │  BIT*, …   │  └────────┬─────────┘    │
    │  └────────────┘           │              │
    │                           ▼              │
    │                  ┌─────────────────┐     │
    │                  │ VAMP SIMD       │     │
    │                  │  sphere-cloud   │     │
    │                  │  collision      │     │
    │                  └─────────────────┘     │
    └─────────────────────────────────────────┘
```

The frontend is vanilla OMPL — `RealVectorStateSpace`, `SimpleSetup`.
The collision checker is the only replaced component: it swaps OMPL's
per-state user callback for a VAMP pipeline that batches many
sphere/point checks into a single AVX2 kernel.

On both robots (the X500 and PHANTOM's drone, see
[Robot Models](../robot.md)) the flight controller holds roll and pitch,
so the body is fully actuated in `(x, y, z, yaw)`: every subgroup is a
plain bounded `RealVectorStateSpace`, edges are straight lines in joint
space, and the yaw is a bounded joint in `[−π, π]` (a path from 3 rad to
−3 rad turns the long way round, like every revolute joint). For timed
motions within the robot's velocity and acceleration limits use
[kinodynamic planning](kinodynamic.md).

## Supported planners

The `planner_name` field of
[`PlannerConfig`](../api/types.md#drone_planning.types.planning.PlannerConfig)
accepts any of:

=== "Return on the first solution"

    | Family | Planners |
    |---|---|
    | RRT | `rrtc`, `rrt`, `trrt`, `bitrrt` |
    | KPIECE | `kpiece`, `bkpiece`, `lbkpiece` |
    | PRM | `prm`, `lazyprm` |
    | Exploration | `est`, `biest`, `sbl`, `stride`, `pdst` |
    | FMT | `fmt`, `bfmt` (after one batch of samples) |

=== "Run the whole budget (refine the cost)"

    | Family | Planners |
    |---|---|
    | RRT\* | `rrtstar`, `informed_rrtstar`, `rrtsharp`, `rrtxstatic`, `lbtrrt` |
    | Informed trees | `bitstar`, `abitstar`, `aitstar`, `eitstar`, `blitstar` |
    | PRM\* | `prmstar`, `lazyprmstar` |
    | Sparse roadmaps | `spars`, `spars2` |

Which group a planner falls in was measured: one solve per planner on
the first demo-room leg for `x500_base` with a 1 s budget, at commit
`580687f` on 2026-10-19 on an AMD EPYC 7543 (raw output in
`/nas/pollux/tianrun/drone-planning-flask/g1b_planner_budget.txt`); the
second group's solves took the whole second, the first group's at most
0.06 s:

```bash
pixi run python -c "import sys; sys.path.insert(0, 'examples/planning'); from _shared import LEGS, WORKSPACE, window_scene; from drone_planning.planning import create_planner; from drone_planning.types import PlannerConfig; from tests.test_planner_checks import OMPL_NAMES; _, s, g = LEGS[0]; [print(n, create_planner('x500_base', config=PlannerConfig(planner_name=n, time_limit=1.0, joint_limits=WORKSPACE), pointcloud=window_scene()).plan(s[:4], g[:4]).planning_time_ns / 1e9, flush=True) for n in OMPL_NAMES]"
```

A plan is `success` only for an exact solution: OMPL's approximate
solutions, which stop short of the goal, are reported as `FAILED`.

## Minimal example

<video controls loop muted playsinline width="100%">
  <source src="../assets/motion_planning.mp4" type="video/mp4">
</video>

```python
from drone_planning.planning import create_planner
from drone_planning.types import PlannerConfig

planner = create_planner(
    "x500_whole_body",
    config=PlannerConfig(
        planner_name="rrtc",
        time_limit=1.0,
        # The room the drone may fly in (the base joints span ±10 m otherwise).
        joint_limits={"base_x_joint": (-0.5, 5.5), "base_y_joint": (-2.0, 2.0),
                      "base_z_joint": (0.3, 2.7)},
    ),
    pointcloud=obstacle_cloud,           # (N, 3) np.float32
)

start = [0.0, 1.5, 2.2, -1.57, 0.0, 0.0]  # x, y, z, yaw, pan, tilt
goal = [5.0, -1.5, 0.6, 3.14, 1.0, -1.2]
result = planner.plan(start, goal)
print(result.success, result.planning_time_ns * 1e-6, "ms")
```

For PHANTOM's drone, use `"drone_whole_body"` (or `"drone"`) and
`"drone_base"` instead of the `x500_*` names.

## Configuring a plan

Every plan is driven by a single
[`PlannerConfig`](../api/types.md#drone_planning.types.planning.PlannerConfig)
dataclass. The knobs that matter in practice:

```python
PlannerConfig(
    # Which OMPL planner to run — see "Supported planners" above.
    planner_name="rrtc",

    # Wall-clock ceiling for the search.  The first group above returns
    # on the first solution, so this is just a safety cap; the others
    # (rrtstar, bitstar, spars, …) run until the budget to refine cost.
    time_limit=1.0,

    # Inflation of every obstacle point.  0 (default): points are checked
    # as points — the X500's spheres already cover its meshes.
    point_radius=0.0,

    # After the raw path, run OMPL's SimpleSetup shortcutter.
    simplify=True,

    # Resample the (simplified) path densely:
    #   resolution > 0.0        → ceil(edge_length * resolution) samples per edge
    #   0                       → OMPL's default longest-valid-segment fraction
    interpolate=True,
    resolution=64.0,

    # Joint ranges the planner samples in, by joint name (the workspace).
    joint_limits={"base_z_joint": (0.3, 2.7)},
)
```

## Post-hoc simplify / interpolate

`simplify` and `interpolate` run inside `plan(...)` by default, but the
same pipeline is exposed as standalone methods on `MotionPlanner`:

```python
result = planner.plan(start, goal)
smooth = planner.simplify_path(result.path, time_limit=1.0)
dense  = planner.interpolate_path(smooth, resolution=128.0)     # or count=200, resolution=0.0
```

`simplify_path` only consults the motion validator; `interpolate_path`
runs `StateSpace::interpolate` on the existing edges and does not perform
collision checks itself.

## Distance-grid environment

A mapped scene can also come as a signed distance field (an ESDF)
instead of a point cloud. The field gives, at each point of a regular
3-D lattice, the distance (m) to the nearest obstacle: positive in free
space, negative inside an obstacle. VAMP reads it by trilinear
interpolation between the 8 lattice points around a robot sphere's
centre. A sphere is free when this distance is at least its radius
**and** the sphere lies wholly inside the lattice's box (from `origin`
to the last lattice point). So the box is also the workspace: a robot
that reaches out of the box collides.

```python
planner.set_distance_grid(
    distances,              # (X, Y, Z) signed distances, m, positive in free space
    origin=grid_origin,     # world position of lattice point (0, 0, 0)
    spacing=0.05,           # m, one value or one per axis
)
planner.has_distance_grid  # True
planner.remove_distance_grid()
```

Lattice point `(i, j, k)` sits at `origin + (i, j, k) * spacing`. The
planner reads a float32 C-order array in place, without a copy, and keeps
it alive, so its values must be finite and change only between queries.
`MotionPlanner.set_distance_grid` converts any other array to such a copy
first. The low-level `OmplVampPlanner.set_distance_grid(distances, origin,
spacing)` takes only a float32 C-order `(X, Y, Z)` array and raises
`TypeError` for anything else, because a converted array would be a copy;
`origin` and `spacing` are 3 values each there. A lattice with fewer than
2 points on an axis, or a spacing that is not positive, raises
`ValueError` and keeps the grid in effect.

A planner holds at most one grid and one point cloud; calling
`set_distance_grid` again replaces the grid, and both apply when both are
set. `clear_environment()` drops the grid too.

## Standalone collision checking

```python
planner.validate(cfg)                   # single config → bool
planner.validate_batch(cfgs)            # (N, ndof) → (N,) bool array
```

`validate_batch` packs up to `rake` distinct configurations directly
into a VAMP `ConfigurationBlock<rake>` and runs **one** `fkcc<rake>`
call per block. Per-config result is preserved: when a packed block
fails, only that block is re-checked configuration by configuration.

It costs 2.65 μs per configuration for 100,000 uniform random whole-body
configurations in the demo room (99,715 points), measured at commit
`2fefdc3` on 2026-10-19 on one core of an AMD EPYC 7543 (raw output in
`/nas/pollux/tianrun/drone-planning-flask/g1_validate_batch.txt`):

```bash
pixi run python -c "import sys, time, numpy as np; sys.path.insert(0, 'examples/planning'); from _shared import WORKSPACE, window_scene; from drone_planning.planning import create_planner; from drone_planning.types import PlannerConfig; p = create_planner('x500', config=PlannerConfig(joint_limits=WORKSPACE), pointcloud=window_scene()); lo, hi = p.bounds; q = np.random.default_rng(0).uniform(lo, hi, (100000, 6)); p.validate_batch(q[:1000]); t = time.perf_counter(); p.validate_batch(q); print(f'{(time.perf_counter() - t) / len(q) * 1e6:.2f} us per configuration')"
```

Like every VAMP edge, a planned path is collision-free up to the motion
resolution: edges are checked every 1/64 of configuration distance
(1.6 cm of translation), so a point obstacle can graze a sphere between
two checks.

## Low-level API

`drone_planning._ompl_vamp.OmplVampPlanner` is the same extension API
as the sister packages (Autolife-Planning, Fetch-Planning), for callers
that drive the planner directly:

```python
from drone_planning._ompl_vamp import OmplVampPlanner

planner = OmplVampPlanner("drone")                # full body, 6 DOF
lower, upper = planner.default_joint_limits()     # this robot's, full-body order
planner.set_joint_limits(lower, upper)            # e.g. the map extent
planner.set_subgroup([0, 1, 2, 3], full_config)   # plan the body, gimbal frozen
planner.set_distance_grid(esdf, origin, spacing)  # replaces the previous grid
result = planner.plan(start, goal, planner_name="rrtc", time_limit=1.0,
                      simplify=True, interpolate=False)
result.solved, result.path                         # bool, (N, 4) array
```

The robot is `"x500"` or `"drone"`; `OmplVampPlanner()` without a name is
the X500, as before. `OmplVampPlanner(robot, active_indices,
frozen_config)` builds a subgroup planner in one step, and
`OmplVampPlanner(active_indices, frozen_config)` still builds an X500
subgroup planner. On a planner, `default_joint_limits()` gives the limits
of its robot; on the class, `OmplVampPlanner.default_joint_limits()`
gives the X500's and `OmplVampPlanner.default_joint_limits("drone")` the
drone's.

`plan_kinodynamic(start, start_velocity, goal, time_limit, settings)` is
the kinodynamic entry point on the same object.

## More

<div class="grid cards" markdown>

-   [__Subgroup planning__](subgroup.md)

    ---

    Plan the body with the gimbal held, or body and gimbal together,
    with the other joints pinned to any 6-DOF stance.

-   [__Time parameterization__](trajectory.md)

    ---

    Convert geometric paths into time-optimal trajectories with
    per-joint velocity and acceleration limits. TOPP-RA (Pham & Pham)
    in C++, one call from Python.

-   [__Kinodynamic planning__](kinodynamic.md)

    ---

    FLASK: plan a time-parameterised trajectory directly, within PX4's
    velocity and acceleration limits.

</div>
