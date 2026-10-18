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

The X500's flight controller holds roll and pitch, so the body is fully
actuated in `(x, y, z, yaw)`: every subgroup is a plain bounded
`RealVectorStateSpace`, edges are straight lines in joint space, and the
yaw is a bounded joint in `[−π, π]` (a path from 3 rad to −3 rad turns
the long way round, like every revolute joint). For timed motions within
PX4's velocity and acceleration limits use
[kinodynamic planning](kinodynamic.md).

## Supported planners

The `planner_name` field of
[`PlannerConfig`](../api/types.md#drone_planning.types.planning.PlannerConfig)
accepts any of:

=== "Feasibility (single-query, returns on first solution)"

    | Family | Planners |
    |---|---|
    | RRT | `rrtc`, `rrt`, `trrt`, `bitrrt`, `lbtrrt` |
    | KPIECE | `kpiece`, `bkpiece`, `lbkpiece` |
    | PRM | `prm`, `lazyprm`, `spars`, `spars2` |
    | Exploration | `est`, `biest`, `sbl`, `stride`, `pdst` |
    | RRT\*-feasible | `rrtsharp`, `rrtxstatic` |

=== "Asymptotically optimal (keep refining until budget)"

    | Family | Planners |
    |---|---|
    | RRT\* | `rrtstar`, `informed_rrtstar` |
    | Informed trees | `bitstar`, `abitstar`, `aitstar`, `eitstar`, `blitstar` |
    | FMT | `fmt`, `bfmt` |
    | PRM\* | `prmstar`, `lazyprmstar` |

A plan is `success` only for an exact solution: OMPL's approximate
solutions, which stop short of the goal, are reported as `FAILED`.

## Minimal example

<video controls loop muted playsinline width="100%">
  <source src="../../assets/motion_planning.mp4" type="video/mp4">
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

## Configuring a plan

Every plan is driven by a single
[`PlannerConfig`](../api/types.md#drone_planning.types.planning.PlannerConfig)
dataclass. The knobs that matter in practice:

```python
PlannerConfig(
    # Which OMPL planner to run — see "Supported planners" above.
    planner_name="rrtc",

    # Wall-clock ceiling for the search.  Feasibility planners return
    # on the first solution, so this is just a safety cap.  AO planners
    # (rrtstar, bitstar, …) run until the budget to refine cost.
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
dense  = planner.interpolate_path(smooth, resolution=128.0)     # or count=200
```

`simplify_path` only consults the motion validator; `interpolate_path`
runs `StateSpace::interpolate` on the existing edges and does not perform
collision checks itself.

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

planner = OmplVampPlanner()                       # full body, 6 DOF
lower, upper = planner.default_joint_limits()     # full-body order
planner.set_joint_limits(lower, upper)            # e.g. the map extent
planner.set_subgroup([0, 1, 2, 3], full_config)   # plan the body, gimbal frozen
planner.add_pointcloud(points_float32_nx3, 0.0)   # replaces the previous cloud
result = planner.plan(start, goal, planner_name="rrtc", time_limit=1.0,
                      simplify=True, interpolate=False)
result.solved, result.path                         # bool, (N, 4) array
```

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
