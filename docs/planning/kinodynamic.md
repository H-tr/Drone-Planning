# Kinodynamic Planning (FLASK)

`MotionPlanner.plan_kinodynamic` plans a **time-parameterised,
dynamically feasible trajectory** directly — no separate time
parameterization step. It implements FLASK (Duong et al., "Ultrafast
Sampling-based Kinodynamic Planning via Differential Flatness", T-RO
2026): RRT-Connect in the differentially flat output space, where every
tree edge is a closed-form cubic that is collision-checked with the same
SIMD VAMP kernels as the geometric planner.

<video controls loop muted playsinline width="100%">
  <source src="../../assets/kinodynamic.mp4" type="video/mp4">
</video>

```python
from drone_planning.planning import create_planner
from drone_planning.types import KinodynamicConfig

planner = create_planner("x500_whole_body", pointcloud=cloud)
result = planner.plan_kinodynamic(start, goal, config=KinodynamicConfig(time_limit=1.0))

traj = result.trajectory                      # KinodynamicTrajectory
times, q, qd, qdd = traj.sample_uniform(dt=0.02)
# qd[:, :4] = (x_dot, y_dot, z_dot, yaw_rate): the PX4 offboard velocity setpoint
# qd[:, 4:] = gimbal pan / tilt rates
```

It works for both subgroups (`x500_base`, `x500_whole_body`).

## How it works

<div class="grid cards" markdown>

-   __Flat outputs__

    ---

    A quadrotor is differentially flat in its position and yaw. With roll
    and pitch held by the flight controller, `(x, y, z, yaw)` are all
    commanded directly — no nonholonomic constraint — and the gimbal pan
    and tilt are flat outputs of their own.

-   __Closed-form edges__

    ---

    Nodes are `(position, velocity)` states. An edge is the cubic
    Hermite between its endpoints — the minimum-effort (LQMT) motion —
    with the duration `T*` that minimises
    `sum_i int y_i''^2 / a_i^2 dt + rho T` (a quartic root).

-   __Limits, exact__

    ---

    Checked exactly on each cubic: joint ranges, per-joint velocity and
    acceleration ranges (asymmetric for the climb rate), and the **norm**
    of the horizontal velocity (the roots of `v · a`, a cubic) and
    acceleration (affine in time, so largest at an end). `T*` is
    stretched until they hold.

-   __SIMD collision checks__

    ---

    Each edge is sampled at VAMP's resolution along its arc length and
    checked in interleaved `ConfigurationBlock` batches (FLASK Alg. 3),
    then the solution is simplified the way VAMP simplifies geometric
    paths: greedy shortcuts (Alg. 4) plus subdivide-and-cut-corners
    smoothing, keeping a change only if it lowers the cost.

</div>

## The limits

They come from PX4's default parameters (see [Robot Model](../robot.md#limits))
and can be scaled with `KinodynamicConfig.velocity_scale` /
`acceleration_scale`:

| Flat output | Velocity | Acceleration |
|---|---|---|
| horizontal `(x, y)`, norm | 12 m/s (`MPC_XY_VEL_MAX`) | 5 m/s² (`MPC_ACC_HOR_MAX`) |
| vertical `z` | −1.5 … 3 m/s (`MPC_Z_VEL_MAX_DN/UP`) | −3 … 4 m/s² (`MPC_ACC_DOWN/UP_MAX`) |
| yaw | 60 °/s (`MPC_YAWRAUTO_MAX`) | 20 °/s² (`MPC_YAWRAUTO_ACC`) |
| gimbal pan / tilt | 30 °/s (`MNT_RATE_YAW/PITCH`) | 97 / 300 rad/s² (gimbal model torque) |

The yaw is a bounded joint in `[−π, π]` like in the geometric planner.

## Tracking the trajectory

The trajectory is C¹ in every flat output: positions and velocities are
continuous; accelerations jump at segment knots. Stream
`traj.velocity(t)` as the velocity / yaw-rate setpoint (world frame, z
up — convert to NED for PX4) and the gimbal rates, with position feedback
on `traj.position(t)`.

Pass the current state to replan on the fly — the new trajectory starts
with the given velocity:

```python
q_now, qd_now = traj.position(t), traj.velocity(t)
result = planner.plan_kinodynamic(q_now, new_goal, start_velocity=qd_now)
```

Trajectories are collision-free with zero clearance, up to the sampling
resolution like every VAMP edge: the spheres are the margin.

## Configuration

`KinodynamicConfig` holds the planner parameters; the limits come from
`drone_planning/x500.py`.

| Field | Default | Meaning |
|---|---|---|
| `time_limit` | `1.0` | Planning budget (s) |
| `rho` | `1.0` | Time weight; rest-to-rest edges peak at `sqrt(rho)` × the acceleration limit |
| `velocity_scale` / `acceleration_scale` | `1.0` | Scale every limit |
| `max_extension_time` | `3.0` | Tree step: longer cubics are truncated (node stays on the cubic) |
| `rest_sample_probability` | `0.3` | Share of zero-velocity samples (straight-line edges) |
| `simplify_iterations` / `simplify_time_limit` | `5` / `0.05` | Rounds and time cap for the VAMP-style simplification |
| `seed` | `0` | RNG seed (`0` = nondeterministic) |

## Results on the demo room

`examples/planning/kinodynamic.py` plans the four legs of the demo room
(`examples/planning/_shared.py`: a wall with a 1.2 × 1.0 m window and a
low beam, 100k points) 50 times each. Planning times per call on one core
of an AMD EPYC 7543; "peak / limit" is the largest velocity or
acceleration over all trajectories as a fraction of its limit (horizontal
pair in norm), on a 1 ms resampling.

| Subgroup | Leg | FLASK success | FLASK median / p95 | Duration | Peak / limit | RRT-Connect median (solve / `plan()`) |
|---|---|---|---|---|---|---|
| `x500_base` | through | 50/50 | 11.6 / 25.9 ms | 9.7 s | 1.000 | 2.5 / 17.4 ms |
| `x500_base` | back | 50/50 | 9.4 / 25.1 ms | 9.8 s | 1.000 | 2.3 / 13.4 ms |
| `x500_base` | diagonal | 50/50 | 6.4 / 18.4 ms | 5.8 s | 1.000 | 1.2 / 6.8 ms |
| `x500_base` | look down | 50/50 | 9.4 / 23.9 ms | 9.1 s | 1.000 | 2.3 / 15.4 ms |
| `x500_whole_body` | through | 50/50 | 11.0 / 29.1 ms | 9.7 s | 1.000 | 3.3 / 18.0 ms |
| `x500_whole_body` | back | 50/50 | 12.3 / 29.6 ms | 9.8 s | 1.000 | 3.4 / 16.0 ms |
| `x500_whole_body` | diagonal | 50/50 | 5.6 / 18.4 ms | 5.8 s | 1.000 | 1.2 / 7.5 ms |
| `x500_whole_body` | look down | 50/50 | 12.5 / 28.6 ms | 9.4 s | 1.000 | 2.3 / 15.4 ms |

RRT-Connect succeeds 50/50 on every leg; `plan()` includes OMPL's path
simplification. TOPP-RA times the RRT-Connect paths, under per-joint
limits, in 5.8–9.2 s.

Takeaways:

* FLASK returns a trajectory within every limit in about 10 ms; the
  geometric planner finds a path in 1–3 ms but needs simplification and
  time parameterization on top.
* Durations are set by the slow yaw (20 °/s²) and gimbal (30 °/s)
  limits more than by the 12 m/s speed: tree nodes sample random yaw and
  gimbal angles, and shortcuts that remove a detour are kept only when
  collision-free, so FLASK trajectories carry yaw excursions that a
  geometric path smoothed by OMPL loses.
