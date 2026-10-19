# Drone Planning

A planning library for the **Holybro X500 V2 quadrotor with a pan-tilt
gimbal camera** — geometric motion planning (OMPL frontend, VAMP SIMD
collision backend), kinodynamic planning (FLASK) under PX4's velocity and
acceleration limits, and time-optimal trajectory generation, all behind a
unified Python API.

<video controls autoplay loop muted playsinline width="100%">
  <source src="assets/kinodynamic.mp4" type="video/mp4">
</video>

<div class="grid cards" markdown>

-   :material-package-variant:{ .lg .middle } __Drop-in, few deps__

    ---

    For inference: **`numpy`, `pin`**. No conda required, no ROS, no
    MoveIt. One `pip install`.

-   :material-source-branch:{ .lg .middle } __31 planners__

    ---

    Full OMPL planner library, not a bespoke subset: RRT-Connect,
    RRT\*, BIT\*, AIT\*, EIT\*, KPIECE, PRM, FMT, SPARS, … over the
    body `(x, y, z, yaw)` and the gimbal `(pan, tilt)`.

-   :material-speedometer:{ .lg .middle } __Microsecond checks, millisecond plans__

    ---

    VAMP SIMD collision backend: microsecond checks of the 126-sphere
    model ([measured](planning/index.md#standalone-collision-checking));
    RRT-Connect and FLASK plan the demo room in milliseconds
    ([measured](planning/kinodynamic.md#results-on-the-demo-room)).

</div>

## Features

<div class="grid cards" markdown>

-   :material-quadcopter:{ .lg .middle } [__Robot Model__](robot.md)

    ---

    PX4's official `x500_gimbal` Gazebo model converted to URDF —
    meshes, poses and dimensions kept — spherized with foam and compiled
    to a VAMP collision model with cricket.

-   :material-map-marker-path:{ .lg .middle } [__Motion Planning__](planning/index.md)

    ---

    OMPL frontend + VAMP SIMD backend. Body-only and whole-body
    subgroups, point-cloud obstacles checked as points (the spheres are
    the margin).

-   :material-timer-outline:{ .lg .middle } [__Time Parameterization__](planning/trajectory.md)

    ---

    Time-optimal path parameterization (TOPP-RA) converts geometric paths
    into executable trajectories with per-joint velocity and acceleration
    limits.

-   :material-chart-bell-curve:{ .lg .middle } [__Kinodynamic Planning__](planning/kinodynamic.md)

    ---

    FLASK: a time-parameterised trajectory in the quadrotor's flat
    outputs, within PX4's horizontal speed / acceleration norms, climb and
    descent rates, yaw rate and gimbal rates — the velocity setpoint
    stream an offboard controller tracks.

</div>

## Quick Install

For inference — running the planners — just pip install the prebuilt
wheel (Linux x86_64, Python 3.10–3.14):

```bash
pip install drone-planning
```

Two runtime deps: `numpy`, `pin`. See the
[Getting Started](getting-started.md) guide for the full development
setup (pixi + conda-forge toolchain, robot model rebuilds, FK codegen).
