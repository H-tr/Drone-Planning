# Subgroup Planning

The X500 exposes 6 joints: the body `(x, y, z, yaw)` — virtual joints,
since the flight controller holds roll and pitch — and the gimbal
`(pan, tilt)`. A subgroup is an ordered list of indices into this 6-DOF
configuration: the planner reasons in the reduced joint space while the
C++ collision pipeline injects the frozen stance for the other joints and
still checks the **full** body.

## Built-in aliases

| Alias | DOF | What it controls |
|---|---:|---|
| `x500_base` | 4 | body `(x, y, z, yaw)`; the gimbal holds its pose |
| `x500_whole_body` | 6 | body + gimbal pan, tilt |
| `x500` | 6 | alias for `x500_whole_body` |

The list is a dict in `drone_planning.x500.PLANNING_SUBGROUPS`, also
listed by
[`available_robots()`](../api/planning.md#drone_planning.planning.motion_planner.available_robots).

## Using a built-in alias

```python
from drone_planning.planning import create_planner
from drone_planning.types import PlannerConfig

live_config = drone.get_state()                 # 6-D array from your system

planner = create_planner(
    "x500_base",
    config=PlannerConfig(time_limit=1.0),
    base_config=live_config,                    # the gimbal is held here
    pointcloud=obstacle_cloud,
)

start = planner.extract_config(live_config)     # 6-D → 4-D
result = planner.plan(start, goal_body)

full_path = planner.embed_path(result.path)     # 4-D path → 6-D path
```

`base_config` defaults to `HOME_JOINTS` (hovering 1 m above the origin,
gimbal level and forward). Pass the live state instead to pin the
gimbal where it is — collision checks see the real stance.

`set_subgroup(robot_name, base_config)` switches an existing planner to
another subgroup without rebuilding the collision environment; the
point cloud and the joint limits (`set_joint_limits`) are kept.

## Low-level subgroups

Any ordered list of joint indices is a valid subgroup with the C++
binding:

```python
import numpy as np
from drone_planning._ompl_vamp import OmplVampPlanner
from drone_planning.x500 import HOME_JOINTS

planner = OmplVampPlanner([2, 3], HOME_JOINTS.tolist())   # altitude + yaw only
planner.add_pointcloud(cloud, 0.0)                        # (N, 3) float32
result = planner.plan(np.array([1.0, 0.0]), np.array([2.0, 1.5]),
                      planner_name="rrtc", time_limit=1.0)
```

The binding takes numpy arrays (float64 configurations, float32 points).

`set_subgroup(active_indices, frozen_config)` switches it in place.

## When it matters

The gimbal never collides with the body (see the SRDF in
[Robot Model](../robot.md)), but it hangs 14–18 cm below it and can touch
obstacles: plan with `x500_whole_body` when the camera has to point
somewhere on arrival, and with `x500_base` (a 4-D search) when the gimbal
can stay where it is.
