# Examples

All examples plan in the demo room of `examples/planning/_shared.py`: a
7 × 5 × 3 m room split by a wall with a 1.2 × 1.0 m window, and a low
beam across the room, as a 3 cm point cloud. The clips come from the
offscreen renderer in `scripts/render_videos/render_docs_videos.py`
(`pixi run render-docs-videos`), which draws the room as boxes.

---

## Motion planning

<video controls autoplay loop muted playsinline width="100%">
  <source src="../assets/motion_planning.mp4" type="video/mp4">
</video>

`examples/planning/motion.py` plans the whole body (body + gimbal) from
one corner of the room to the other: through the window, under the beam,
turning the camera to look down on arrival, and plays the path in
PyBullet.

```bash
pixi run -e dev python examples/planning/motion.py
pixi run -e dev python examples/planning/motion.py --planner_name bitstar --time_limit 3
```

```python
from _shared import LEGS, WORKSPACE, window_scene

from drone_planning.planning import create_planner
from drone_planning.types import PlannerConfig

planner = create_planner(
    "x500_whole_body",
    config=PlannerConfig(planner_name="rrtc", time_limit=2.0, joint_limits=WORKSPACE),
    pointcloud=window_scene(),
)
_, start, goal = LEGS[3]
result = planner.plan(start, goal)
```

---

## Subgroup planning

`examples/planning/subgroup.py` plans `x500_base` and `x500_whole_body`
from three gimbal stances (forward, down, left); `x500_base` keeps the
stance, `x500_whole_body` moves the gimbal to its goal. Press `n` for
the next plan.

```bash
pixi run -e dev python examples/planning/subgroup.py
```

---

## Kinodynamic planning

<video controls autoplay loop muted playsinline width="100%">
  <source src="../assets/kinodynamic.mp4" type="video/mp4">
</video>

`examples/planning/kinodynamic.py` benchmarks FLASK against RRT-Connect
(+ TOPP-RA) on every leg of the room for both subgroups, and checks every
trajectory against the PX4 limits (results in
[Kinodynamic planning](planning/kinodynamic.md#results-on-the-demo-room)).

```bash
pixi run -e dev python examples/planning/kinodynamic.py
pixi run -e dev python examples/planning/kinodynamic.py --trials=5 --visualize
```

---

## Time parameterization

`examples/planning/time_parameterization.py` times a synthetic 3-DOF
zig-zag with TOPP-RA and prints the peak velocity and acceleration
against the limits.

```bash
pixi run -e dev python examples/planning/time_parameterization.py
```
