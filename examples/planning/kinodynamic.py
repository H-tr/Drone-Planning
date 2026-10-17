"""Kinodynamic (FLASK) planning on the demo room, against RRT-Connect.

Plans every leg of ``_shared.LEGS`` for both subgroups, ``trials`` times
each, and prints

* **FLASK** — success, planning time (median / p95), trajectory duration,
  and the largest velocity / acceleration as a fraction of its PX4 limit
  (the horizontal pair in norm), measured on a 1 ms resampling;
* **RRT-Connect** — success and planning time of the geometric planner
  (OMPL's solve, and the whole ``plan`` call including simplification),
  and the duration TOPP-RA gives its path under per-joint limits.

``--visualize`` plays the FLASK trajectories in PyBullet in real time.

Usage::

    pixi run python examples/planning/kinodynamic.py
    pixi run python examples/planning/kinodynamic.py --trials=5 --visualize
"""

from __future__ import annotations

import time

import numpy as np
from _shared import LEGS, WORKSPACE, window_scene
from fire import Fire

from drone_planning.planning import create_planner
from drone_planning.trajectory import TimeOptimalParameterizer
from drone_planning.types import KinodynamicConfig, PlannerConfig
from drone_planning.x500 import (
    HORIZONTAL_MAX_ACCELERATION,
    HORIZONTAL_MAX_SPEED,
    JOINT_ACCELERATION_LIMITS,
    JOINT_VELOCITY_LIMITS,
    PLANNING_SUBGROUPS,
    x500_robot_config,
)


def summarize(ms: list[float]) -> str:
    t = np.asarray(ms)
    return f"median {np.median(t):7.2f}  p95 {np.percentile(t, 95):7.2f} ms"


def limit_ratio(planner, traj) -> float:
    """Largest velocity / acceleration as a fraction of its limit."""
    _, _, qd, qdd = traj.sample_uniform(1e-3)
    names = planner.joint_names
    ratio = 0.0
    for values, limits in (
        (qd, JOINT_VELOCITY_LIMITS),
        (qdd, JOINT_ACCELERATION_LIMITS),
    ):
        lo = np.array([limits[j][0] for j in names])
        hi = np.array([limits[j][1] for j in names])
        ratio = max(ratio, (values / hi).max(), (values / lo).max())
    x, y = names.index("base_x_joint"), names.index("base_y_joint")
    ratio = max(
        ratio,
        np.hypot(qd[:, x], qd[:, y]).max() / HORIZONTAL_MAX_SPEED,
        np.hypot(qdd[:, x], qdd[:, y]).max() / HORIZONTAL_MAX_ACCELERATION,
    )
    return float(ratio)


def benchmark(cloud: np.ndarray, trials: int, time_limit: float) -> dict:
    """Plan every leg ``trials`` times per subgroup; return FLASK trajectories."""
    shown = {}
    for name in PLANNING_SUBGROUPS:
        planner = create_planner(
            name,
            config=PlannerConfig(
                time_limit=time_limit, interpolate=False, joint_limits=WORKSPACE
            ),
            pointcloud=cloud,
        )
        names = planner.joint_names
        vel = np.array(
            [
                min(-JOINT_VELOCITY_LIMITS[j][0], JOINT_VELOCITY_LIMITS[j][1])
                for j in names
            ]
        )
        acc = np.array(
            [
                min(-JOINT_ACCELERATION_LIMITS[j][0], JOINT_ACCELERATION_LIMITS[j][1])
                for j in names
            ]
        )
        toppra = TimeOptimalParameterizer(vel, acc)
        print(f"\n── {name} ({planner.num_dof} DOF) ──")
        for leg, start, goal in LEGS:
            start, goal = planner.extract_config(start), planner.extract_config(goal)
            flask_ms, durations, ratios = [], [], []
            rrtc_ms, rrtc_total_ms, toppra_durations = [], [], []
            for trial in range(trials):
                cfg = KinodynamicConfig(time_limit=time_limit, seed=trial + 1)
                r = planner.plan_kinodynamic(start, goal, config=cfg)
                flask_ms.append(r.planning_time_ns / 1e6)
                if r.success:
                    durations.append(r.trajectory.duration)
                    ratios.append(limit_ratio(planner, r.trajectory))
                    shown[(name, leg)] = (planner, r.trajectory)

                t0 = time.perf_counter()
                g = planner.plan(start, goal)
                rrtc_total_ms.append((time.perf_counter() - t0) * 1e3)
                rrtc_ms.append(g.planning_time_ns / 1e6)
                if g.success:
                    toppra_durations.append(toppra.parameterize(g.path).duration)
            print(
                f"  {leg:10s} FLASK {len(durations):3d}/{trials}  {summarize(flask_ms)}  "
                f"duration {np.median(durations):5.2f} s  peak/limit {max(ratios):.3f}\n"
                f"  {'':10s} RRTC  {len(toppra_durations):3d}/{trials}  {summarize(rrtc_ms)}  "
                f"(plan() {np.median(rrtc_total_ms):6.2f} ms)  TOPP-RA {np.median(toppra_durations):5.2f} s"
            )
    return shown


def main(trials: int = 20, time_limit: float = 1.0, visualize: bool = False) -> None:
    cloud = window_scene()
    print(f"demo room: {len(cloud)} points")
    shown = benchmark(cloud, trials, time_limit)
    if not visualize:
        return

    from drone_planning.envs.pybullet_env import PyBulletEnv

    env = PyBulletEnv(x500_robot_config, visualize=True)
    env.add_pointcloud(cloud[::4], pointsize=2)
    for (name, leg), (planner, traj) in shown.items():
        print(f"  playing {name} / {leg}: {traj.duration:.2f} s")
        times, q, _, _ = traj.sample_uniform(1.0 / 60.0)
        for row in planner.embed_path(q):
            env.set_configuration(row)
            time.sleep(1.0 / 60.0)
    env.wait_for_close()


if __name__ == "__main__":
    Fire(main)
