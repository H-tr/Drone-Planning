"""Render the documentation videos in one pass.

Runs entirely in ``DIRECT`` mode (no GUI), uses the offscreen
TinyRenderer and pipes frames into ``ffmpeg`` via
:class:`drone_planning.utils.video_recorder.VideoRecorder`.
Output files land in ``docs/assets/``.  The demo room is drawn as boxes
(the planners see it as the point cloud of ``examples/planning/_shared.py``),
with the wall cut away on the camera side.

Clips produced, both played in real time:
  * ``motion_planning.mp4`` — RRT-Connect path of the whole body (body +
                              gimbal), timed by TOPP-RA
  * ``kinodynamic.mp4``     — FLASK trajectory of the same flight

Usage:
    pixi run python scripts/render_videos/render_docs_videos.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO_ROOT / "examples" / "planning"))

from _shared import LEGS, WORKSPACE, window_scene  # noqa: E402

from drone_planning.envs.pybullet_env import PyBulletEnv  # noqa: E402
from drone_planning.planning import create_planner  # noqa: E402
from drone_planning.trajectory import TimeOptimalParameterizer  # noqa: E402
from drone_planning.types import KinodynamicConfig, PlannerConfig  # noqa: E402
from drone_planning.utils.video_recorder import VideoRecorder  # noqa: E402
from drone_planning.x500 import (  # noqa: E402
    JOINT_ACCELERATION_LIMITS,
    JOINT_VELOCITY_LIMITS,
    x500_robot_config,
)

ASSETS_DIR = _REPO_ROOT / "docs" / "assets"
FPS = 30


def draw_room(env: PyBulletEnv) -> None:
    """Floor, the window wall without its camera-side half, and the beam."""
    wall = (0.72, 0.74, 0.78, 1.0)
    env.draw_box([2.5, 0.0, -0.01], [3.5, 2.5, 0.01], (0.88, 0.89, 0.91, 1.0))
    env.draw_box([2.0, 1.55, 1.5], [0.01, 0.95, 1.5], wall)  # beside the window
    env.draw_box([2.0, 0.0, 0.5], [0.01, 0.6, 0.5], wall)  # below it
    env.draw_box([2.0, 0.0, 2.5], [0.01, 0.6, 0.5], wall)  # above it
    env.draw_box([4.1, 0.0, 1.25], [0.1, 2.5, 0.15], (0.80, 0.45, 0.25, 1.0))


def main() -> None:
    cloud = window_scene()
    planner = create_planner(
        "x500_whole_body",
        config=PlannerConfig(interpolate=False, joint_limits=WORKSPACE),
        pointcloud=cloud,
    )
    _, start, goal = LEGS[3]

    names = planner.joint_names
    vel = np.array(
        [min(-JOINT_VELOCITY_LIMITS[j][0], JOINT_VELOCITY_LIMITS[j][1]) for j in names]
    )
    acc = np.array(
        [
            min(-JOINT_ACCELERATION_LIMITS[j][0], JOINT_ACCELERATION_LIMITS[j][1])
            for j in names
        ]
    )
    path = planner.plan(start, goal).path
    timed = TimeOptimalParameterizer(vel, acc).parameterize(path)
    flask = planner.plan_kinodynamic(
        start, goal, config=KinodynamicConfig(seed=1)
    ).trajectory

    env = PyBulletEnv(x500_robot_config, visualize=False)
    draw_room(env)
    for name, traj in (("motion_planning", timed), ("kinodynamic", flask)):
        _, q, _, _ = traj.sample_uniform(1.0 / FPS)
        env.set_configuration(q[0])
        with VideoRecorder(env, ASSETS_DIR / f"{name}.mp4", fps=FPS) as rec:
            rec.hold(seconds=0.5)
            rec.play(q)
            rec.hold(seconds=1.0)
        print(f"  {name}.mp4: {traj.duration:.1f} s")


if __name__ == "__main__":
    main()
