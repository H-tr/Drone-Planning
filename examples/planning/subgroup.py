"""Plan both kinematic subgroups at different gimbal stances.

``x500_base`` plans the body (x, y, z, yaw) with the gimbal held at the
stance the caller passes in; ``x500_whole_body`` plans the gimbal too.
The stances below differ only in the gimbal; replace them with any 6-DOF
array (e.g. the live state of the drone) to plan around it.

    pixi run python examples/planning/subgroup.py
"""

import numpy as np
from _shared import LEGS, WORKSPACE, window_scene
from fire import Fire

from drone_planning.envs.pybullet_env import PyBulletEnv
from drone_planning.planning import create_planner
from drone_planning.types import PlannerConfig
from drone_planning.x500 import x500_robot_config

# Gimbal (pan, tilt) of three example stances.
STANCES = {
    "forward": (0.0, 0.0),
    "down": (0.0, -np.pi / 2),
    "left": (-np.pi / 2, -0.3),
}

SUBGROUPS = [
    "x500_base",  # 4 DOF: x, y, z, yaw — gimbal held
    "x500_whole_body",  # 6 DOF: body + gimbal pan, tilt
]


def main(planner_name: str = "rrtc", time_limit: float = 1.0):
    """Run the subgroup sweep with the chosen OMPL planner.

    Available planner names (pick one and pass as ``--planner_name``):

        RRT family ........... rrtc / rrtconnect, rrt, rrtstar,
                               informed_rrtstar, rrtsharp, rrtxstatic,
                               lbtrrt, trrt, bitrrt
        Informed trees ....... bitstar, abitstar, aitstar, eitstar, blitstar
        FMT .................. fmt, bfmt
        KPIECE ............... kpiece, bkpiece, lbkpiece
        PRM family ........... prm, prmstar, lazyprm, lazyprmstar,
                               spars, spars2
        Exploration-based .... est, biest, sbl, stride, pdst
    """
    env = PyBulletEnv(x500_robot_config, visualize=True)
    cloud = window_scene()
    env.add_pointcloud(cloud[::4], pointsize=2)
    config = PlannerConfig(
        planner_name=planner_name, time_limit=time_limit, joint_limits=WORKSPACE
    )
    _, start, goal = LEGS[0]

    for stance_name, gimbal in STANCES.items():
        base = start.copy()
        base[4:] = gimbal
        for robot_name in SUBGROUPS:
            planner = create_planner(
                robot_name, config=config, pointcloud=cloud, base_config=base
            )
            result = planner.plan(
                planner.extract_config(base), planner.extract_config(goal)
            )
            n_wp = result.path.shape[0] if result.path is not None else 0
            label = f"{robot_name} @ {stance_name}"
            print(f"  [{label}] {result.status.value} — {n_wp} waypoints")
            if not result.success:
                continue
            if not env.animate_path(planner.embed_path(result.path), next_key="n"):
                return

    env.wait_for_close()


if __name__ == "__main__":
    Fire(main)
