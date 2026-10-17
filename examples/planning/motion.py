"""Motion planning through a window: plan a collision-free flight.

Loads the demo room (``_shared.py``): a wall with a window and a low beam
between the start and the goal.  The 6-DOF planner (body + gimbal) has to
climb through the window and pass under or over the beam while the
gimbal turns to its goal pose.  The C++ collision checker sees every
point as a point (``point_radius`` 0): the X500's spheres already cover
its meshes.

    pixi run python examples/planning/motion.py
    pixi run python examples/planning/motion.py --planner_name bitstar --time_limit 3
"""

from _shared import LEGS, WORKSPACE, window_scene
from fire import Fire

from drone_planning.envs.pybullet_env import PyBulletEnv
from drone_planning.planning import create_planner
from drone_planning.types import PlannerConfig
from drone_planning.x500 import x500_robot_config


def main(planner_name: str = "rrtc", time_limit: float = 2.0) -> None:
    env = PyBulletEnv(x500_robot_config, visualize=True)

    cloud = window_scene()
    env.add_pointcloud(cloud[::4], pointsize=2)

    planner = create_planner(
        "x500_whole_body",
        config=PlannerConfig(
            planner_name=planner_name,
            time_limit=time_limit,
            joint_limits=WORKSPACE,
        ),
        pointcloud=cloud,
    )

    _, start, goal = LEGS[3]
    result = planner.plan(start, goal)
    n = result.path.shape[0] if result.path is not None else 0
    print(
        f"── motion planning: through the window, under the beam ──\n"
        f"  planner={planner_name}  cloud={len(cloud)} pts\n"
        f"  {result.status.value}: {n} waypoints in "
        f"{result.planning_time_ns / 1e6:.1f} ms, cost {result.path_cost:.2f}"
    )

    if result.success:
        env.animate_path(result.path, fps=60)
    else:
        env.wait_for_close()


if __name__ == "__main__":
    Fire(main)
