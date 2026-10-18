"""Checks of the low-level ``OmplVampPlanner``.

Most use the body subgroup (x, y, z, yaw), the gimbal frozen at home,
among sphere obstacles.
"""

import numpy as np
import pytest

from drone_planning._ompl_vamp import OmplVampPlanner
from drone_planning.x500 import HOME_JOINTS

START = np.array([-1.5, 0.0, 1.0, 0.0])
GOAL = np.array([1.5, 0.0, 1.0, 0.0])


def body_planner():
    """Body (x, y, z, yaw) planner kept at 0.9-1.1 m, the gimbal frozen."""
    planner = OmplVampPlanner([0, 1, 2, 3], HOME_JOINTS.tolist())
    lower, upper = OmplVampPlanner.default_joint_limits()
    lower[:3], upper[:3] = [-3.0, -3.0, 0.9], [3.0, 3.0, 1.1]
    planner.set_joint_limits(lower, upper)
    return planner


@pytest.mark.parametrize("name", ["pdst", "kpiece", "bkpiece", "lbkpiece", "stride"])
def test_partial_motions_stop_at_obstacles(name):
    """These planners grow from the last valid state of a blocked motion."""
    planner = body_planner()
    # A wall across the straight line from START to GOAL, |y| <= 1.6.
    for y in np.arange(-1.5, 1.51, 0.1):
        planner.add_sphere([0.0, y, 1.0], 0.1)
    for _ in range(5):
        result = planner.plan(START, GOAL, name, 5.0, False, False, 0.0)
        assert result.solved
        # The body crosses x = 0 only beyond the ends of the wall.
        a, b = result.path[:-1], result.path[1:]
        cross = np.flatnonzero(np.sign(a[:, 0]) != np.sign(b[:, 0]))
        y = a[cross, 1] + (b[cross, 1] - a[cross, 1]) * a[cross, 0] / (
            a[cross, 0] - b[cross, 0]
        )
        assert np.all(np.abs(y) > 1.5)
