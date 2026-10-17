"""Geometric planning (OMPL RRT-Connect + VAMP) on the demo scene.

Every returned path is densified and re-checked for collisions
independently of the planner (``validate_batch``).
"""

import numpy as np
import pytest
from _shared import LEGS, WORKSPACE, window_scene
from conftest import assert_collision_free

from drone_planning.planning import create_planner
from drone_planning.types import PlannerConfig, PlanningStatus

CLOUD = window_scene()


@pytest.fixture(scope="module", params=["x500_base", "x500_whole_body"])
def planner(request):
    return create_planner(
        request.param,
        config=PlannerConfig(time_limit=2.0, joint_limits=WORKSPACE),
        pointcloud=CLOUD,
    )


@pytest.mark.parametrize("leg", LEGS, ids=[leg[0] for leg in LEGS])
def test_plan_is_collision_free(planner, leg):
    _, start, goal = leg
    start, goal = planner.extract_config(start), planner.extract_config(goal)
    result = planner.plan(start, goal)
    assert result.success, result.status
    path = result.path
    np.testing.assert_array_equal(path[0], start)
    np.testing.assert_array_equal(path[-1], goal)
    lower, upper = planner.bounds
    assert np.all(path >= lower) and np.all(path <= upper)
    # 1 mm / 1 mrad between re-checked configurations.
    assert_collision_free(planner, planner.interpolate_path(path, resolution=1000.0))


def test_straight_line_through_the_wall_is_blocked(planner):
    """The demo scene is not trivial: the straight segment collides."""
    _, start, goal = LEGS[0]
    start, goal = planner.extract_config(start), planner.extract_config(goal)
    line = start + np.linspace(0.0, 1.0, 500)[:, None] * (goal - start)
    assert not planner.validate_batch(line).all()


def test_invalid_start_is_reported(planner):
    _, start, goal = LEGS[0]
    inside_wall = planner.extract_config(start).copy()
    inside_wall[0] = 2.0
    assert planner.plan(inside_wall, planner.extract_config(goal)).status == (
        PlanningStatus.INVALID_START
    )
