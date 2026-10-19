"""Checks of the low-level ``OmplVampPlanner`` and the planner names.

Most use the body subgroup (x, y, z, yaw), the gimbal frozen at home,
among sphere obstacles.
"""

import re

import numpy as np
import pytest
from _shared import LEGS, WORKSPACE, window_scene

from drone_planning._ompl_vamp import OmplVampPlanner
from drone_planning.planning import create_planner
from drone_planning.types import PlannerConfig
from drone_planning.x500 import HOME_JOINTS

START = np.array([-1.5, 0.0, 1.0, 0.0])
GOAL = np.array([1.5, 0.0, 1.0, 0.0])

# Planner name -> the name OMPL logs for the planner it builds.
OMPL_NAMES = {
    "rrtc": "RRTConnect",
    "rrt": "RRT",
    "rrtstar": "RRTstar",
    "informed_rrtstar": "InformedRRTstar",
    "rrtsharp": "RRT#",
    "rrtxstatic": "RRTXstatic",
    "lbtrrt": "LBTRRT",
    "trrt": "TRRT",
    "bitrrt": "BiTRRT",
    "bitstar": "kBITstar",
    "abitstar": "kABITstar",
    "aitstar": "AITstar",
    "eitstar": "EIT*",
    "blitstar": "BLITstar",
    "fmt": "FMT",
    "bfmt": "BFMT",
    "kpiece": "KPIECE1",
    "bkpiece": "BKPIECE1",
    "lbkpiece": "LBKPIECE1",
    "prm": "PRM",
    "prmstar": "PRMstar",
    "lazyprm": "LazyPRM",
    "lazyprmstar": "LazyPRMstar",
    "spars": "SPARS",
    "spars2": "SPARStwo",
    "est": "EST",
    "biest": "BiEST",
    "sbl": "SBL",
    "stride": "STRIDE",
    "pdst": "PDST",
}


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


@pytest.mark.parametrize("name", OMPL_NAMES)
def test_planner_names_build_their_planners(name, capfd):
    planner = create_planner(
        "x500_base",
        config=PlannerConfig(
            planner_name=name, time_limit=0.05, joint_limits=WORKSPACE
        ),
    )
    _, start, goal = LEGS[0]
    assert planner.plan(
        planner.extract_config(start), planner.extract_config(goal)
    ).success
    logged = re.findall(r"^\w+:\s+(\S+?):", capfd.readouterr().out, re.MULTILINE)
    assert OMPL_NAMES[name] in logged


@pytest.mark.parametrize("name", ["strrtstar", "rrt_connect"])
def test_unknown_planner_names_raise(name):
    with pytest.raises(ValueError, match="Unknown planner"):
        PlannerConfig(planner_name=name)
    with pytest.raises(ValueError, match="Unknown planner"):
        body_planner().plan(START, GOAL, name, 0.1, False, False, 0.0)


def test_subgroup_inputs_are_checked():
    home = HOME_JOINTS.tolist()
    with pytest.raises(ValueError):
        OmplVampPlanner([0, 1, 2], home[:5])
    with pytest.raises(ValueError):
        OmplVampPlanner([0, 1, 6], home)
    with pytest.raises(ValueError):
        OmplVampPlanner([-1, 1, 2], home)
    planner = OmplVampPlanner()
    with pytest.raises(ValueError):
        planner.set_subgroup([0, 1, 2], home + [0.0])
    with pytest.raises(ValueError):
        planner.set_subgroup([0, 1, 6], home)


def test_body_subgroup_checks_the_frozen_gimbal():
    """The body subgroup checks the whole X500, the gimbal held at its
    frozen pose: its verdicts equal the full body's on the same poses."""
    frozen = HOME_JOINTS.copy()
    frozen[4:] = 1.0, -1.2  # pan right, tilt down
    cloud = window_scene()
    body = OmplVampPlanner([0, 1, 2, 3], frozen.tolist())
    full = OmplVampPlanner()
    for planner in (body, full):
        planner.add_pointcloud(cloud, 0.0)
    lower, upper = np.array([WORKSPACE[j] for j in WORKSPACE]).T
    rng = np.random.default_rng(0)
    poses = np.column_stack(
        [rng.uniform(lower, upper, (2000, 3)), rng.uniform(-np.pi, np.pi, 2000)]
    )
    expected = full.validate_batch(
        np.column_stack([poses, np.tile(frozen[4:], (2000, 1))])
    )
    assert 0 < expected.sum() < len(expected)
    np.testing.assert_array_equal(body.validate_batch(poses), expected)
