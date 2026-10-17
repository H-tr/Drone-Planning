import numpy as np
import pytest

from drone_planning._ompl_vamp import OmplVampPlanner
from drone_planning.planning import create_planner
from drone_planning.types import PlannerConfig
from drone_planning.x500 import HOME_JOINTS, PLANNING_SUBGROUPS

SUBGROUPS = ["x500", *PLANNING_SUBGROUPS]


@pytest.mark.parametrize("subgroup", SUBGROUPS)
def test_planner_constructs(subgroup):
    planner = create_planner(
        subgroup,
        config=PlannerConfig(planner_name="rrtc", time_limit=0.1),
    )
    assert planner.num_dof == len(planner.joint_names)
    assert planner.validate(planner.extract_config(HOME_JOINTS))


def test_joint_limits_survive_subgroup_switch():
    planner = create_planner(
        "x500", config=PlannerConfig(joint_limits={"base_z_joint": (0.5, 2.0)})
    )
    planner.set_subgroup("x500_base")
    lower, upper = planner.bounds
    assert (lower[2], upper[2]) == (0.5, 2.0)
    assert planner.joint_limits["base_z_joint"] == (0.5, 2.0)


def test_low_level_api():
    """The extension calls a caller like PHANTOM makes: full-body planner,
    joint limits, subgroup over the body, point cloud as float32 (N, 3)."""
    planner = OmplVampPlanner()
    assert planner.dimension() == 6
    lower, upper = planner.default_joint_limits()
    lower[:3], upper[:3] = [-1.0, -1.0, 0.2], [3.0, 1.0, 2.0]
    planner.set_joint_limits(lower, upper)
    planner.set_subgroup([0, 1, 2, 3], HOME_JOINTS.tolist())
    assert planner.dimension() == 4
    # A column of points the straight line passes through.
    z = np.linspace(0.0, 2.0, 101)
    column = np.stack([np.full_like(z, 1.0), np.zeros_like(z), z], 1)
    planner.add_pointcloud(column.astype(np.float32), 0.0)
    start, goal = np.array([0.0, 0.0, 1.0, 0.0]), np.array([2.0, 0.0, 1.0, 0.0])
    result = planner.plan(
        start,
        goal,
        planner_name="rrtc",
        time_limit=1.0,
        simplify=True,
        interpolate=False,
    )
    assert result.solved
    assert isinstance(result.path, np.ndarray) and result.path.shape[1] == 4
    np.testing.assert_array_equal(result.path[0], start)
    np.testing.assert_array_equal(result.path[-1], goal)
    assert np.all(result.path >= planner.lower_bounds())
    assert np.all(result.path <= planner.upper_bounds())


def test_point_cloud_replaces_previous():
    planner = OmplVampPlanner()
    q = HOME_JOINTS.copy()
    body = np.array([q[:3], q[:3] + [0.0, 0.0, 0.01]], dtype=np.float32)
    planner.add_pointcloud(body, 0.0)
    assert not planner.validate(q)
    planner.add_pointcloud(body + 5.0, 0.0)
    assert planner.validate(q)
