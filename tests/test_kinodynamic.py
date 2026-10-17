"""Kinodynamic (FLASK) planning: closed-form math and end-to-end checks.

Every plan is verified independently of the planner's own validation:
the returned trajectory is densely resampled and checked for collisions
(``validate_batch``), joint ranges, the per-joint velocity / acceleration
ranges and the norm of the horizontal velocity / acceleration, and C^1
continuity.
"""

import numpy as np
import pytest
from _shared import LEGS, WORKSPACE, window_scene
from conftest import assert_collision_free

from drone_planning._ompl_vamp import flat_optimal_time
from drone_planning.planning import create_planner
from drone_planning.types import KinodynamicConfig, PlannerConfig, PlanningStatus
from drone_planning.x500 import (
    HORIZONTAL_MAX_ACCELERATION,
    HORIZONTAL_MAX_SPEED,
    JOINT_ACCELERATION_LIMITS,
    JOINT_VELOCITY_LIMITS,
)

DT = 0.001
TOL = 1e-6
CLOUD = window_scene()


def lqmt_cost(y0, v0, y1, v1, w, rho, T):
    """J(T) of the Hermite cubic, by numerical quadrature."""
    y0, v0, y1, v1, w = map(np.asarray, (y0, v0, y1, v1, w))
    d = y1 - y0
    c2 = (3 * d - T * (2 * v0 + v1)) / T**2
    c3 = (-2 * d + T * (v0 + v1)) / T**3
    t = np.linspace(0.0, T, 20001)[:, None]
    acc = 2 * c2 + 6 * c3 * t
    return float(np.trapezoid((w * acc**2).sum(axis=1), t[:, 0]) + rho * T)


def assert_executable(planner, result, start, goal, start_velocity=None):
    """Independent checks of a returned kinodynamic trajectory."""
    assert result.success, result.status
    traj = result.trajectory
    assert traj.duration > 0.0
    t, q, qd, qdd = traj.sample_uniform(DT)

    # Boundary conditions: exact start state, goal reached at rest.
    np.testing.assert_allclose(q[0], start, atol=1e-9)
    np.testing.assert_allclose(q[-1], goal, atol=1e-6)
    np.testing.assert_allclose(qd[-1], 0.0, atol=1e-6)
    v0 = np.zeros_like(start) if start_velocity is None else start_velocity
    np.testing.assert_allclose(qd[0], v0, atol=1e-6)

    # Collision-free along the executed trajectory (not just at nodes).
    assert_collision_free(planner, q)

    # C^1: positions and velocities continuous across segment knots.
    eps = 1e-7
    for k in traj.knot_times[1:-1]:
        np.testing.assert_allclose(
            traj.position(k - eps), traj.position(k + eps), atol=1e-5
        )
        np.testing.assert_allclose(
            traj.velocity(k - eps), traj.velocity(k + eps), atol=1e-4
        )

    # Joint ranges, per-joint velocity / acceleration ranges.
    lower, upper = planner.bounds
    assert np.all(q >= lower - TOL) and np.all(q <= upper + TOL)
    names = planner.joint_names
    for i, name in enumerate(names):
        vlo, vhi = JOINT_VELOCITY_LIMITS[name]
        alo, ahi = JOINT_ACCELERATION_LIMITS[name]
        assert vlo * (1 + TOL) - TOL <= qd[:, i].min(), name
        assert qd[:, i].max() <= vhi * (1 + TOL) + TOL, name
        assert alo * (1 + TOL) - TOL <= qdd[:, i].min(), name
        assert qdd[:, i].max() <= ahi * (1 + TOL) + TOL, name

    # Horizontal speed and acceleration, bounded in norm.
    x, y = names.index("base_x_joint"), names.index("base_y_joint")
    assert np.hypot(qd[:, x], qd[:, y]).max() <= HORIZONTAL_MAX_SPEED * (1 + TOL)
    assert np.hypot(qdd[:, x], qdd[:, y]).max() <= HORIZONTAL_MAX_ACCELERATION * (
        1 + TOL
    )


# ── Closed-form LQMT ───────────────────────────────────────────────────


def test_optimal_time_rest_to_rest_closed_form():
    # r = 2, rest-to-rest, unit weight: T* = (36 d^2 / rho)^(1/4).
    for d, rho in [(1.0, 1.0), (0.3, 4.0), (2.5, 0.5)]:
        T, J = flat_optimal_time([0.0], [0.0], [d], [0.0], [1.0], rho)
        assert T == pytest.approx((36 * d**2 / rho) ** 0.25, rel=1e-9)
        assert J == pytest.approx(lqmt_cost([0], [0], [d], [0], [1], rho, T), rel=1e-6)


@pytest.mark.parametrize("seed", range(10))
def test_optimal_time_is_global_minimum(seed):
    rng = np.random.default_rng(seed)
    n = 6
    y0, y1 = rng.uniform(-2, 2, n), rng.uniform(-2, 2, n)
    v0, v1 = rng.uniform(-1, 1, n), rng.uniform(-1, 1, n)
    w = rng.uniform(0.2, 3.0, n)
    rho = float(rng.uniform(0.1, 10.0))
    T, J = flat_optimal_time(y0, v0, y1, v1, w, rho)
    grid = np.geomspace(1e-2, 1e2, 4000)
    d = y1 - y0
    a = np.sum(w * 12 * d**2)
    b = np.sum(w * 12 * (v0 + v1) * d)
    c = np.sum(w * 4 * (v0**2 + v0 * v1 + v1**2))
    costs = a / grid**3 - b / grid**2 + c / grid + rho * grid
    assert J <= costs.min() + 1e-9
    # Stationarity at T*.
    dJ = -3 * a / T**4 + 2 * b / T**3 - c / T**2 + rho
    assert abs(dJ) < 1e-6 * max(1.0, rho)
    assert J == pytest.approx(lqmt_cost(y0, v0, y1, v1, w, rho, T), rel=1e-5)


# ── The demo scene ─────────────────────────────────────────────────────


@pytest.fixture(scope="module", params=["x500_base", "x500_whole_body"])
def planner(request):
    return create_planner(
        request.param,
        config=PlannerConfig(joint_limits=WORKSPACE),
        pointcloud=CLOUD,
    )


@pytest.mark.parametrize("leg", range(len(LEGS)), ids=[leg[0] for leg in LEGS])
def test_plan_is_executable(planner, leg):
    _, start, goal = LEGS[leg]
    start, goal = planner.extract_config(start), planner.extract_config(goal)
    cfg = KinodynamicConfig(time_limit=2.0, seed=leg + 1)
    result = planner.plan_kinodynamic(start, goal, config=cfg)
    assert_executable(planner, result, start, goal)


def test_plan_is_deterministic_with_seed(planner):
    _, start, goal = LEGS[0]
    start, goal = planner.extract_config(start), planner.extract_config(goal)
    cfg = KinodynamicConfig(time_limit=2.0, seed=3)
    a = planner.plan_kinodynamic(start, goal, config=cfg)
    b = planner.plan_kinodynamic(start, goal, config=cfg)
    assert a.success and b.success
    np.testing.assert_allclose(a.trajectory.knot_times, b.trajectory.knot_times)


def test_replans_from_moving_state(planner):
    _, start, goal = LEGS[0]
    start, goal = planner.extract_config(start), planner.extract_config(goal)
    cfg = KinodynamicConfig(time_limit=2.0, seed=5)
    first = planner.plan_kinodynamic(start, goal, config=cfg)
    assert first.success
    t_mid = 0.4 * first.trajectory.duration
    q_mid = first.trajectory.position(t_mid)
    qd_mid = first.trajectory.velocity(t_mid)
    assert np.abs(qd_mid).max() > 1e-3, "replanning test needs a moving state"
    second = planner.plan_kinodynamic(q_mid, start, start_velocity=qd_mid, config=cfg)
    assert_executable(planner, second, q_mid, start, start_velocity=qd_mid)


def test_scaled_limits_slow_down(planner):
    _, start, goal = LEGS[0]
    start, goal = planner.extract_config(start), planner.extract_config(goal)
    fast = planner.plan_kinodynamic(start, goal, config=KinodynamicConfig(seed=7))
    slow_cfg = KinodynamicConfig(seed=7, velocity_scale=0.25, acceleration_scale=0.25)
    slow = planner.plan_kinodynamic(start, goal, config=slow_cfg)
    assert fast.success and slow.success
    assert slow.trajectory.duration > fast.trajectory.duration
    _, _, qd, _ = slow.trajectory.sample_uniform(DT)
    names = planner.joint_names
    x, y = names.index("base_x_joint"), names.index("base_y_joint")
    assert np.hypot(qd[:, x], qd[:, y]).max() <= 0.25 * HORIZONTAL_MAX_SPEED * (1 + TOL)


def test_invalid_start_is_reported(planner):
    _, start, goal = LEGS[0]
    inside_wall = planner.extract_config(start).copy()
    inside_wall[0] = 2.0
    result = planner.plan_kinodynamic(inside_wall, planner.extract_config(goal))
    assert result.status == PlanningStatus.INVALID_START
    assert result.trajectory is None and not result.success
