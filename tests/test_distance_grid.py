"""The distance-grid (ESDF) input of ``OmplVampPlanner``, for both robots.

A sphere of the robot is free where the trilinear distance at its centre
is at least its radius and the sphere lies wholly inside the lattice's
box.  The planner reads the float32 array in place, without a copy.
"""

import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import pytest
from conftest import lattice

from drone_planning import drone, x500
from drone_planning._ompl_vamp import OmplVampPlanner
from drone_planning.planning import create_planner

ROBOTS = {"x500": x500, "drone": drone}


def sphere_model(urdf: str, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """World centres ``(n, 3)`` and radii ``(n,)`` of the collision spheres
    of a spherized URDF at configuration ``q``."""
    model = pin.buildModelFromUrdf(urdf)
    data = model.createData()
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    centers, radii = [], []
    for link in ET.parse(urdf).getroot().findall("link"):
        for c in link.findall("collision"):
            M = data.oMf[model.getFrameId(link.get("name"))]
            centers.append(
                M.rotation @ np.array(c.find("origin").get("xyz").split(), float)
                + M.translation
            )
            radii.append(float(c.find("geometry/sphere").get("radius")))
    return np.array(centers), np.array(radii)


@pytest.mark.parametrize("robot", ROBOTS)
def test_distance_grid_agrees_with_sphere_obstacles(robot):
    """Away from the surface, a grid of the distance to three spheres
    accepts and rejects what the spheres themselves do.

    The obstacle spheres lie far apart, so trilinear interpolation of
    their distance only overestimates it, by at most 3 h^2 / (8 d) in a
    cell at distance d or more from a centre: about 1 mm here.
    Configurations clear of the spheres by ``tol`` are free in the grid;
    those that overlap them by more than ``tol`` collide.
    """
    centers = np.array([[0.6, 0.0, 1.2], [-0.5, 0.6, 1.0], [0.0, -0.7, 1.4]])
    radius, spacing, tol = 0.2, 0.025, 2e-3
    points, origin = lattice([-2.0, -2.0, 0.0], [2.0, 2.0, 2.5], spacing)
    distances = np.min([np.linalg.norm(points - c, axis=-1) for c in centers], 0)
    distances -= radius
    grid = OmplVampPlanner(robot)
    grid.set_distance_grid(distances.astype(np.float32), origin, [spacing] * 3)
    inflated, deflated = OmplVampPlanner(robot), OmplVampPlanner(robot)
    for center in centers:
        inflated.add_sphere(center.tolist(), radius + tol)
        deflated.add_sphere(center.tolist(), radius - tol)

    rng = np.random.default_rng(0)
    lower, upper = grid.default_joint_limits()
    q = rng.uniform(lower, upper, (4000, 6))
    q[:, :3] = rng.uniform([-1.0, -1.0, 0.7], [1.0, 1.0, 1.7], (4000, 3))
    valid, free, colliding = (
        grid.validate_batch(q),
        inflated.validate_batch(q),
        ~deflated.validate_batch(q),
    )
    assert valid[free].all()
    assert not valid[colliding].any()
    assert 0.2 < free.mean() < 0.8


@pytest.mark.parametrize("robot", ROBOTS)
def test_distance_grid_box_is_the_workspace(robot):
    """Every sphere must lie wholly inside the lattice's box: the body
    collides 1 cm past each face, and is free 1 cm before it."""
    module = ROBOTS[robot]
    spacing = 0.05
    lower, upper = np.array([-1.0, -1.5, 0.0]), np.array([1.0, 1.5, 2.5])
    points, origin = lattice(lower, upper, spacing)
    planner = OmplVampPlanner(robot)
    planner.set_distance_grid(
        np.full(points.shape[:3], 10.0, dtype=np.float32), origin, [spacing] * 3
    )
    q = module.HOME_JOINTS.copy()
    q[:3] = 0.0
    q[3:] = [0.4, 0.3, -0.5]  # yaw, pan, tilt: no symmetry to rely on
    centers, radii = sphere_model(module.SPHERIZED_URDF_PATH, q)
    low_reach = (centers - radii[:, None]).min(axis=0)  # body extent from q[:3]
    high_reach = (centers + radii[:, None]).max(axis=0)
    centre = (lower + upper) / 2 - (low_reach + high_reach) / 2
    assert planner.validate(np.concatenate([centre, q[3:]]))
    for axis in range(3):
        for gap, expected in ((0.01, True), (-0.01, False)):
            high = centre.copy()
            high[axis] = upper[axis] - high_reach[axis] - gap
            low = centre.copy()
            low[axis] = lower[axis] - low_reach[axis] + gap
            for body in (high, low):
                assert planner.validate(np.concatenate([body, q[3:]])) == expected


@pytest.mark.parametrize("robot", ROBOTS)
def test_distance_grid_replace_and_remove(robot):
    home = ROBOTS[robot].HOME_JOINTS
    spacing = 0.05
    points, origin = lattice([-2.0, -2.0, 0.0], [2.0, 2.0, 2.5], spacing)
    # A floor up to 1.5 m: the hovering robot is inside it.
    floor = (points[..., 2] - 1.5).astype(np.float32)
    free = np.full(points.shape[:3], 10.0, dtype=np.float32)
    planner = OmplVampPlanner(robot)
    assert not planner.has_distance_grid()
    assert planner.validate(home)
    planner.set_distance_grid(floor, origin, [spacing] * 3)
    assert planner.has_distance_grid() and not planner.validate(home)
    # A second grid replaces the first.
    planner.set_distance_grid(free, origin, [spacing] * 3)
    assert planner.validate(home)
    planner.set_distance_grid(floor, origin, [spacing] * 3)
    # A rejected grid leaves the grid in effect.
    with pytest.raises(ValueError):
        planner.set_distance_grid(free, origin, [spacing, 0.0, spacing])
    assert planner.has_distance_grid() and not planner.validate(home)
    assert planner.remove_distance_grid()
    assert not planner.has_distance_grid() and planner.validate(home)
    assert not planner.remove_distance_grid()
    planner.set_distance_grid(floor, origin, [spacing] * 3)
    planner.clear_environment()
    assert not planner.has_distance_grid() and planner.validate(home)


@pytest.mark.parametrize("robot", ROBOTS)
def test_distance_grid_inputs_are_checked(robot):
    planner = OmplVampPlanner(robot)
    field = np.zeros((4, 5, 6), dtype=np.float32)
    origin, spacing = [0.0, 0.0, 0.0], [0.1, 0.1, 0.1]
    # Wrong dtype, order, layout or number of axes: not converted (a
    # converted array would be a copy, not read in place).
    for bad in (
        field.astype(np.float64),
        np.asfortranarray(field),
        np.zeros((4, 5, 12), dtype=np.float32)[:, :, ::2],
        field[0],
        field[None],
    ):
        with pytest.raises(TypeError):
            planner.set_distance_grid(bad, origin, spacing)
    # Fewer than 2 lattice points on an axis, a bad spacing or origin.
    for bad_field, bad_origin, bad_spacing in (
        (field[:1].copy(), origin, spacing),
        (field[:, :, :1].copy(), origin, spacing),
        (field, origin, [0.1, 0.0, 0.1]),
        (field, origin, [0.1, -0.1, 0.1]),
        (field, [0.0, float("nan"), 0.0], spacing),
        (field, origin, [float("inf"), 0.1, 0.1]),
    ):
        with pytest.raises(ValueError):
            planner.set_distance_grid(bad_field, bad_origin, bad_spacing)
    with pytest.raises(TypeError):
        planner.set_distance_grid(field, [0.0, 0.0], spacing)
    assert not planner.has_distance_grid()
    planner.set_distance_grid(field, origin, spacing)
    assert planner.has_distance_grid()


@pytest.mark.parametrize("robot", ROBOTS)
def test_distance_grid_reads_the_array_in_place(robot):
    """The planner reads the caller's array without a copy and keeps it alive."""
    home = ROBOTS[robot].HOME_JOINTS
    spacing = 0.05
    points, origin = lattice([-2.0, -2.0, 0.0], [2.0, 2.0, 2.5], spacing)
    field = np.full(points.shape[:3], 10.0, dtype=np.float32)
    planner = OmplVampPlanner(robot)
    planner.set_distance_grid(field, origin, [spacing] * 3)
    assert planner.validate(home)
    field[...] = -1.0
    assert not planner.validate(home)
    del field
    assert not planner.validate(home)
    # A read-only array is accepted too.
    readonly = np.full(points.shape[:3], 10.0, dtype=np.float32)
    readonly.setflags(write=False)
    planner.set_distance_grid(readonly, origin, [spacing] * 3)
    assert planner.validate(home)


@pytest.mark.parametrize("robot", ROBOTS)
def test_motion_planner_distance_grid(robot):
    """``MotionPlanner.set_distance_grid`` converts its inputs (here float64
    and one spacing for all axes) and drives the same check."""
    planner = create_planner(f"{robot}_base")
    spacing = 0.05
    points, origin = lattice([-2.0, -2.0, 0.0], [2.0, 2.0, 2.5], spacing)
    floor = points[..., 2] - 1.5
    body = planner.extract_config(ROBOTS[robot].HOME_JOINTS)
    planner.set_distance_grid(floor, np.array(origin), spacing)
    assert planner.has_distance_grid and not planner.validate(body)
    assert planner.remove_distance_grid()
    assert not planner.has_distance_grid and planner.validate(body)
