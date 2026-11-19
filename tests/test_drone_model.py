"""PHANTOM's drone: the model, its collision spheres and planning with it.

The drone has 6 DOF in PHANTOM's order (base x, y, z, yaw, camera pan,
camera tilt) and 10 collision spheres on the body: foam's fit of the body
box and one sphere at the gimbal pivot that holds the camera.
"""

import math
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import pytest
from conftest import assert_collision_free, box_distance, lattice

from drone_planning._ompl_vamp import OmplVampPlanner
from drone_planning.drone import (
    HOME_JOINTS,
    JOINT_ACCELERATION_LIMITS,
    JOINT_GROUPS,
    JOINT_VELOCITY_LIMITS,
    PLANNING_SUBGROUPS,
    SPHERIZED_URDF_PATH,
    drone_robot_config,
)
from drone_planning.kinematics import (
    compute_forward_kinematics,
    create_pinocchio_context,
)
from drone_planning.planning import available_robots, create_planner
from drone_planning.types import KinodynamicConfig, PlannerConfig

BOX_HALF = np.array([0.23, 0.23, 0.06])  # the body box, 0.46 x 0.46 x 0.12 m
PIVOT = np.array([0.0, 0.0, -0.06])  # the gimbal pivot in the body frame
CAMERA_BOX = (np.array([0.0, -0.03, -0.02]), np.array([0.08, 0.03, 0.02]))
JOINT_NAMES = [
    "base_x_joint",
    "base_y_joint",
    "base_z_joint",
    "base_yaw_joint",
    "camera_pan",
    "camera_tilt",
]
LOWER = [-10.0, -10.0, 0.0, -math.pi, -math.pi / 2, -math.pi / 2]
UPPER = [10.0, 10.0, 10.0, math.pi, math.pi / 2, math.pi / 6]


def body_spheres() -> np.ndarray:
    """``(10, 4)`` centres (body frame) and radii of drone_spherized.urdf."""
    root = ET.parse(SPHERIZED_URDF_PATH).getroot()
    spheres = []
    for link in root.findall("link"):
        for collision in link.findall("collision"):
            assert link.get("name") == "base_link"
            xyz = [float(v) for v in collision.find("origin").get("xyz").split()]
            spheres.append(
                xyz + [float(collision.find("geometry/sphere").get("radius"))]
            )
    return np.array(spheres)


def yaw_matrix(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def gimbal_matrix(pan: float, tilt: float) -> np.ndarray:
    """Camera frame in the body frame: pan about z, then tilt about -y."""
    c, s = math.cos(-tilt), math.sin(-tilt)
    return yaw_matrix(pan) @ np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def world_spheres(q) -> np.ndarray:
    spheres = body_spheres()
    spheres[:, :3] = spheres[:, :3] @ yaw_matrix(q[3]).T + q[:3]
    return spheres


def outside(points, spheres) -> np.ndarray:
    """Distance of each point outside the nearest sphere (negative inside)."""
    gaps = np.linalg.norm(points[:, None] - spheres[None, :, :3], axis=-1)
    return (gaps - spheres[None, :, 3]).min(axis=1)


def box_lattice(lower, upper, n=21) -> np.ndarray:
    axes = [np.linspace(lo, hi, n) for lo, hi in zip(lower, upper)]
    return np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)


def rows(points: np.ndarray) -> np.ndarray:
    """``points`` in lexicographic row order."""
    return points[np.lexsort(points.T[::-1])]


def camera_points(pan: float, tilt: float) -> np.ndarray:
    """The camera box's lattice in the body frame at the gimbal angles."""
    return PIVOT + box_lattice(*CAMERA_BOX, n=7) @ gimbal_matrix(pan, tilt).T


def wall_grid(x_wall: float, spacing: float = 0.05):
    """A distance grid of the half-space ``x >= x_wall`` (exact: the
    distance is linear, so trilinear interpolation is exact)."""
    points, origin = lattice([-2.0, -2.0, 0.0], [2.0, 2.0, 2.5], spacing)
    return (x_wall - points[..., 0]).astype(np.float32), origin, [spacing] * 3


def test_dimension_and_joint_limits():
    planner = OmplVampPlanner("drone")
    assert planner.dimension() == 6
    for lower, upper in (
        planner.default_joint_limits(),
        OmplVampPlanner.default_joint_limits("drone"),
        planner.joint_limits(),
    ):
        np.testing.assert_allclose(lower, LOWER, atol=1e-6)
        np.testing.assert_allclose(upper, UPPER, atol=1e-6)


def test_compiled_model_matches_urdf():
    """The VAMP model (cricket) has the URDF's joints, order and ranges."""
    assert drone_robot_config.joint_names == JOINT_NAMES
    lower, upper = OmplVampPlanner("drone").default_joint_limits()
    for urdf in (drone_robot_config.urdf_path, SPHERIZED_URDF_PATH):
        model = pin.buildModelFromUrdf(urdf)
        names = [model.names[i] for i in range(1, model.njoints)]
        assert names == JOINT_NAMES
        np.testing.assert_allclose(lower, model.lowerPositionLimit, atol=1e-6)
        np.testing.assert_allclose(upper, model.upperPositionLimit, atol=1e-6)


def test_python_description():
    assert HOME_JOINTS.shape == (6,)
    covered = set()
    for group in JOINT_GROUPS.values():
        covered.update(range(*group.indices(6)))
    assert covered == set(range(6))
    assert set(PLANNING_SUBGROUPS) == {"drone_base", "drone_whole_body"}
    assert PLANNING_SUBGROUPS["drone_whole_body"]["joints"] == JOINT_NAMES
    assert PLANNING_SUBGROUPS["drone_base"]["joints"] == JOINT_NAMES[:4]
    assert {"drone", "drone_base", "drone_whole_body"} <= set(available_robots())
    # PHANTOM's actuator limits, symmetric.
    expected = {
        "base_x_joint": (1.5, 1.0),
        "base_y_joint": (1.5, 1.0),
        "base_z_joint": (1.5, 1.0),
        "base_yaw_joint": (1.5, 3.0),
        "camera_pan": (1.0, 2.0),
        "camera_tilt": (1.0, 2.0),
    }
    assert list(JOINT_VELOCITY_LIMITS) == JOINT_NAMES
    assert list(JOINT_ACCELERATION_LIMITS) == JOINT_NAMES
    for name, (velocity, acceleration) in expected.items():
        assert JOINT_VELOCITY_LIMITS[name] == (-velocity, velocity)
        assert JOINT_ACCELERATION_LIMITS[name] == (-acceleration, acceleration)


def test_spheres_cover_the_body_and_the_camera():
    """Every point of the body box, and of the camera box at every pan and
    tilt, lies inside a sphere; every sphere is on base_link."""
    spheres = body_spheres()
    assert len(spheres) == 10
    assert outside(box_lattice(-BOX_HALF, BOX_HALF, 41), spheres).max() < 0.0
    for pan in np.linspace(-math.pi / 2, math.pi / 2, 13):
        for tilt in np.linspace(-math.pi / 2, math.pi / 6, 13):
            assert outside(camera_points(pan, tilt), spheres).max() < 0.0


@pytest.mark.parametrize("yaw", [0.0, 0.4, math.pi / 4, 2.0, -2.7])
def test_compiled_spheres_turn_with_the_yaw(yaw):
    """VAMP's compiled model has the spheres of drone_spherized.urdf, turned
    by the yaw: the body box and the camera touch the robot, and a point
    touches it exactly when it is inside a sphere (1 mm either side)."""
    planner = OmplVampPlanner("drone")
    rng = np.random.default_rng(int(1000 * abs(yaw)))
    for pan, tilt in ((0.0, 0.0), (1.2, -math.pi / 2), (-0.7, 0.4)):
        q = np.array([0.5, -0.3, 1.5, yaw, pan, tilt])
        body = np.vstack([box_lattice(-BOX_HALF, BOX_HALF), camera_points(pan, tilt)])
        points = (body @ yaw_matrix(yaw).T + q[:3]).astype(np.float32)
        assert len(planner.filter_self_from_pointcloud(points, 0.0, q)) == 0
        points = q[:3] + rng.uniform(-0.5, 0.5, (20000, 3))
        gap = outside(points, world_spheres(q))
        clear = np.abs(gap) > 1e-3
        points, inside = points[clear].astype(np.float32), gap[clear] < 0
        kept = planner.filter_self_from_pointcloud(points, 0.0, q)
        assert 0 < inside.sum() < len(points)
        np.testing.assert_array_equal(rows(np.asarray(kept)), rows(points[~inside]))


@pytest.mark.parametrize("scene", ["distance_grid", "point_cloud"])
@pytest.mark.parametrize("yaw", [0.0, math.pi / 4, 1.0])
def test_clearance_to_a_wall(scene, yaw):
    """The drone is free 1 cm short of touching a wall at x = 1 and collides
    1 cm past it; the reach toward the wall is that of the spheres turned
    by the yaw."""
    planner = OmplVampPlanner("drone")
    x_wall = 1.0
    if scene == "distance_grid":
        planner.set_distance_grid(*wall_grid(x_wall))
    else:
        y, z = np.meshgrid(np.arange(-1.0, 1.0, 0.01), np.arange(0.0, 2.5, 0.01))
        wall = np.stack([np.full(y.size, x_wall), y.ravel(), z.ravel()], 1)
        planner.add_pointcloud(wall.astype(np.float32), 0.0)
    q = HOME_JOINTS.copy()
    q[3] = yaw
    reach = (world_spheres(q)[:, 0] + body_spheres()[:, 3]).max() - q[0]
    box = (box_lattice(-BOX_HALF, BOX_HALF, 2) @ yaw_matrix(yaw).T)[:, 0].max()
    assert (
        box < reach < box + 0.08
    )  # the box's reach toward the wall, and the spheres' excess
    for clearance, valid in ((reach + 0.01, True), (reach - 0.01, False)):
        q[0] = x_wall - clearance
        assert planner.validate(q) == valid


def test_gimbal_angles_do_not_change_validity():
    planner = OmplVampPlanner("drone")
    planner.set_distance_grid(*wall_grid(0.5))
    rng = np.random.default_rng(1)
    lower, upper = planner.default_joint_limits()
    q = rng.uniform(lower, upper, (3000, 6))
    q[:, :3] = rng.uniform([-1.0, -1.0, 0.5], [1.5, 1.0, 2.0], (3000, 3))
    level = q.copy()
    level[:, 4:] = 0.0
    valid = planner.validate_batch(q)
    assert 0 < valid.sum() < len(valid)
    np.testing.assert_array_equal(valid, planner.validate_batch(level))


def test_camera_fk():
    """The camera hangs 6 cm below the body centre and looks along the
    gimbal angles: positive pan turns it left, positive tilt raises it."""
    ctx = create_pinocchio_context(
        drone_robot_config.urdf_path,
        drone_robot_config.camera.link_name,
        drone_robot_config.joint_names,
    )
    home = compute_forward_kinematics(ctx, HOME_JOINTS)
    np.testing.assert_allclose(home.position, HOME_JOINTS[:3] - [0, 0, 0.06])
    np.testing.assert_allclose(home.rotation[:, 0], [1.0, 0.0, 0.0], atol=1e-9)
    q = HOME_JOINTS.copy()
    q[3], q[4], q[5] = 0.3, 0.5, 0.4  # yaw, pan, tilt
    axis = compute_forward_kinematics(ctx, q).rotation[:, 0]
    azimuth, elevation = q[3] + q[4], q[5]
    np.testing.assert_allclose(
        axis,
        [
            np.cos(azimuth) * np.cos(elevation),
            np.sin(azimuth) * np.cos(elevation),
            np.sin(elevation),
        ],
        atol=1e-9,
    )


def gap_wall_distance(points):
    """Distance (m) to a 0.2 m wall at x = 0 with a 1 m x 1 m gap at
    y in [0.5, 1.5], z in [0.7, 1.7]: the union of four boxes around it."""
    big = 10.0
    boxes = [
        ([-0.1, -big, -big], [0.1, 0.5, big]),  # beside the gap, -y side
        ([-0.1, 1.5, -big], [0.1, big, big]),  # beside the gap, +y side
        ([-0.1, 0.5, -big], [0.1, 1.5, 0.7]),  # below the gap
        ([-0.1, 0.5, 1.7], [0.1, 1.5, big]),  # above the gap
    ]
    return np.min([box_distance(points, lo, hi) for lo, hi in boxes], axis=0)


def gap_wall_grid(spacing=0.05):
    """The distance grid of ``gap_wall_distance`` across the lattice."""
    points, origin = lattice([-2.0, -2.0, 0.0], [2.0, 2.0, 2.5], spacing)
    return gap_wall_distance(points).astype(np.float32), origin, [spacing] * 3


def test_rrtc_through_a_gap_in_a_wall():
    """RRT-Connect on the body subgroup, the gimbal frozen, passes the
    wall's gap; every vertex is valid and the path ends exactly at start
    and goal."""
    planner = OmplVampPlanner("drone", [0, 1, 2, 3], HOME_JOINTS.tolist())
    planner.set_distance_grid(*gap_wall_grid())
    lower, upper = planner.default_joint_limits()
    lower[:3], upper[:3] = [-1.6, -1.6, 0.4], [1.6, 1.6, 2.0]  # inside the lattice
    planner.set_joint_limits(lower, upper)
    start, goal = np.array([-1.0, 0.0, 1.2, 0.0]), np.array([1.0, 0.0, 1.2, 0.5])
    line = start + np.linspace(0.0, 1.0, 200)[:, None] * (goal - start)
    assert not planner.validate_batch(line).all()
    for _ in range(5):
        result = planner.plan(
            start,
            goal,
            planner_name="rrtc",
            time_limit=2.0,
            simplify=True,
            interpolate=False,
        )
        assert result.solved
        path = result.path
        assert path.shape[1] == 4
        np.testing.assert_array_equal(path[0], start)
        np.testing.assert_array_equal(path[-1], goal)
        assert all(planner.validate(vertex) for vertex in path)
        assert_collision_free(
            planner, planner.interpolate_path(path, resolution=1000.0)
        )
        # The body centre crosses x = 0 inside the gap, and there every
        # sphere is clear of the wall (within 1 cm, VAMP's check step).
        a, b = path[:-1], path[1:]
        cross = np.flatnonzero(np.sign(a[:, 0]) != np.sign(b[:, 0]))
        assert cross.size
        for i in cross:
            q = a[i] + a[i, 0] / (a[i, 0] - b[i, 0]) * (b[i] - a[i])
            assert 0.5 < q[1] < 1.5 and 0.7 < q[2] < 1.7
            spheres = world_spheres(q)
            assert np.all(gap_wall_distance(spheres[:, :3]) > spheres[:, 3] - 0.01)


@pytest.mark.parametrize("subgroup", ["drone", *PLANNING_SUBGROUPS])
def test_motion_planner_subgroups(subgroup):
    planner = create_planner(
        subgroup,
        config=PlannerConfig(time_limit=1.0, joint_limits={"camera_pan": (-1.0, 1.0)}),
    )
    assert planner.num_dof == PLANNING_SUBGROUPS.get(subgroup, {"dof": 6})["dof"]
    assert planner.joint_names == JOINT_NAMES[: planner.num_dof]
    assert planner.joint_limits["camera_pan"] == (-1.0, 1.0)
    assert planner.joint_limits["camera_tilt"] == pytest.approx(
        (-math.pi / 2, math.pi / 6)
    )
    assert planner.validate(planner.extract_config(HOME_JOINTS))
    with pytest.raises(ValueError):
        planner.set_subgroup("x500_base")
    with pytest.raises(ValueError):
        planner.set_joint_limits({"cgo3_camera_joint": (-1.0, 1.0)})


def test_kinodynamic_respects_phantom_limits():
    """FLASK on the drone's body keeps PHANTOM's velocity and acceleration
    limits (per axis, and the per-axis value on the horizontal norm)."""
    planner = create_planner(
        "drone_base",
        config=PlannerConfig(
            joint_limits={
                "base_x_joint": (-3.0, 3.0),
                "base_y_joint": (-3.0, 3.0),
                "base_z_joint": (0.5, 2.5),
            }
        ),
    )
    start, goal = np.array([-2.0, -1.0, 1.0, 0.0]), np.array([2.0, 1.5, 2.0, 1.0])
    result = planner.plan_kinodynamic(
        start, goal, config=KinodynamicConfig(time_limit=1.0, seed=1)
    )
    assert result.success, result.status
    _, q, qd, qdd = result.trajectory.sample_uniform(0.001)
    np.testing.assert_allclose(q[0], start, atol=1e-9)
    np.testing.assert_allclose(q[-1], goal, atol=1e-6)
    names = planner.joint_names
    v_max = np.array([JOINT_VELOCITY_LIMITS[j][1] for j in names])
    a_max = np.array([JOINT_ACCELERATION_LIMITS[j][1] for j in names])
    assert np.all(np.abs(qd) <= v_max * (1 + 1e-6))
    assert np.all(np.abs(qdd) <= a_max * (1 + 1e-6))
    assert np.all(np.linalg.norm(qd[:, :2], axis=1) <= 1.5 * (1 + 1e-6))
    assert np.all(np.linalg.norm(qdd[:, :2], axis=1) <= 1.0 * (1 + 1e-6))
