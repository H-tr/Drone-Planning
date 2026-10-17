"""The compiled VAMP model covers the X500's meshes.

Surface points are sampled on every visual mesh of ``x500.urdf`` (the
propeller blades at random spin angles, so the whole rotor disc), placed
in the world by forward kinematics, and handed to the planner's
``filter_self_from_pointcloud``, which keeps only the points no robot
sphere touches.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pinocchio as pin
import pytest
from _shared import window_scene

from drone_planning._ompl_vamp import OmplVampPlanner
from drone_planning.x500 import HOME_JOINTS, SPHERIZED_URDF_PATH, x500_robot_config

URDF = Path(x500_robot_config.urdf_path)
SAMPLES_PER_MESH = 20000


def stl_triangles(path: Path) -> np.ndarray:
    """``(n, 3, 3)`` triangle vertices of a binary STL."""
    data = np.fromfile(path, dtype=np.uint8)
    n = int(np.frombuffer(data[80:84], np.uint32)[0])
    record = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    return np.frombuffer(data[84 : 84 + 50 * n], dtype=record)["v"].astype(np.float64)


def sample_surface(triangles: np.ndarray, n: int, rng) -> np.ndarray:
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    i = rng.choice(len(triangles), n, p=area / area.sum())
    u, v = rng.random((2, n))
    flip = u + v > 1.0
    u[flip], v[flip] = 1.0 - u[flip], 1.0 - v[flip]
    return a[i] + u[:, None] * (b[i] - a[i]) + v[:, None] * (c[i] - a[i])


def link_samples(rng) -> dict[str, np.ndarray]:
    """Surface samples of every link's visual meshes, in the link frame."""
    out = {}
    for link in ET.parse(URDF).getroot().findall("link"):
        points = []
        for visual in link.findall("visual"):
            filename = visual.find("geometry/mesh").get("filename")
            mesh = URDF.parent / filename.removeprefix("package://")
            p = sample_surface(stl_triangles(mesh), SAMPLES_PER_MESH, rng)
            origin = visual.find("origin")
            R = pin.rpy.rpyToMatrix(*map(float, origin.get("rpy").split()))
            p = p @ R.T + np.array(origin.get("xyz").split(), float)
            if "prop" in mesh.name:  # spinning about the rotor axis
                spin = rng.uniform(0.0, 2.0 * np.pi, len(p))
                c, s = np.cos(spin), np.sin(spin)
                p = np.stack(
                    [c * p[:, 0] - s * p[:, 1], s * p[:, 0] + c * p[:, 1], p[:, 2]], 1
                )
            points.append(p)
        if points:
            out[link.get("name")] = np.concatenate(points)
    return out


CONFIGS = [HOME_JOINTS] + [
    np.array([0.5, -0.3, 1.5, yaw, pan, tilt])
    for yaw, pan, tilt in [(0.7, 1.5, -2.0), (-2.5, -3.0, 0.7), (3.0, 0.3, -0.8)]
]


@pytest.mark.parametrize("q", CONFIGS, ids=[f"q{i}" for i in range(len(CONFIGS))])
def test_spheres_cover_meshes(q):
    rng = np.random.default_rng(0)
    model = pin.buildModelFromUrdf(str(URDF))
    data = model.createData()
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    planner = OmplVampPlanner()
    for link, points in link_samples(rng).items():
        M = data.oMf[model.getFrameId(link)]
        world = points @ M.rotation.T + M.translation
        uncovered = planner.filter_self_from_pointcloud(
            world.astype(np.float32), 0.0, q
        )
        assert len(uncovered) <= 1e-3 * len(world), link


def test_validate_matches_brute_force():
    """VAMP's verdict equals a brute-force check of every sphere of
    ``x500_spherized.urdf`` against every point, for configurations around
    the demo scene's window (VAMP computes in float32: near-ties skipped)."""
    cloud = window_scene()
    planner = OmplVampPlanner()
    planner.add_pointcloud(cloud, 0.0)

    model = pin.buildModelFromUrdf(SPHERIZED_URDF_PATH)
    data = model.createData()
    spheres = [
        (
            model.getFrameId(link.get("name")),
            np.array(c.find("origin").get("xyz").split(), float),
            float(c.find("geometry/sphere").get("radius")),
        )
        for link in ET.parse(SPHERIZED_URDF_PATH).getroot().findall("link")
        for c in link.findall("collision")
    ]
    frames = sorted({f for f, _, _ in spheres})
    radii = np.array([r for _, _, r in spheres])

    rng = np.random.default_rng(0)
    lower, upper = OmplVampPlanner.default_joint_limits()
    configs = rng.uniform(lower, upper, (1000, 6))
    configs[:, :3] = rng.uniform([1.5, -0.9, 0.8], [2.5, 0.9, 2.2], (1000, 3))
    valid = planner.validate_batch(configs)
    for q, vamp_valid in zip(configs, valid):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        M = {f: data.oMf[f] for f in frames}
        centers = np.array(
            [M[f].rotation @ p + M[f].translation for f, p, _ in spheres]
        )
        near = cloud[np.linalg.norm(cloud - q[:3], axis=1) < 0.7]
        gap = np.linalg.norm(near[None] - centers[:, None], axis=2) - radii[:, None]
        clearance = gap.min() if gap.size else np.inf
        if abs(clearance) > 1e-5:
            assert vamp_valid == (clearance > 0), (q, clearance)
