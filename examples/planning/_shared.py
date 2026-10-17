"""Demo scene shared by the planning examples and the tests.

A 7 x 5 x 3 m room (floor and ceiling) split by a wall at x = 2 m with a
1.2 x 1.0 m window, and a low beam across the room at x = 4 m, 1.1-1.4 m
above the floor.  Flying from one end to the other the X500 has to climb
through the window, then pass under or over the beam.  Obstacles are
point clouds sampled every 3 cm.
"""

import numpy as np

SPACING = 0.03

# The room the planners sample in (virtual joint ranges).
WORKSPACE = {
    "base_x_joint": (-0.5, 5.5),
    "base_y_joint": (-2.0, 2.0),
    "base_z_joint": (0.3, 2.7),
}


def _plane(origin, u, v, length_u, length_v):
    a = np.arange(0.0, length_u + 1e-9, SPACING)
    b = np.arange(0.0, length_v + 1e-9, SPACING)
    A, B = np.meshgrid(a, b, indexing="ij")
    return (
        np.asarray(origin)
        + A.reshape(-1, 1) * np.asarray(u)
        + B.reshape(-1, 1) * np.asarray(v)
    )


def window_scene() -> np.ndarray:
    """``(N, 3) float32`` point cloud of the demo room."""
    x, y, z = np.eye(3)
    floor = _plane([-1.0, -2.5, 0.0], x, y, 7.0, 5.0)
    ceiling = floor + [0.0, 0.0, 3.0]
    wall = _plane([2.0, -2.5, 0.0], y, z, 5.0, 3.0)
    window = (np.abs(wall[:, 1]) < 0.6) & (wall[:, 2] > 1.0) & (wall[:, 2] < 2.0)
    beam_side = _plane([4.0, -2.5, 1.1], y, z, 5.0, 0.3)
    beam_bottom = _plane([4.0, -2.5, 1.1], x, y, 0.2, 5.0)
    cloud = [
        floor,
        ceiling,
        wall[~window],
        beam_side,
        beam_side + [0.2, 0.0, 0.0],
        beam_bottom,
        beam_bottom + [0.0, 0.0, 0.3],
    ]
    return np.concatenate(cloud).astype(np.float32)


def full(x, y, z, yaw, pan=0.0, tilt=0.0) -> np.ndarray:
    """6-DOF configuration: body (x, y, z, yaw), gimbal (pan, tilt)."""
    return np.array([x, y, z, yaw, pan, tilt])


# (name, start, goal) — full 6-DOF configurations.
LEGS = [
    ("through", full(0.0, 0.0, 1.0, 0.0), full(5.0, 0.0, 1.0, 0.0)),
    ("back", full(5.0, 0.0, 1.0, 0.0), full(0.0, 0.0, 1.0, 0.0)),
    ("diagonal", full(0.0, -1.5, 0.6, 0.0), full(5.0, 1.5, 2.2, np.pi / 2)),
    (
        "look down",
        full(0.0, 1.5, 2.2, -np.pi / 2, 0.0, 0.0),
        full(5.0, -1.5, 0.6, np.pi, 1.0, -1.2),
    ),
]
