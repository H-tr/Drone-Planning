"""Type stubs for the ``_ompl_vamp`` C++ extension.

The actual implementation is in ``ext/ompl_vamp/ompl_vamp_ext.cpp`` and
ships as a compiled ``_ompl_vamp.cpython-*.so`` next to this file in the
installed package.  This stub mirrors the nanobind bindings exactly so
type checkers can resolve ``import drone_planning._ompl_vamp``.
"""

from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray

class PlanResult:
    """Result of a single ``OmplVampPlanner.plan`` call."""

    @property
    def solved(self) -> bool:
        """``True`` if OMPL found an exact solution within the time limit
        (approximate solutions, which stop short of the goal, are not)."""
        ...

    @property
    def path(self) -> NDArray[np.float64]:
        """``(N, dimension())`` solution waypoints in the planner's active
        joint space; empty when ``solved`` is false."""
        ...

    @property
    def planning_time_ns(self) -> int:
        """Wall-clock time spent inside ``ss.solve(...)``, in nanoseconds
        (simplification and interpolation not included)."""
        ...

    @property
    def path_cost(self) -> float:
        """Length of the (possibly simplified) solution path; ``inf`` when
        ``solved`` is false."""
        ...

class KinodynamicSettings:
    """Parameters of ``OmplVampPlanner.plan_kinodynamic`` (FLASK).

    See ``ext/ompl_vamp/plan_kinodynamic.hpp`` for field semantics;
    :class:`drone_planning.types.KinodynamicConfig` fills them from the
    robot limits in :mod:`drone_planning.x500`.  ``min_*`` / ``max_*``
    hold one value per active DOF; the horizontal pair (``base_x_joint``,
    ``base_y_joint``) is also bounded in norm when both are active.
    """

    min_velocity: list[float]
    max_velocity: list[float]
    min_acceleration: list[float]
    max_acceleration: list[float]
    max_horizontal_speed: float
    max_horizontal_acceleration: float
    rho: float
    limit_aware_duration: bool
    max_extension_time: float
    velocity_sample_scale: float
    velocity_metric_weight: float
    rest_sample_probability: float
    simplify: bool
    simplify_iterations: int
    simplify_time_limit: float
    max_iterations: int
    seed: int

    def __init__(self) -> None: ...

class FlatTrajectory:
    """Piecewise-cubic trajectory in the flat output space (C++ handle)."""

    @property
    def duration(self) -> float: ...
    @property
    def active_dim(self) -> int: ...
    @property
    def num_segments(self) -> int: ...
    def position(self, t: float) -> NDArray[np.float64]: ...
    def velocity(self, t: float) -> NDArray[np.float64]: ...
    def acceleration(self, t: float) -> NDArray[np.float64]: ...
    def sample(
        self, times: Sequence[float]
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]: ...
    def sample_uniform(self, dt: float) -> tuple[
        NDArray[np.float64],
        NDArray[np.float64],
        NDArray[np.float64],
        NDArray[np.float64],
    ]: ...
    def knot_times(self) -> list[float]: ...

class KinodynamicResult:
    """Result of ``OmplVampPlanner.plan_kinodynamic``."""

    @property
    def solved(self) -> bool: ...
    @property
    def trajectory(self) -> FlatTrajectory: ...
    @property
    def planning_time_ns(self) -> int:
        """Total planning time including simplification, in nanoseconds."""
        ...

    @property
    def simplify_time_ns(self) -> int: ...
    @property
    def cost(self) -> float:
        """LQMT cost ``sum_i w_i int y_i''^2 dt + rho T`` of the trajectory."""
        ...

    @property
    def iterations(self) -> int: ...
    @property
    def start_tree_size(self) -> int: ...
    @property
    def goal_tree_size(self) -> int: ...
    @property
    def edges_checked(self) -> int: ...

def flat_optimal_time(
    y0: Sequence[float],
    v0: Sequence[float],
    y1: Sequence[float],
    v1: Sequence[float],
    weights: Sequence[float],
    rho: float,
) -> tuple[float, float]:
    """Minimum-cost duration ``T*`` and cost ``J(T*)`` of the LQMT (cubic)
    motion between two flat states, ``J(T) = sum_i w_i int y_i''^2 dt + rho T``."""
    ...

class OmplVampPlanner:
    """OMPL planner with VAMP SIMD-accelerated collision checking.

    Two construction modes:

    * ``OmplVampPlanner()`` — full body, 6 DOF (x, y, z, yaw, gimbal pan,
      gimbal tilt).
    * ``OmplVampPlanner(active_indices, frozen_config)`` — subgroup over
      the listed joints; the C++ collision checker injects
      ``frozen_config`` for every other joint before every query.
    """

    def __init__(
        self,
        active_indices: Sequence[int] = ...,
        frozen_config: Sequence[float] = ...,
    ) -> None: ...
    def add_pointcloud(self, points: NDArray[np.float32], point_radius: float) -> None:
        """Set the scene point cloud, replacing any previous one.

        Args:
            points: ``(N, 3)`` float32 obstacle positions in world frame.
            point_radius: Inflation radius of every point (0 checks them
                as points: the robot's spheres already cover its meshes).
        """
        ...

    def remove_pointcloud(self) -> bool:
        """Drop the point cloud.  Returns ``False`` if there was none."""
        ...

    def has_pointcloud(self) -> bool:
        """``True`` if a point cloud is registered."""
        ...

    def add_sphere(self, center: Sequence[float], radius: float) -> None:
        """Add a single sphere obstacle (centre + radius) to the environment."""
        ...

    def clear_environment(self) -> None:
        """Remove all obstacles from the collision environment."""
        ...

    def plan(
        self,
        start: NDArray[np.float64],
        goal: NDArray[np.float64],
        planner_name: str = "rrtc",
        time_limit: float = 10.0,
        simplify: bool = True,
        interpolate: bool = True,
        resolution: float = 64.0,
    ) -> PlanResult:
        """Plan a collision-free path from ``start`` to ``goal``.

        Args:
            start: Active-DOF start configuration (length :meth:`dimension`).
            goal: Active-DOF goal configuration (length :meth:`dimension`).
            planner_name: OMPL planner identifier (e.g. ``"rrtc"``,
                ``"rrtstar"``, ``"prm"``, ``"bitstar"``).
            time_limit: Solver time limit in seconds.
            simplify: If true, run ``SimpleSetup::simplifySolution`` on the
                returned path.
            interpolate: If true, densify the simplified path:
                ``ceil(d * resolution)`` waypoints per edge of length ``d``
                when ``resolution > 0``, else OMPL's default.
            resolution: Waypoints per unit of state-space distance.
        """
        ...

    def plan_kinodynamic(
        self,
        start: NDArray[np.float64],
        start_velocity: NDArray[np.float64],
        goal: NDArray[np.float64],
        time_limit: float,
        settings: KinodynamicSettings,
    ) -> KinodynamicResult:
        """FLASK kinodynamic planning in the flat output space.

        Args:
            start: Active-DOF start configuration.
            start_velocity: Active-DOF start velocity, or empty for rest.
            goal: Active-DOF goal configuration, reached at rest.
            time_limit: Solver time limit in seconds.
            settings: Limits and planner parameters.
        """
        ...

    def simplify_path(
        self, path: NDArray[np.float64], time_limit: float = 1.0
    ) -> NDArray[np.float64]:
        """Run OMPL's shortcut simplifier on an ``(N, dimension())`` path
        against the current collision environment."""
        ...

    def interpolate_path(
        self, path: NDArray[np.float64], count: int = 0, resolution: float = 64.0
    ) -> NDArray[np.float64]:
        """Densify an ``(N, dimension())`` path along its straight edges
        (``count`` total waypoints, or ``resolution`` per unit distance);
        no collision check."""
        ...

    def validate(self, config: NDArray[np.float64]) -> bool:
        """``True`` if the active-DOF ``config`` is collision-free."""
        ...

    def validate_batch(self, configs: NDArray[np.float64]) -> NDArray[np.bool_]:
        """Per-configuration collision check of ``(N, dimension())``
        configurations, packed ``rake`` at a time into one SIMD sweep."""
        ...

    def dimension(self) -> int:
        """Number of active joints — 6 for the full body."""
        ...

    def lower_bounds(self) -> NDArray[np.float64]:
        """Lower joint limits of the active DOF."""
        ...

    def upper_bounds(self) -> NDArray[np.float64]:
        """Upper joint limits of the active DOF."""
        ...

    @staticmethod
    def default_joint_limits() -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """``(lower, upper)`` limits compiled into the robot model, full-body order."""
        ...

    def joint_limits(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """``(lower, upper)`` limits in effect, full-body order."""
        ...

    def set_joint_limits(
        self, lower: NDArray[np.float64], upper: NDArray[np.float64]
    ) -> None:
        """Replace the limits of every joint (full-body order; finite,
        ``lower < upper``).  They survive :meth:`set_subgroup`."""
        ...

    def filter_pointcloud(
        self,
        points: NDArray[np.float32],
        min_dist: float,
        max_range: float,
        origin: Sequence[float],
        workspace_min: Sequence[float],
        workspace_max: Sequence[float],
        cull: bool = True,
    ) -> NDArray[np.float32]:
        """Morton-curve spatial down-sampling of a point cloud."""
        ...

    def filter_self_from_pointcloud(
        self,
        points: NDArray[np.float32],
        point_radius: float,
        config: NDArray[np.float64],
    ) -> NDArray[np.float32]:
        """Remove points that collide with the robot at ``config`` or with
        the registered environment."""
        ...

    def set_subgroup(
        self, active_indices: Sequence[int], frozen_config: Sequence[float]
    ) -> None:
        """Switch active joints without rebuilding the collision environment.

        Args:
            active_indices: Positions in the full-body config that this
                planner plans over, in DOF order.
            frozen_config: Full-body stance (6 values) for every joint
                *not* in ``active_indices``.

        Raises:
            ValueError: For an index outside ``[0, 6)`` or a
                ``frozen_config`` without 6 entries (the subgroup
                constructor raises the same).
        """
        ...
