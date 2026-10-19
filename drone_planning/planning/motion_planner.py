"""OMPL + VAMP motion planner.

Uses OMPL for planning algorithms and VAMP for SIMD-accelerated
collision checking.  The entire planning pipeline runs in C++ via
the ``_ompl_vamp`` extension — Python only crosses the boundary
once per ``plan()`` call.

Supports subgroup planning: each subgroup operates on a reduced
state space while frozen joints are expanded to the full 6-DOF
X500 config before collision checks.  The body (x, y, z, yaw) is
fully actuated — the flight controller holds roll and pitch — so
every subgroup plans in a plain ``RealVectorStateSpace``.

:meth:`MotionPlanner.plan_kinodynamic` is the kinodynamic alternative:
FLASK (flatness-based kinodynamic RRT-Connect) returns a
time-parameterised trajectory that respects the PX4 velocity /
acceleration limits, for every subgroup.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Protocol, runtime_checkable

import numpy as np

from drone_planning.types import (
    KinodynamicConfig,
    KinodynamicResult,
    PlannerConfig,
    PlanningResult,
    PlanningStatus,
)


@runtime_checkable
class MotionPlannerBase(Protocol):
    """Protocol for motion planner backends."""

    @property
    def robot_name(self) -> str: ...

    @property
    def num_dof(self) -> int: ...

    def plan(self, start: np.ndarray, goal: np.ndarray) -> PlanningResult: ...

    def validate(self, configuration: np.ndarray) -> bool: ...


class MotionPlanner:
    """Motion planner using OMPL + VAMP C++ backend.

    All internals are private.  The public API accepts and returns
    only numpy arrays.

    For subgroup planners the helper methods :meth:`extract_config`,
    :meth:`embed_config`, and :meth:`embed_path` convert between the
    reduced DOF space used by the planner and the full 6-DOF body
    configuration.
    """

    def __init__(
        self,
        robot_name: str,
        config: PlannerConfig | None = None,
        pointcloud: np.ndarray | None = None,
        base_config: np.ndarray | None = None,
    ) -> None:
        from drone_planning._ompl_vamp import OmplVampPlanner
        from drone_planning.x500 import HOME_JOINTS

        if config is None:
            config = PlannerConfig()

        self._config = config
        self._robot_name = robot_name

        # Frozen 6-DOF joint values for any joint not controlled by
        # this planner.  Defaults to HOME_JOINTS, but the caller can
        # pass any 6-DOF array — e.g. the live state of the drone — so
        # the inactive joints are pinned wherever they currently are.
        if base_config is None:
            base_config = HOME_JOINTS
        self._base_config = np.asarray(base_config, dtype=np.float64).copy()
        if self._base_config.shape != HOME_JOINTS.shape:
            raise ValueError(
                f"base_config must have shape {HOME_JOINTS.shape}, "
                f"got {self._base_config.shape}"
            )

        active_indices = self._resolve_subgroup(robot_name)
        self._planner = OmplVampPlanner(active_indices, self._base_config.tolist())
        self._apply_subgroup_state(active_indices)

        if config.joint_limits:
            self.set_joint_limits(config.joint_limits)

        if pointcloud is not None:
            self.add_pointcloud(pointcloud)

    # ── Subgroup resolution ───────────────────────────────────────────

    @staticmethod
    def _resolve_subgroup(robot_name: str) -> list[int]:
        """Translate a subgroup name into active joint indices.

        Accepts both the explicit ``"x500_whole_body"`` name and the
        shorter alias ``"x500"`` for the full 6-DOF body.
        """
        from drone_planning.x500 import PLANNING_SUBGROUPS, x500_robot_config

        name = "x500_whole_body" if robot_name == "x500" else robot_name
        if name not in PLANNING_SUBGROUPS:
            raise ValueError(
                f"Unknown robot name '{robot_name}'. "
                f"Use one of: {available_robots()}"
            )
        full_names = x500_robot_config.joint_names
        return [full_names.index(j) for j in PLANNING_SUBGROUPS[name]["joints"]]

    def _apply_subgroup_state(self, active_indices: list[int]) -> None:
        """Update cached Python-side subgroup metadata."""
        from drone_planning.x500 import x500_robot_config

        full_names = x500_robot_config.joint_names
        self._joint_names = [full_names[i] for i in active_indices]
        if active_indices == list(range(len(full_names))):
            self._subgroup_indices = None
        else:
            self._subgroup_indices = np.array(active_indices)
        self._ndof = self._planner.dimension()

    # ── Properties ────────────────────────────────────────────────────

    @property
    def robot_name(self) -> str:
        return self._robot_name

    @property
    def num_dof(self) -> int:
        return self._ndof

    @property
    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        """``(lower, upper)`` joint limits of the active DOF."""
        return self._planner.lower_bounds(), self._planner.upper_bounds()

    @property
    def joint_limits(self) -> dict[str, tuple[float, float]]:
        """``{joint name: (lower, upper)}`` the planner samples in, for every
        joint of the full body.  :attr:`bounds` gives the active joints'
        ranges as arrays.
        """
        from drone_planning.x500 import x500_robot_config

        lower, upper = self._planner.joint_limits()
        return {
            name: (float(lo), float(hi))
            for name, lo, hi in zip(x500_robot_config.joint_names, lower, upper)
        }

    @property
    def joint_names(self) -> list[str]:
        """Joint names controlled by this planner, in DOF order."""
        return self._joint_names

    @property
    def is_subgroup(self) -> bool:
        """True if this planner controls a subset of the full body."""
        return self._subgroup_indices is not None

    @property
    def subgroup_indices(self) -> np.ndarray | None:
        """Indices of this planner's joints in the full 6-DOF config."""
        return self._subgroup_indices

    @property
    def base_config(self) -> np.ndarray:
        """The 6-DOF stance frozen for joints outside this subgroup."""
        return self._base_config.copy()

    # ── Joint limits ──────────────────────────────────────────────────

    def set_joint_limits(
        self, limits: Mapping[str, tuple[float, float]] | None = None
    ) -> None:
        """Set the joint ranges the planner samples in, by joint name.

        Joints left out of ``limits`` take the range compiled into the
        robot model, so ``None`` restores every compiled range.  The
        ranges cover the full body and are kept across
        :meth:`set_subgroup`.  Collision checking does not read them.

        Args:
            limits: ``{joint name: (lower, upper)}`` in metres for the
                virtual base joints and radians otherwise — e.g. the
                extent of the map for ``base_x_joint`` / ``base_y_joint``
                / ``base_z_joint``.

        Raises:
            ValueError: For an unknown joint name, or bounds that are not
                finite with ``lower < upper``.  The ranges in effect are
                then left unchanged.
        """
        from drone_planning._ompl_vamp import OmplVampPlanner
        from drone_planning.x500 import x500_robot_config

        names = x500_robot_config.joint_names
        lower, upper = OmplVampPlanner.default_joint_limits()
        for name, (lo, hi) in (limits or {}).items():
            if name not in names:
                raise ValueError(
                    f"Unknown joint {name!r}; the joints are {', '.join(names)}"
                )
            index = names.index(name)
            lower[index], upper[index] = lo, hi
        self._planner.set_joint_limits(lower, upper)

    # ── Pointcloud environment ────────────────────────────────────────

    def add_pointcloud(self, pointcloud: np.ndarray) -> None:
        """Set the scene pointcloud, replacing any previously-registered cloud.

        Args:
            pointcloud: ``(N, 3)`` array of obstacle positions in world
                frame.  Uses ``config.point_radius`` as the per-point
                inflation radius.
        """
        self._planner.add_pointcloud(
            np.asarray(pointcloud, dtype=np.float32), self._config.point_radius
        )

    def remove_pointcloud(self) -> bool:
        """Drop the currently-registered pointcloud.

        Returns ``False`` if there was no cloud to remove.
        """
        return self._planner.remove_pointcloud()

    @property
    def has_pointcloud(self) -> bool:
        """``True`` if a pointcloud is currently registered."""
        return self._planner.has_pointcloud()

    def clear_environment(self) -> None:
        """Drop every registered obstacle (spheres and point clouds)."""
        self._planner.clear_environment()

    # ── Point cloud filtering ────────────────────────────────────────

    def filter_pointcloud(
        self,
        pointcloud: np.ndarray,
        min_dist: float,
        max_range: float,
        origin: np.ndarray | list[float],
        workspace_min: np.ndarray | list[float],
        workspace_max: np.ndarray | list[float],
        cull: bool = True,
    ) -> np.ndarray:
        """Spatially downsample a point cloud via Morton-curve sorting.

        Keeps one representative point per ``min_dist`` neighbourhood and
        discards points farther than ``max_range`` from ``origin`` or
        outside the ``[workspace_min, workspace_max]`` bounding box.

        Args:
            pointcloud: ``(N, 3)`` array of 3-D points.
            min_dist: Minimum distance between two retained points.
            max_range: Maximum distance from ``origin`` to keep a point.
            origin: ``(3,)`` reference position for range culling.
            workspace_min: ``(3,)`` lower corner of the workspace AABB.
            workspace_max: ``(3,)`` upper corner of the workspace AABB.
            cull: If ``True`` (default), apply range and AABB culling.

        Returns:
            ``(M, 3)`` filtered point cloud with ``M <= N``.
        """
        return self._planner.filter_pointcloud(
            np.asarray(pointcloud, dtype=np.float32),
            float(min_dist),
            float(max_range),
            [float(x) for x in origin],
            [float(x) for x in workspace_min],
            [float(x) for x in workspace_max],
            cull,
        )

    def filter_self_from_pointcloud(
        self,
        pointcloud: np.ndarray,
        point_radius: float,
        config: np.ndarray,
    ) -> np.ndarray:
        """Remove points that collide with the robot body or environment.

        Computes forward kinematics at ``config``, then drops every
        point whose inflated sphere (radius ``point_radius``) overlaps
        any robot collision sphere or any registered obstacle.

        Args:
            pointcloud: ``(N, 3)`` array of 3-D points.
            point_radius: Inflation radius for each point.
            config: Active-DOF configuration (same space as ``plan``).

        Returns:
            ``(M, 3)`` filtered point cloud with ``M <= N``.
        """
        return self._planner.filter_self_from_pointcloud(
            np.asarray(pointcloud, dtype=np.float32),
            float(point_radius),
            np.asarray(config, dtype=np.float64),
        )

    # ── Subgroup switching ───────────────────────────────────────────

    def set_subgroup(
        self,
        robot_name: str,
        base_config: np.ndarray | None = None,
    ) -> None:
        """Switch active joints without rebuilding the collision environment.

        The pointcloud and the joint limits are preserved.

        Args:
            robot_name: Subgroup name from ``PLANNING_SUBGROUPS``, or
                ``"x500"`` for the full 6-DOF body.
            base_config: 6-DOF frozen config for inactive joints.
                Defaults to the previously stored base config.
        """
        if base_config is not None:
            self._base_config = np.asarray(base_config, dtype=np.float64).copy()
        self._robot_name = robot_name
        active_indices = self._resolve_subgroup(robot_name)
        self._planner.set_subgroup(active_indices, self._base_config.tolist())
        self._apply_subgroup_state(active_indices)

    # ── Subgroup helpers ──────────────────────────────────────────────

    def extract_config(self, full_config: np.ndarray) -> np.ndarray:
        """Extract this planner's joints from a full 6-DOF configuration."""
        full_config = np.asarray(full_config, dtype=np.float64)
        if self._subgroup_indices is None:
            return full_config.copy()
        return full_config[self._subgroup_indices].copy()

    def embed_config(
        self,
        config: np.ndarray,
        base_config: np.ndarray | None = None,
    ) -> np.ndarray:
        """Embed a subgroup config into a full 6-DOF configuration.

        ``base_config`` defaults to the planner's stored base — the same
        6-DOF values the C++ collision checker injects for inactive
        joints — so the embedded config matches what was validated.
        """
        config = np.asarray(config, dtype=np.float64)
        if self._subgroup_indices is None:
            return config.copy()

        if base_config is None:
            base_config = self._base_config
        full = np.array(base_config, dtype=np.float64)
        full[self._subgroup_indices] = config
        return full

    def embed_path(
        self,
        path: np.ndarray,
        base_config: np.ndarray | None = None,
    ) -> np.ndarray:
        """Convert a subgroup path ``(N, sub_dof)`` to ``(N, 6)``.

        ``base_config`` defaults to the planner's stored base — the same
        6-DOF values the C++ collision checker injects for inactive
        joints — so the embedded path matches what was validated.
        """
        path = np.asarray(path, dtype=np.float64)
        if self._subgroup_indices is None:
            return path.copy()

        if base_config is None:
            base_config = self._base_config
        n = path.shape[0]
        full_path = np.tile(np.array(base_config, dtype=np.float64), (n, 1))
        full_path[:, self._subgroup_indices] = path
        return full_path

    # ── Planning ──────────────────────────────────────────────────────

    def plan(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        time_limit: float | None = None,
    ) -> PlanningResult:
        """Plan a collision-free path from start to goal.

        Args:
            start: Start configuration (active DOF).
            goal: Goal configuration (active DOF).
            time_limit: Optional per-call override for the solver time
                limit.  Defaults to ``self._config.time_limit``.
        """
        start = np.asarray(start, dtype=np.float64)
        goal = np.asarray(goal, dtype=np.float64)

        if not self._planner.validate(start):
            return PlanningResult(
                PlanningStatus.INVALID_START, None, 0, 0, float("inf")
            )
        if not self._planner.validate(goal):
            return PlanningResult(PlanningStatus.INVALID_GOAL, None, 0, 0, float("inf"))

        if time_limit is None:
            time_limit = self._config.time_limit

        result = self._planner.plan(
            start,
            goal,
            self._config.planner_name,
            time_limit,
            self._config.simplify,
            self._config.interpolate,
            self._config.resolution,
        )

        if not result.solved:
            return PlanningResult(
                PlanningStatus.FAILED, None, result.planning_time_ns, 0, float("inf")
            )
        return PlanningResult(
            PlanningStatus.SUCCESS,
            np.array(result.path),
            result.planning_time_ns,
            0,
            result.path_cost,
        )

    def plan_kinodynamic(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        start_velocity: np.ndarray | None = None,
        config: KinodynamicConfig | None = None,
    ) -> KinodynamicResult:
        """Plan a dynamically feasible, time-parameterised trajectory (FLASK).

        Kinodynamic RRT-Connect in the differentially flat output space
        (Duong et al., T-RO 2026): every tree edge is the closed-form
        minimum-effort cubic between two (position, velocity) states,
        validated with SIMD-batched collision checks and the velocity /
        acceleration limits from :mod:`drone_planning.x500` — the norm of
        the horizontal velocity and acceleration, the climb and descent
        rates, the yaw rate and the gimbal rates.  The output is the
        velocity setpoint stream a PX4 offboard controller tracks.

        Args:
            start: Start configuration (active DOF).
            goal: Goal configuration (active DOF).  Reached at rest.
            start_velocity: Optional active-DOF velocity at ``start`` —
                e.g. ``traj.velocity(t)`` of the trajectory being
                executed, for replanning on the fly.  Defaults to rest.
            config: Kinodynamic planner parameters.

        Returns:
            A :class:`~drone_planning.types.KinodynamicResult` whose
            ``trajectory`` is a
            :class:`~drone_planning.trajectory.KinodynamicTrajectory`.
        """
        from drone_planning.trajectory import KinodynamicTrajectory

        if config is None:
            config = KinodynamicConfig()
        start = np.asarray(start, dtype=np.float64)
        goal = np.asarray(goal, dtype=np.float64)
        velocity = np.zeros(0) if start_velocity is None else start_velocity

        for q, status in (
            (start, PlanningStatus.INVALID_START),
            (goal, PlanningStatus.INVALID_GOAL),
        ):
            if not self._planner.validate(q):
                return KinodynamicResult(status, None, 0, 0, float("inf"))

        r = self._planner.plan_kinodynamic(
            start,
            np.asarray(velocity, dtype=np.float64),
            goal,
            config.time_limit,
            self._kinodynamic_settings(config),
        )
        return KinodynamicResult(
            status=PlanningStatus.SUCCESS if r.solved else PlanningStatus.FAILED,
            trajectory=KinodynamicTrajectory(r.trajectory) if r.solved else None,
            planning_time_ns=r.planning_time_ns,
            iterations=r.iterations,
            cost=r.cost,
            simplify_time_ns=r.simplify_time_ns,
            start_tree_size=r.start_tree_size,
            goal_tree_size=r.goal_tree_size,
            edges_checked=r.edges_checked,
        )

    def _kinodynamic_settings(self, config: KinodynamicConfig):
        """Translate a :class:`KinodynamicConfig` + robot limits into the
        C++ ``KinodynamicSettings`` for this subgroup."""
        from drone_planning._ompl_vamp import KinodynamicSettings
        from drone_planning.x500 import (
            HORIZONTAL_MAX_ACCELERATION,
            HORIZONTAL_MAX_SPEED,
            JOINT_ACCELERATION_LIMITS,
            JOINT_VELOCITY_LIMITS,
        )

        vs, acs = config.velocity_scale, config.acceleration_scale
        s = KinodynamicSettings()
        s.min_velocity = [JOINT_VELOCITY_LIMITS[j][0] * vs for j in self._joint_names]
        s.max_velocity = [JOINT_VELOCITY_LIMITS[j][1] * vs for j in self._joint_names]
        s.min_acceleration = [
            JOINT_ACCELERATION_LIMITS[j][0] * acs for j in self._joint_names
        ]
        s.max_acceleration = [
            JOINT_ACCELERATION_LIMITS[j][1] * acs for j in self._joint_names
        ]
        s.max_horizontal_speed = HORIZONTAL_MAX_SPEED * vs
        s.max_horizontal_acceleration = HORIZONTAL_MAX_ACCELERATION * acs
        s.rho = config.rho
        s.limit_aware_duration = config.limit_aware_duration
        s.max_extension_time = config.max_extension_time
        s.velocity_sample_scale = config.velocity_sample_scale
        s.velocity_metric_weight = config.velocity_metric_weight
        s.rest_sample_probability = config.rest_sample_probability
        s.simplify = config.simplify
        s.simplify_iterations = config.simplify_iterations
        s.simplify_time_limit = config.simplify_time_limit
        s.max_iterations = config.max_iterations
        s.seed = config.seed
        return s

    def simplify_path(self, path: np.ndarray, time_limit: float = 1.0) -> np.ndarray:
        """Run OMPL's shortcut-based path simplifier on ``path``.

        Same pipeline ``plan(..., simplify=True)`` uses internally
        (``reduceVertices`` + ``collapseCloseVertices`` + ``shortcutPath``
        + B-spline smoothing), but detached so you can apply it to any
        path you already have — e.g. replay an old plan with a
        different collision environment.

        Args:
            path: ``(N, ndof)`` array of waypoints in the planner's
                active DOF space.
            time_limit: Wall-clock budget for the simplifier, seconds.

        Returns:
            ``(M, ndof)`` simplified waypoint array with ``M <= N``.
        """
        return self._planner.simplify_path(
            np.asarray(path, dtype=np.float64), float(time_limit)
        )

    def interpolate_path(
        self,
        path: np.ndarray,
        count: int = 0,
        resolution: float = 64.0,
    ) -> np.ndarray:
        """Densify ``path`` with uniform waypoints along every edge.

        Three modes (pick one; the other must be zero):

            * ``count > 0``        — exactly that many total waypoints
              distributed proportionally to edge length.
            * ``resolution > 0.0`` — ``ceil(edge_length * resolution)``
              waypoints per edge (uniform density in state-space
              distance — the default).
            * both ``0``           — OMPL's default longest-valid-segment
              fraction.

        No collision check is performed — the densification only lifts
        points along the existing (straight) edges.

        Args:
            path: ``(N, ndof)`` waypoint array.
            count: Exact total waypoint count if ``> 0``.
            resolution: Waypoints per unit state-space distance if
                ``> 0.0``.

        Returns:
            ``(M, ndof)`` densified waypoint array with ``M >= N``.
        """
        return self._planner.interpolate_path(
            np.asarray(path, dtype=np.float64), int(count), float(resolution)
        )

    def validate(self, configuration: np.ndarray) -> bool:
        """Check if a configuration is collision-free."""
        return self._planner.validate(np.asarray(configuration, dtype=np.float64))

    def validate_batch(self, configurations: np.ndarray) -> np.ndarray:
        """Batched collision check — one SIMD block per ``rake`` configs.

        Packs ``rake`` distinct configurations into a single VAMP
        ``ConfigurationBlock<rake>`` and runs one ``fkcc<rake>`` call
        per block, so ``N`` queries cost ``ceil(N / rake)`` SIMD
        sweeps in the common case.  When a packed block fails, that
        block's configurations are re-checked one by one, so the
        returned array is always exactly one bool per input.

        Args:
            configurations: ``(N, ndof)`` array of active-DOF
                configurations.

        Returns:
            ``(N,)`` boolean array; ``True`` at index ``i`` iff
            ``configurations[i]`` is collision-free.
        """
        return self._planner.validate_batch(
            np.asarray(configurations, dtype=np.float64)
        )

    def sample_valid(self) -> np.ndarray:
        """Sample a random collision-free configuration."""
        lo, hi = self.bounds
        while True:
            config = np.random.uniform(lo, hi)
            if self._planner.validate(config):
                return config


def available_robots() -> list[str]:
    """Return all available robot names for planning."""
    from drone_planning.x500 import PLANNING_SUBGROUPS

    return ["x500"] + sorted(PLANNING_SUBGROUPS.keys())


def create_planner(
    robot_name: str = "x500",
    config: PlannerConfig | None = None,
    pointcloud: np.ndarray | None = None,
    base_config: np.ndarray | None = None,
) -> MotionPlanner:
    """Create a motion planner for any robot or subgroup.

    Args:
        robot_name: Robot or subgroup name. Use :func:`available_robots`
            to list all names. The default ``"x500"`` is an alias for
            the full whole-body planner (6 DOF: body + gimbal).
        config: Planner configuration (uses defaults if None).
        pointcloud: ``(N, 3)`` obstacle point cloud (optional).
        base_config: 6-DOF values to inject for joints not controlled
            by this planner (i.e. the frozen joints of a subgroup — the
            gimbal for ``"x500_base"``).  Defaults to ``HOME_JOINTS``.

    Returns:
        A :class:`MotionPlanner` instance.
    """
    return MotionPlanner(robot_name, config, pointcloud, base_config)
