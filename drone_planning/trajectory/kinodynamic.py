"""Kinodynamic trajectory — the time-parameterised output of FLASK planning.

Thin Python veneer over the C++ ``FlatTrajectory`` handle returned by
:meth:`~drone_planning.planning.MotionPlanner.plan_kinodynamic`.  The
trajectory is a chain of closed-form cubic segments in the flat output
space, so it can be sampled exactly at any time — no separate time
parameterisation step is needed.

It exposes the same sampling API as :class:`~drone_planning.trajectory.Trajectory`
(``duration``, ``position``, ``velocity``, ``acceleration``, ``sample``,
``sample_uniform``) in the planner's active-DOF layout.  For the body the
velocity is the world-frame ``(x_dot, y_dot, z_dot, yaw_rate)`` — the
velocity setpoint a PX4 offboard controller takes — and the gimbal rates
follow.

Typical feed-forward tracking::

    result = planner.plan_kinodynamic(start, goal)
    traj = result.trajectory
    times, q_ref, qd_ref, _ = traj.sample_uniform(dt=0.02)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from drone_planning._ompl_vamp import FlatTrajectory as _FlatTrajectory


@dataclass(frozen=True)
class KinodynamicTrajectory:
    """A dynamically feasible trajectory produced by FLASK planning.

    Instances are immutable handles around a C++ piecewise-cubic
    trajectory; query them via :meth:`position`, :meth:`velocity`,
    :meth:`acceleration`, or one of the batch samplers.
    """

    _handle: "_FlatTrajectory"

    @property
    def duration(self) -> float:
        """Trajectory duration in seconds."""
        return float(self._handle.duration)

    @property
    def num_dof(self) -> int:
        """Active DOF of the planner that produced the trajectory."""
        return int(self._handle.active_dim)

    @property
    def num_segments(self) -> int:
        """Number of closed-form segments (after simplification)."""
        return int(self._handle.num_segments)

    @property
    def knot_times(self) -> np.ndarray:
        """Segment boundary times ``[0, t_1, ..., duration]``."""
        return np.asarray(self._handle.knot_times(), dtype=np.float64)

    # ── Pointwise sampling ────────────────────────────────────────────

    def position(self, t: float) -> np.ndarray:
        """Configuration at time ``t`` (seconds), clamped to ``[0, duration]``."""
        return np.asarray(self._handle.position(float(t)), dtype=np.float64)

    def velocity(self, t: float) -> np.ndarray:
        """Active-DOF velocity at time ``t`` (world-frame for the body)."""
        return np.asarray(self._handle.velocity(float(t)), dtype=np.float64)

    def acceleration(self, t: float) -> np.ndarray:
        """Active-DOF acceleration at time ``t`` (world-frame for the body)."""
        return np.asarray(self._handle.acceleration(float(t)), dtype=np.float64)

    # ── Batch sampling ────────────────────────────────────────────────

    def sample(self, times: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Sample at the user-supplied ``times`` grid.

        Args:
            times: ``(T,)`` array of sample times in seconds.

        Returns:
            ``(positions, velocities, accelerations)`` — each ``(T, ndof)``.
        """
        times = np.ascontiguousarray(times, dtype=np.float64).reshape(-1)
        positions, velocities, accelerations = self._handle.sample(times.tolist())
        return (
            np.asarray(positions, dtype=np.float64),
            np.asarray(velocities, dtype=np.float64),
            np.asarray(accelerations, dtype=np.float64),
        )

    def sample_uniform(
        self, dt: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Uniformly-spaced rollout at step ``dt``.

        The returned ``times`` always start at ``0`` and end at
        :attr:`duration`, which may make the final step shorter than
        ``dt`` — this matches what a streaming controller expects.

        Args:
            dt: Sample interval in seconds (must be ``> 0``).

        Returns:
            ``(times, positions, velocities, accelerations)`` —
            ``times`` has shape ``(T,)``; state arrays have shape
            ``(T, ndof)``.
        """
        times, positions, velocities, accelerations = self._handle.sample_uniform(
            float(dt)
        )
        return (
            np.asarray(times, dtype=np.float64),
            np.asarray(positions, dtype=np.float64),
            np.asarray(velocities, dtype=np.float64),
            np.asarray(accelerations, dtype=np.float64),
        )
