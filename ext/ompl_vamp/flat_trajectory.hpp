/**
 * Closed-form local motions in a differentially flat output space.
 *
 * Building blocks for the FLASK kinodynamic planner (Duong et al.,
 * "Ultrafast Sampling-based Kinodynamic Planning via Differential
 * Flatness", T-RO 2026), specialised to the X500:
 *
 *   * Flat output ``y``: the quadrotor's flat outputs are its position
 *     (x, y, z) and yaw; with roll and pitch held by the flight controller
 *     every one of them is commanded directly, and the gimbal pan / tilt
 *     joints are flat outputs of their own.  ``y`` is the active DOF.
 *   * Flat state ``z = (y, y_dot)`` with pseudo-control ``w = y_ddot``
 *     (order r = 2), so ``z`` follows a chain of double integrators.
 *   * Local motion between two flat states: the linear-quadratic
 *     minimum-time (LQMT) solution.  For r = 2 this is the cubic Hermite
 *     spline through both endpoint positions and velocities, and its
 *     cost ``J(T) = int |y_ddot|_R^2 dt + rho T`` is a rational function
 *     of the duration whose minimiser is a root of a quartic.
 *
 * A trajectory is a sequence of segments, each one cubic per active DOF.
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

namespace drone_planning {
namespace flat {

// ─── Scalar cubic ──────────────────────────────────────────────────────

/// ``p(t) = p0 + v0 t + c2 t^2 + c3 t^3`` on ``[0, T]``.
struct Cubic {
  double p0 = 0.0;
  double v0 = 0.0;
  double c2 = 0.0;
  double c3 = 0.0;

  /// Hermite fit: the unique cubic with p(0)=p0, p'(0)=v0, p(T)=p1,
  /// p'(T)=v1.  This is also the minimum-effort (LQMT, r = 2) motion.
  static auto hermite(double p0, double v0, double p1, double v1, double T)
      -> Cubic {
    const double d = p1 - p0;
    return {p0, v0, (3.0 * d - T * (2.0 * v0 + v1)) / (T * T),
            (-2.0 * d + T * (v0 + v1)) / (T * T * T)};
  }

  auto pos(double t) const -> double {
    return p0 + t * (v0 + t * (c2 + t * c3));
  }
  auto vel(double t) const -> double {
    return v0 + t * (2.0 * c2 + 3.0 * c3 * t);
  }
  auto acc(double t) const -> double { return 2.0 * c2 + 6.0 * c3 * t; }

  /// Exact ``[min p', max p']`` on ``[0, T]`` (endpoints + the vertex of
  /// the quadratic velocity).
  auto vel_range(double T) const -> std::pair<double, double> {
    double lo = std::min(vel(0.0), vel(T));
    double hi = std::max(vel(0.0), vel(T));
    if (c3 != 0.0) {
      const double tv = -c2 / (3.0 * c3);
      if (tv > 0.0 && tv < T) {
        lo = std::min(lo, vel(tv));
        hi = std::max(hi, vel(tv));
      }
    }
    return {lo, hi};
  }

  /// Exact ``[min p'', max p'']`` on ``[0, T]`` (acceleration is linear).
  auto acc_range(double T) const -> std::pair<double, double> {
    return std::minmax(acc(0.0), acc(T));
  }

  /// Exact ``[min p, max p]`` on ``[0, T]`` (endpoints + stationary
  /// points of the cubic).
  auto pos_range(double T) const -> std::pair<double, double> {
    double lo = std::min(pos(0.0), pos(T));
    double hi = std::max(pos(0.0), pos(T));
    auto visit = [&](double t) {
      if (t > 0.0 && t < T) {
        const double p = pos(t);
        lo = std::min(lo, p);
        hi = std::max(hi, p);
      }
    };
    // p'(t) = 3 c3 t^2 + 2 c2 t + v0
    const double A = 3.0 * c3, B = 2.0 * c2, C = v0;
    if (std::abs(A) < 1e-15) {
      if (std::abs(B) > 1e-15) visit(-C / B);
    } else {
      const double disc = B * B - 4.0 * A * C;
      if (disc >= 0.0) {
        const double s = std::sqrt(disc);
        visit((-B + s) / (2.0 * A));
        visit((-B - s) / (2.0 * A));
      }
    }
    return {lo, hi};
  }
};

/// Exact ``max |(x'(t), y'(t))|`` on ``[0, T]``: the speed is largest at an
/// endpoint or where ``d/dt |v|^2 / 2 = v . a`` vanishes.  ``v . a`` is a
/// cubic; its roots are bisected on the pieces where it is monotone.
inline auto max_planar_speed(const Cubic& x, const Cubic& y, double T)
    -> double {
  // v . a = g0 + g1 t + g2 t^2 + g3 t^3, summed over both axes.
  double g[4] = {0.0, 0.0, 0.0, 0.0};
  for (const Cubic* c : {&x, &y}) {
    g[0] += 2.0 * c->v0 * c->c2;
    g[1] += 6.0 * c->v0 * c->c3 + 4.0 * c->c2 * c->c2;
    g[2] += 18.0 * c->c2 * c->c3;
    g[3] += 18.0 * c->c3 * c->c3;
  }
  auto dot = [&](double t) {
    return g[0] + t * (g[1] + t * (g[2] + t * g[3]));
  };
  auto speed2 = [&](double t) {
    return x.vel(t) * x.vel(t) + y.vel(t) * y.vel(t);
  };
  // Monotone pieces: split [0, T] at the roots of (v . a)'.
  std::vector<double> knots{0.0};
  const double A = 3.0 * g[3], B = 2.0 * g[2], C = g[1];
  const double disc = B * B - 4.0 * A * C;
  if (A != 0.0 && disc >= 0.0) {
    const double s = std::sqrt(disc);
    for (double t : {(-B - s) / (2.0 * A), (-B + s) / (2.0 * A)})
      if (t > 0.0 && t < T) knots.push_back(t);
  } else if (A == 0.0 && B != 0.0 && -C / B > 0.0 && -C / B < T) {
    knots.push_back(-C / B);
  }
  knots.push_back(T);
  std::sort(knots.begin(), knots.end());
  double best = std::max(speed2(0.0), speed2(T));
  for (std::size_t k = 1; k < knots.size(); ++k) {
    double lo = knots[k - 1], hi = knots[k];
    if ((dot(lo) > 0.0) == (dot(hi) > 0.0)) continue;
    for (int it = 0; it < 60; ++it) {
      const double mid = 0.5 * (lo + hi);
      ((dot(mid) > 0.0) == (dot(lo) > 0.0) ? lo : hi) = mid;
    }
    best = std::max(best, speed2(0.5 * (lo + hi)));
  }
  return std::sqrt(best);
}

/// Exact ``max |(x''(t), y''(t))|`` on ``[0, T]``: the acceleration is
/// affine in t, so its norm is convex and largest at an endpoint.
inline auto max_planar_acceleration(const Cubic& x, const Cubic& y, double T)
    -> double {
  return std::max(std::hypot(x.acc(0.0), y.acc(0.0)),
                  std::hypot(x.acc(T), y.acc(T)));
}

// ─── LQMT cost and optimal duration ────────────────────────────────────

/// ``J(T) = a / T^3 - b / T^2 + c / T + rho T`` — the minimum effort of
/// the Hermite cubic between two flat states over duration T, summed
/// over flat outputs with weights ``r_i`` (Example 4 of the paper, with
/// ``R = diag(r)``), plus the time penalty.
struct LqmtTerms {
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;

  void add(double r, double p0, double v0, double p1, double v1) {
    const double d = p1 - p0;
    a += r * 12.0 * d * d;
    b += r * 12.0 * (v0 + v1) * d;
    c += r * 4.0 * (v0 * v0 + v0 * v1 + v1 * v1);
  }

  auto cost(double T, double rho) const -> double {
    return a / (T * T * T) - b / (T * T) + c / T + rho * T;
  }
};

/// Real roots of ``x^4 + p x^2 + q x + r = 0`` (Ferrari's method with a
/// safeguarded-Newton resolvent root, then a Newton polish on the
/// quartic).  Returns the number of roots written to ``roots``.
inline auto solve_depressed_quartic(double p, double q, double r,
                                    std::array<double, 4>& roots) -> int {
  int n = 0;
  // Roots of x^2 + B x + C = 0, written in the cancellation-free form.
  auto quadratic = [&](double B, double C) {
    const double disc = B * B - 4.0 * C;
    if (disc < 0.0) return;
    const double qq = -0.5 * (B + std::copysign(std::sqrt(disc), B));
    roots[n++] = qq;
    roots[n++] = C / qq;
  };

  const double scale = 1.0 + std::abs(p) + std::sqrt(std::abs(r));
  if (std::abs(q) <= 1e-13 * scale * std::sqrt(scale)) {
    // Biquadratic: u^2 + p u + r = 0 with u = x^2.
    const double disc = p * p - 4.0 * r;
    if (disc < 0.0) return 0;
    const double s = std::sqrt(disc);
    for (double u : {(-p + s) * 0.5, (-p - s) * 0.5}) {
      if (u >= 0.0) {
        roots[n++] = std::sqrt(u);
        roots[n++] = -std::sqrt(u);
      }
    }
  } else {
    // Resolvent cubic g(m) = 8m^3 + 8p m^2 + (2p^2 - 8r) m - q^2 has
    // g(0) = -q^2 < 0, so it owns a root m > 0.
    auto g = [&](double m) {
      return ((8.0 * m + 8.0 * p) * m + (2.0 * p * p - 8.0 * r)) * m - q * q;
    };
    auto dg = [&](double m) {
      return (24.0 * m + 16.0 * p) * m + (2.0 * p * p - 8.0 * r);
    };
    double lo = 0.0, hi = 1.0;
    while (g(hi) < 0.0) hi *= 2.0;
    double m = hi;
    for (int it = 0; it < 200; ++it) {
      const double gm = g(m);
      if (gm > 0.0) {
        hi = m;
      } else {
        lo = m;
      }
      const double d = dg(m);
      double next = (d != 0.0) ? m - gm / d : 0.5 * (lo + hi);
      if (!(next > lo && next < hi)) next = 0.5 * (lo + hi);
      if (std::abs(next - m) <= 1e-15 * std::max(1.0, m)) {
        m = next;
        break;
      }
      m = next;
    }
    const double s = std::sqrt(2.0 * m);
    const double t = q / (2.0 * s);
    quadratic(-s, 0.5 * p + m + t);
    quadratic(s, 0.5 * p + m - t);
  }

  for (int i = 0; i < n; ++i) {
    double x = roots[i];
    for (int it = 0; it < 3; ++it) {
      const double f = ((x * x + p) * x + q) * x + r;
      x -= f / ((4.0 * x * x + 2.0 * p) * x + q);
    }
    roots[i] = x;
  }
  return n;
}

/// Duration minimising ``J(T)`` over ``T > 0``: the best positive root
/// of ``dJ/dT = 0``, i.e. ``rho T^4 - c T^2 + 2 b T - 3 a = 0``
/// (Eq. 28 of the paper, generalised to per-output weights).  Since the
/// quartic is negative at 0 and positive at infinity, a positive root
/// exists unless the two states coincide.
inline auto optimal_time(const LqmtTerms& k, double rho) -> double {
  std::array<double, 4> roots{};
  const int n = solve_depressed_quartic(-k.c / rho, 2.0 * k.b / rho,
                                        -3.0 * k.a / rho, roots);
  double best_t = 0.0;
  double best_j = std::numeric_limits<double>::infinity();
  for (int i = 0; i < n; ++i) {
    if (roots[i] > 0.0 && k.cost(roots[i], rho) < best_j) {
      best_j = k.cost(roots[i], rho);
      best_t = roots[i];
    }
  }
  return best_t;
}

// ─── Segments and trajectories ─────────────────────────────────────────

/// One local motion: a cubic per active DOF on ``[t0, t0 + T]``.
struct Segment {
  double t0 = 0.0;
  double T = 0.0;
  std::vector<Cubic> cubics;
};

/// Evaluate a segment at local time ``s`` into active-DOF position /
/// velocity / acceleration (any output pointer may be null).
inline void eval_segment(const Segment& seg, double s, double* q, double* qd,
                         double* qdd) {
  for (std::size_t i = 0; i < seg.cubics.size(); ++i) {
    const Cubic& c = seg.cubics[i];
    if (q) q[i] = c.pos(s);
    if (qd) qd[i] = c.vel(s);
    if (qdd) qdd[i] = c.acc(s);
  }
}

/// A time-parameterised flat trajectory: consecutive segments with C^1
/// continuity in every active DOF (position and velocity).
class FlatTrajectory {
 public:
  FlatTrajectory() = default;
  explicit FlatTrajectory(int active_dim) : active_dim_(active_dim) {}

  void push(Segment seg) {
    seg.t0 = duration_;
    duration_ += seg.T;
    segments_.push_back(std::move(seg));
  }

  auto duration() const -> double { return duration_; }
  auto active_dim() const -> int { return active_dim_; }
  auto segments() const -> const std::vector<Segment>& { return segments_; }

  /// Locate the segment containing ``t`` (clamped to [0, duration]).
  auto locate(double t) const -> std::pair<const Segment*, double> {
    t = std::clamp(t, 0.0, duration_);
    auto it = std::upper_bound(
        segments_.begin(), segments_.end(), t,
        [](double tt, const Segment& sg) { return tt < sg.t0; });
    const Segment& seg =
        (it == segments_.begin()) ? segments_.front() : *(it - 1);
    return {&seg, std::clamp(t - seg.t0, 0.0, seg.T)};
  }

  void evaluate(double t, double* q, double* qd, double* qdd) const {
    auto [seg, s] = locate(t);
    eval_segment(*seg, s, q, qd, qdd);
  }

  using Matrix = std::vector<std::vector<double>>;

  auto sample(const std::vector<double>& times) const
      -> std::tuple<Matrix, Matrix, Matrix> {
    Matrix P(times.size(), std::vector<double>(active_dim_));
    Matrix V = P, A = P;
    for (std::size_t k = 0; k < times.size(); ++k) {
      evaluate(times[k], P[k].data(), V[k].data(), A[k].data());
    }
    return {std::move(P), std::move(V), std::move(A)};
  }

  auto sample_uniform(double dt) const
      -> std::tuple<std::vector<double>, Matrix, Matrix, Matrix> {
    std::vector<double> times;
    const auto n = static_cast<std::size_t>(std::floor(duration_ / dt));
    times.reserve(n + 2);
    for (std::size_t k = 0; k <= n; ++k) times.push_back(k * dt);
    if (duration_ - times.back() > 1e-12) times.push_back(duration_);
    auto [P, V, A] = sample(times);
    return {std::move(times), std::move(P), std::move(V), std::move(A)};
  }

  /// Segment boundary times ``[0, t_1, ..., duration]``.
  auto knot_times() const -> std::vector<double> {
    std::vector<double> out;
    out.reserve(segments_.size() + 1);
    for (const auto& sg : segments_) out.push_back(sg.t0);
    out.push_back(duration_);
    return out;
  }

 private:
  int active_dim_ = 0;
  double duration_ = 0.0;
  std::vector<Segment> segments_;
};

}  // namespace flat
}  // namespace drone_planning
