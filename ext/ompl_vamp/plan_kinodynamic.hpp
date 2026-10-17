/**
 * FLASK kinodynamic planner: RRT-Connect in the flat state space with
 * closed-form cubic edges and SIMD-batched collision checking.
 *
 * Follows Duong et al., "Ultrafast Sampling-based Kinodynamic Planning
 * via Differential Flatness" (T-RO 2026):
 *
 *   * Tree nodes are flat states ``z = (y, y_dot)``; see
 *     ``flat_trajectory.hpp`` for the X500's flat outputs (every active
 *     DOF: position, yaw and the gimbal joints).
 *   * Every edge is the closed-form LQMT motion — the cubic Hermite
 *     between its endpoint states — with the cost-optimal duration T*
 *     (Eq. 26/28), optionally stretched until the velocity /
 *     acceleration bounds hold ("limit-aware duration").
 *   * Edge validation checks the bounds exactly on the cubics — joint
 *     ranges, per-DOF velocity / acceleration ranges (asymmetric for the
 *     climb rate) and the norm of the horizontal velocity / acceleration
 *     — then evaluates the closed form at time samples and packs the
 *     configurations into VAMP ``ConfigurationBlock<rake>`` batches for
 *     ``fkcc`` (Alg. 3).
 *   * The solution is simplified like vamp::planning::simplify does for
 *     geometric paths: greedy shortcuts (Alg. 4) and subdivide-then-cut-
 *     corners smoothing, each change kept only if it lowers the LQMT cost.
 */

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <utility>
#include <vector>

#include "flat_trajectory.hpp"
#include "validity.hpp"

namespace drone_planning {

/// X500 joints whose velocity / acceleration are bounded in norm:
/// base_x_joint and base_y_joint (horizontal flight).
inline constexpr std::array<int, 2> kHorizontalJoints{0, 1};

struct KinodynamicSettings {
  /// Per active DOF ranges, in joint units per s / s^2.
  std::vector<double> min_velocity;
  std::vector<double> max_velocity;
  std::vector<double> min_acceleration;
  std::vector<double> max_acceleration;
  /// Norm bounds on the horizontal velocity / acceleration, used when
  /// both base_x_joint and base_y_joint are active.
  double max_horizontal_speed = 0.0;         ///< m/s
  double max_horizontal_acceleration = 0.0;  ///< m/s^2

  /// LQMT time weight.  Effort weights are ``1 / a^2`` per flat output
  /// (``a`` the smaller acceleration bound), so for rest-to-rest edges the
  /// dominant DOF peaks at ``sqrt(rho)`` times that bound.
  double rho = 1.0;
  /// Stretch T* until the velocity / acceleration bounds hold (a few
  /// multiplicative steps).  Off = the paper's pure T*.
  bool limit_aware_duration = true;

  /// Tree extension step (s): longer BVP solutions are truncated at
  /// this duration, keeping the node on the optimal cubic.
  double max_extension_time = 3.0;
  /// Fraction of the velocity limits used when sampling node velocities.
  double velocity_sample_scale = 0.5;
  /// Time constant (s) weighting velocity differences in the
  /// nearest-neighbour metric ``|dy|^2 + lambda^2 |dv|^2``.
  double velocity_metric_weight = 0.3;
  /// Probability of sampling a zero-velocity target.  Rest-to-rest edges
  /// are straight lines in configuration space, so the planner degrades
  /// gracefully to geometric RRT-Connect where curved cubics collide.
  double rest_sample_probability = 0.3;

  /// Post-processing after vamp::planning::simplify: repeat greedy
  /// shortcuts and subdivide-then-cut-corners smoothing until neither
  /// improves the trajectory, for at most ``simplify_iterations`` rounds.
  bool simplify = true;
  std::size_t simplify_iterations = 5;
  double simplify_time_limit = 0.05;  ///< s
  std::size_t max_iterations = 200000;
  std::uint64_t seed = 0;  ///< 0 = nondeterministic
};

struct KinodynamicResult {
  bool solved = false;
  flat::FlatTrajectory trajectory;
  int64_t planning_time_ns = 0;  ///< total, including simplification
  int64_t simplify_time_ns = 0;
  double cost = std::numeric_limits<double>::infinity();  ///< LQMT cost
  std::size_t iterations = 0;
  std::size_t start_tree_size = 0;
  std::size_t goal_tree_size = 0;
  std::size_t edges_checked = 0;
};

class FlaskPlanner {
 public:
  FlaskPlanner(const VampEnv& env, const std::vector<int>& active_indices,
               const std::vector<float>& frozen_config,
               const std::vector<double>& lower,
               const std::vector<double>& upper,
               const KinodynamicSettings& settings)
      : env_(env),
        active_indices_(active_indices),
        frozen_config_(frozen_config),
        n_(static_cast<int>(active_indices.size())),
        lo_(lower),
        hi_(upper),
        s_(settings) {
    weight_.resize(n_);
    for (int f = 0; f < n_; ++f) {
      const double a =
          std::min(-s_.min_acceleration[f], s_.max_acceleration[f]);
      weight_[f] = 1.0 / (a * a);
    }
    const auto hx = std::find(active_indices_.begin(), active_indices_.end(),
                              kHorizontalJoints[0]);
    const auto hy = std::find(active_indices_.begin(), active_indices_.end(),
                              kHorizontalJoints[1]);
    if (hx != active_indices_.end() && hy != active_indices_.end()) {
      hx_ = static_cast<int>(hx - active_indices_.begin());
      hy_ = static_cast<int>(hy - active_indices_.begin());
    }
    rng_.seed(s_.seed ? s_.seed : std::random_device{}());
  }

  auto plan(const std::vector<double>& start_q,
            const std::vector<double>& start_qd,
            const std::vector<double>& goal_q, double time_limit)
      -> KinodynamicResult {
    const auto t0 = std::chrono::steady_clock::now();
    auto elapsed = [&] {
      return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                           t0)
          .count();
    };
    KinodynamicResult result;
    result.trajectory = flat::FlatTrajectory(n_);
    start_tree_ = {Node{
        start_q, start_qd.empty() ? std::vector<double>(n_, 0.0) : start_qd}};
    goal_tree_ = {Node{goal_q, std::vector<double>(n_, 0.0)}};

    // Direct BVP first — the "fast path" of RRT-Connect.
    int conn_start = -1, conn_goal = -1;
    double T_conn = 0.0;
    if (connect(start_tree_[0], goal_tree_[0], T_conn))
      conn_start = conn_goal = 0;

    bool a_is_start = true;
    while (conn_start < 0 && elapsed() < time_limit &&
           result.iterations < s_.max_iterations) {
      ++result.iterations;
      auto& tree_a = a_is_start ? start_tree_ : goal_tree_;
      auto& tree_b = a_is_start ? goal_tree_ : start_tree_;

      const Node target = sample_state();
      const int near_a = nearest(tree_a, target);
      const int x = extend(tree_a, near_a, target, a_is_start);
      if (x >= 0) {
        const Node xn = tree_a[x];
        const int near_b = nearest(tree_b, xn);
        const int si = a_is_start ? x : near_b;
        const int gi = a_is_start ? near_b : x;
        if (connect(start_tree_[si], goal_tree_[gi], T_conn)) {
          conn_start = si;
          conn_goal = gi;
          break;
        }
        // Grow the other tree one step toward the new node.
        extend(tree_b, near_b, xn, !a_is_start);
      }
      // Balance: always grow the smaller tree next.
      a_is_start = start_tree_.size() <= goal_tree_.size();
    }

    result.start_tree_size = start_tree_.size();
    result.goal_tree_size = goal_tree_.size();
    if (conn_start < 0) {
      result.edges_checked = edges_checked_;
      result.planning_time_ns = ns_since(t0);
      return result;
    }

    // The solution path, VAMP style: a list of states, where node k also
    // carries the duration of the edge that reaches it from node k - 1.
    std::vector<Node> path;
    for (int i = conn_start; i >= 0; i = start_tree_[i].parent)
      path.push_back(start_tree_[i]);
    std::reverse(path.begin(), path.end());
    Node goal_side = goal_tree_[conn_goal];
    goal_side.T = T_conn;
    path.push_back(goal_side);
    for (int j = conn_goal; goal_tree_[j].parent >= 0;
         j = goal_tree_[j].parent) {
      Node next = goal_tree_[goal_tree_[j].parent];
      next.T = goal_tree_[j].T;  // goal-tree edges point to the parent
      path.push_back(next);
    }

    // Like VAMP, a direct start -> goal connection needs no simplification.
    const bool direct = conn_start == 0 && conn_goal == 0;
    if (s_.simplify && !direct) {
      const auto ts = std::chrono::steady_clock::now();
      simplify(path, ts);
      result.simplify_time_ns = ns_since(ts);
    }

    result.cost = 0.0;
    for (std::size_t k = 1; k < path.size(); ++k) {
      result.cost += edge_cost(path[k - 1], path[k], path[k].T);
      result.trajectory.push(segment(path[k - 1], path[k], path[k].T));
    }
    result.solved = true;
    result.edges_checked = edges_checked_;
    result.planning_time_ns = ns_since(t0);
    return result;
  }

 private:
  struct Node {
    std::vector<double> y;  ///< flat position
    std::vector<double> v;  ///< flat velocity
    int parent = -1;
    double T = 0.0;  ///< duration of the edge to the parent
  };

  /// VAMP's straight-edge density (``resolution`` samples per unit of
  /// configuration-space arc length; 1.1 covers chord sums
  /// underestimating the arc).  Like VAMP's own edges this is
  /// collision-free up to the sampling resolution.
  static constexpr double kCollisionDensity = 1.1;
  /// Grid (s) on which the edge's arc length is measured.
  static constexpr double kArcLengthDt = 0.05;
  static constexpr double kTol = 1e-6;

  const VampEnv& env_;
  const std::vector<int>& active_indices_;
  const std::vector<float>& frozen_config_;
  int n_;
  std::vector<double> lo_, hi_, weight_;
  int hx_ = -1, hy_ = -1;  ///< active positions of the horizontal pair
  KinodynamicSettings s_;
  std::mt19937_64 rng_;
  std::uniform_real_distribution<double> unit_{0.0, 1.0};
  std::vector<Node> start_tree_, goal_tree_;
  std::size_t edges_checked_ = 0;

  static auto ns_since(std::chrono::steady_clock::time_point t) -> int64_t {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now() - t)
        .count();
  }

  // ── Sampling ───────────────────────────────────────────────────────

  auto sample_state() -> Node {
    Node n;
    n.y.resize(n_);
    n.v.assign(n_, 0.0);
    for (int f = 0; f < n_; ++f)
      n.y[f] = lo_[f] + (hi_[f] - lo_[f]) * unit_(rng_);
    if (unit_(rng_) < s_.rest_sample_probability) return n;
    for (int f = 0; f < n_; ++f) {
      const double u = unit_(rng_);
      const double v = s_.velocity_sample_scale *
                       (u < 0.5 ? 2.0 * (0.5 - u) * s_.min_velocity[f]
                                : 2.0 * (u - 0.5) * s_.max_velocity[f]);
      // A DOF moving toward its limit must be able to brake before it:
      // |v| <= sqrt(2 a d) with the decelerating bound.  Faster states
      // overshoot the limit whatever the next edge does.
      const double vcap =
          v > 0.0 ? std::sqrt(2.0 * -s_.min_acceleration[f] * (hi_[f] - n.y[f]))
                  : std::sqrt(2.0 * s_.max_acceleration[f] * (n.y[f] - lo_[f]));
      n.v[f] = std::clamp(v, -vcap, vcap);
    }
    return n;
  }

  auto metric(const Node& a, const Node& b) const -> double {
    const double lam2 = s_.velocity_metric_weight * s_.velocity_metric_weight;
    double d = 0.0;
    for (int f = 0; f < n_; ++f) {
      const double dy = a.y[f] - b.y[f];
      const double dv = a.v[f] - b.v[f];
      d += dy * dy + lam2 * dv * dv;
    }
    return d;
  }

  auto nearest(const std::vector<Node>& tree, const Node& target) const -> int {
    int best = 0;
    double best_d = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < tree.size(); ++i) {
      const double d = metric(tree[i], target);
      if (d < best_d) {
        best_d = d;
        best = static_cast<int>(i);
      }
    }
    return best;
  }

  // ── Edges ──────────────────────────────────────────────────────────

  auto segment(const Node& a, const Node& b, double T) const -> flat::Segment {
    flat::Segment seg;
    seg.T = T;
    seg.cubics.resize(n_);
    for (int f = 0; f < n_; ++f)
      seg.cubics[f] = flat::Cubic::hermite(a.y[f], a.v[f], b.y[f], b.v[f], T);
    return seg;
  }

  auto lqmt_terms(const Node& a, const Node& b) const -> flat::LqmtTerms {
    flat::LqmtTerms k;
    for (int f = 0; f < n_; ++f)
      k.add(weight_[f], a.y[f], a.v[f], b.y[f], b.v[f]);
    return k;
  }

  auto edge_cost(const Node& a, const Node& b, double T) const -> double {
    return lqmt_terms(a, b).cost(T, s_.rho);
  }

  /// Largest ratio "achieved / allowed" over the velocity and
  /// acceleration bounds of an edge.  Velocity ratios scale ~1/T and
  /// acceleration ratios ~1/T^2, so the returned value is the factor by
  /// which T should grow.
  auto duration_stretch(const flat::Segment& seg) const -> double {
    double ratio = 0.0;
    for (int f = 0; f < n_; ++f) {
      const auto [vlo, vhi] = seg.cubics[f].vel_range(seg.T);
      const auto [alo, ahi] = seg.cubics[f].acc_range(seg.T);
      ratio =
          std::max({ratio, vhi / s_.max_velocity[f], vlo / s_.min_velocity[f],
                    std::sqrt(std::max(ahi / s_.max_acceleration[f],
                                       alo / s_.min_acceleration[f]))});
    }
    if (hx_ >= 0) {
      const flat::Cubic& x = seg.cubics[hx_];
      const flat::Cubic& y = seg.cubics[hy_];
      ratio = std::max(
          {ratio, flat::max_planar_speed(x, y, seg.T) / s_.max_horizontal_speed,
           std::sqrt(flat::max_planar_acceleration(x, y, seg.T) /
                     s_.max_horizontal_acceleration)});
    }
    return ratio;
  }

  auto choose_duration(const Node& a, const Node& b) const -> double {
    double T = flat::optimal_time(lqmt_terms(a, b), s_.rho);
    if (!s_.limit_aware_duration) return T;
    for (int it = 0; it < 6; ++it) {
      const double r = duration_stretch(segment(a, b, T));
      if (r <= 1.0) break;
      T *= std::min(std::max(r, 1.05), 3.0);
    }
    return T;
  }

  // ── Edge validation (Alg. 3) ───────────────────────────────────────

  /// Exact bounds on the cubics, then the sampled collision check.
  auto valid(const flat::Segment& seg) -> bool {
    ++edges_checked_;
    for (int f = 0; f < n_; ++f) {
      const flat::Cubic& c = seg.cubics[f];
      const auto [pmin, pmax] = c.pos_range(seg.T);
      const auto [vmin, vmax] = c.vel_range(seg.T);
      const auto [amin, amax] = c.acc_range(seg.T);
      if (pmin < lo_[f] - kTol || pmax > hi_[f] + kTol) return false;
      if (vmin < s_.min_velocity[f] * (1.0 + kTol) ||
          vmax > s_.max_velocity[f] * (1.0 + kTol))
        return false;
      if (amin < s_.min_acceleration[f] * (1.0 + kTol) ||
          amax > s_.max_acceleration[f] * (1.0 + kTol))
        return false;
    }
    if (hx_ >= 0) {
      const flat::Cubic& x = seg.cubics[hx_];
      const flat::Cubic& y = seg.cubics[hy_];
      if (flat::max_planar_speed(x, y, seg.T) >
              s_.max_horizontal_speed * (1.0 + kTol) ||
          flat::max_planar_acceleration(x, y, seg.T) >
              s_.max_horizontal_acceleration * (1.0 + kTol))
        return false;
    }
    return collision_free(seg);
  }

  /// Interleaved SIMD collision check of the edge, sampled at VAMP's
  /// density along its configuration-space arc length: block ``i`` holds
  /// samples ``{i, i + m, i + 2m, ...}`` so early blocks already span the
  /// whole edge (Alg. 3's spatially distributed batches).
  auto collision_free(const flat::Segment& seg) const -> bool {
    const int k = static_cast<int>(std::ceil(seg.T / kArcLengthDt)) + 2;
    std::vector<double> prev(n_), cur(n_);
    double length = 0.0;
    flat::eval_segment(seg, 0.0, prev.data(), nullptr, nullptr);
    for (int i = 1; i < k; ++i) {
      flat::eval_segment(seg, seg.T * i / (k - 1), cur.data(), nullptr,
                         nullptr);
      double d2 = 0.0;
      for (int f = 0; f < n_; ++f)
        d2 += (cur[f] - prev[f]) * (cur[f] - prev[f]);
      length += std::sqrt(d2);
      std::swap(prev, cur);
    }
    const int n =
        static_cast<int>(std::ceil(kCollisionDensity * length *
                                   static_cast<double>(Robot::resolution))) +
        2;
    const int m = (n + static_cast<int>(kRake) - 1) / static_cast<int>(kRake);
    std::vector<double> q(n_);
    alignas(vamp::FloatVectorAlignment)
        std::array<float, Robot::dimension * kRake>
            buf{};
    for (int blk = 0; blk < m; ++blk) {
      for (std::size_t lane = 0; lane < kRake; ++lane) {
        const int idx = std::min(static_cast<int>(lane) * m + blk, n - 1);
        flat::eval_segment(seg, seg.T * idx / (n - 1), q.data(), nullptr,
                           nullptr);
        for (std::size_t d = 0; d < Robot::dimension; ++d)
          buf[d * kRake + lane] = frozen_config_[d];
        for (int f = 0; f < n_; ++f)
          buf[active_indices_[f] * kRake + lane] = static_cast<float>(q[f]);
      }
      typename Robot::template ConfigurationBlock<kRake> block(buf.data());
      const bool ok = env_.attachments
                          ? Robot::template fkcc_attach<kRake>(env_, block)
                          : Robot::template fkcc<kRake>(env_, block);
      if (!ok) return false;
    }
    return true;
  }

  // ── Tree growth ────────────────────────────────────────────────────

  /// Flat state at time ``s`` on the cubic from ``a`` to ``b`` (duration
  /// T); exactly ``a`` / ``b`` at the ends so rest states stay at rest.
  auto state_on_edge(const Node& a, const Node& b, double T, double s) const
      -> Node {
    if (s >= T) return Node{b.y, b.v};
    if (s <= 0.0) return Node{a.y, a.v};
    Node x{std::vector<double>(n_), std::vector<double>(n_)};
    for (int f = 0; f < n_; ++f) {
      const auto c = flat::Cubic::hermite(a.y[f], a.v[f], b.y[f], b.v[f], T);
      x.y[f] = c.pos(s);
      x.v[f] = c.vel(s);
    }
    return x;
  }

  /// Grow ``tree[near]`` toward ``target``: forward in time for the start
  /// tree, backward for the goal tree (the new node precedes
  /// ``tree[near]``).  The node is truncated at ``max_extension_time``
  /// on the optimal cubic, so the edge is a sub-arc of it.
  auto extend(std::vector<Node>& tree, int near, const Node& target,
              bool forward) -> int {
    const Node node = tree[near];
    const Node& a = forward ? node : target;
    const Node& b = forward ? target : node;
    const double T = choose_duration(a, b);
    const double tau = std::min(T, s_.max_extension_time);
    Node x = state_on_edge(a, b, T, forward ? tau : T - tau);
    if (!valid(forward ? segment(node, x, tau) : segment(x, node, tau)))
      return -1;
    x.parent = near;
    x.T = tau;
    tree.push_back(std::move(x));
    return static_cast<int>(tree.size()) - 1;
  }

  /// Connect ``a -> b`` with the closed-form BVP.
  auto connect(const Node& a, const Node& b, double& T) -> bool {
    T = choose_duration(a, b);
    return valid(segment(a, b, T));
  }

  // ── Simplification (after vamp::planning::simplify) ──────────────
  //
  // Same structure as VAMP's geometric simplifier: the path is a list of
  // states and small operations each report whether they changed it —
  // greedy shortcuts (also the paper's Alg. 4) and subdivide-then-cut-
  // corners smoothing — repeated until neither does.  Edges are closed-
  // form cubics, so a change is kept only when it is valid and lowers
  // the LQMT cost.

  void simplify(std::vector<Node>& path,
                std::chrono::steady_clock::time_point ts) {
    for (std::size_t i = 0; i < s_.simplify_iterations; ++i) {
      bool any = shortcut_path(path, ts);
      any |= smooth_path(path, ts);
      if (!any) break;
    }
  }

  auto out_of_time(std::chrono::steady_clock::time_point ts) const -> bool {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - ts)
               .count() > s_.simplify_time_limit;
  }

  /// Connect node i to the farthest node j it reaches directly
  /// (vamp::planning::shortcut_path).
  auto shortcut_path(std::vector<Node>& path,
                     std::chrono::steady_clock::time_point ts) -> bool {
    bool changed = false;
    for (std::size_t i = 0; i + 2 < path.size(); ++i) {
      for (std::size_t j = path.size() - 1; j > i + 1; --j) {
        if (out_of_time(ts)) return changed;
        if (replace(path, i, j)) {
          changed = true;
          break;
        }
      }
    }
    return changed;
  }

  /// Split every edge at its time midpoint, then try to cut each original
  /// interior node by connecting its two neighbours directly
  /// (vamp::planning::smooth_bspline's subdivide + corner cut).
  auto smooth_path(std::vector<Node>& path,
                   std::chrono::steady_clock::time_point ts) -> bool {
    std::vector<Node> split{path.front()};
    std::vector<std::size_t> corners;  // original interior nodes in ``split``
    for (std::size_t k = 1; k < path.size(); ++k) {
      Node b = path[k];
      Node m = state_on_edge(path[k - 1], b, b.T, 0.5 * b.T);
      m.T = 0.5 * b.T;
      split.push_back(std::move(m));
      b.T *= 0.5;
      if (k + 1 < path.size()) corners.push_back(split.size());
      split.push_back(std::move(b));
    }
    path = std::move(split);
    // Walk backwards so a replacement never shifts a corner still to visit.
    bool changed = false;
    for (auto it = corners.rbegin(); it != corners.rend() && !out_of_time(ts);
         ++it)
      changed |= replace(path, *it - 1, *it + 1);
    return changed;
  }

  /// Replace nodes ``i+1 .. j-1`` by a direct connection ``i -> j`` if it
  /// is valid and cheaper than the current edges.
  auto replace(std::vector<Node>& path, std::size_t i, std::size_t j) -> bool {
    double old_cost = 0.0;
    for (std::size_t k = i + 1; k <= j; ++k)
      old_cost += edge_cost(path[k - 1], path[k], path[k].T);
    const double T = choose_duration(path[i], path[j]);
    if (edge_cost(path[i], path[j], T) >= old_cost - 1e-9 ||
        !valid(segment(path[i], path[j], T)))
      return false;
    path[j].T = T;
    path.erase(path.begin() + static_cast<long>(i) + 1,
               path.begin() + static_cast<long>(j));
    return true;
  }
};

}  // namespace drone_planning
