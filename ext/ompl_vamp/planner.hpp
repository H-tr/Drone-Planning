/**
 * Main planner class — OMPL frontend with VAMP collision backend.
 *
 * Two construction modes:
 *
 *  - ``OmplVampPlanner()`` — full body, 6 DOF (x, y, z, yaw, gimbal pan,
 *    gimbal tilt).
 *  - ``OmplVampPlanner(active_indices, frozen_config)`` — subgroup
 *    planning over the listed joint indices, with the rest of the body
 *    pinned to ``frozen_config`` for every collision check.
 *
 * Every X500 joint is a plain bounded real (the base is fully actuated:
 * the flight controller holds roll and pitch), so every subgroup plans in
 * a ``RealVectorStateSpace`` with OMPL's geometric planners, and
 * ``plan_kinodynamic`` runs FLASK (plan_kinodynamic.hpp) over the same
 * joints.  ``add_pointcloud`` / ``add_sphere`` / ``clear_environment``
 * build the obstacle environment and ``set_joint_limits(...)`` overrides
 * the joint ranges the planners sample.  Waypoints and points cross the
 * boundary as row-major Eigen matrices (numpy arrays in Python).
 */

#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>

#include "plan_kinodynamic.hpp"
#include "validity.hpp"
// OMPL — informed trees
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/EITstar.h>
#include <ompl/geometric/planners/lazyinformedtrees/BLITstar.h>
// OMPL — FMT
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/fmt/FMT.h>
// OMPL — KPIECE
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
// OMPL — PRM
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
// OMPL — RRT family
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/STRRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
// OMPL — exploration-based
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/stride/STRIDE.h>

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vamp/collision/filter.hh>
#include <vamp/collision/shapes.hh>
#include <vamp/collision/sphere_sphere.hh>
#include <vector>

namespace drone_planning {

namespace og = ompl::geometric;

/// (N, dof) waypoints and (N, 3) points, row-major so numpy arrays map
/// onto them without per-element conversion.
using Waypoints =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using Points = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;
/// (lower, upper) limits of every joint in full-body order.
using JointLimits = std::pair<Eigen::VectorXd, Eigen::VectorXd>;

struct PlanResult {
  bool solved;
  Waypoints path;
  int64_t planning_time_ns;
  double path_cost;
};

class OmplVampPlanner {
 public:
  /// Full-body constructor (6 DOF).
  OmplVampPlanner()
      : OmplVampPlanner(all_joints(),
                        std::vector<double>(Robot::dimension, 0.0)) {}

  /// Subgroup constructor (reduced DOF).
  OmplVampPlanner(std::vector<int> active_indices,
                  std::vector<double> frozen_config)
      : joint_limits_(default_joint_limits()) {
    set_subgroup(std::move(active_indices), std::move(frozen_config));
  }

  /// Set the scene pointcloud.  The planner holds at most one cloud;
  /// calling this replaces any previously-registered cloud.  The cloud's
  /// CAPT covers queries up to the links' bounding spheres
  /// (``kMaxQueryRadius``, validity.hpp).
  void add_pointcloud(const Eigen::Ref<const Points>& points,
                      float point_radius) {
    float_env_.pointclouds.clear();
    float_env_.pointclouds.emplace_back(to_vamp_points_(points),
                                        Robot::min_radius, kMaxQueryRadius,
                                        point_radius);
    sync_env();
  }

  /// Drop the currently-registered pointcloud.  Returns ``false`` if
  /// there was no cloud to remove.
  bool remove_pointcloud() {
    if (float_env_.pointclouds.empty()) return false;
    float_env_.pointclouds.clear();
    sync_env();
    return true;
  }

  bool has_pointcloud() const { return !float_env_.pointclouds.empty(); }

  void add_sphere(const std::array<float, 3>& center, float radius) {
    float_env_.spheres.emplace_back(vamp::collision::Sphere<float>{
        center[0], center[1], center[2], radius});
    float_env_.sort();
    sync_env();
  }

  void clear_environment() {
    float_env_ = FloatEnv{};
    env_ = VampEnv{};
  }

  auto plan(const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
            const std::string& planner_name, double time_limit, bool simplify,
            bool interpolate, int interpolate_count, double resolution)
      -> PlanResult {
    check_dim_(start.size(), "plan: start");
    check_dim_(goal.size(), "plan: goal");
    auto si = make_space_information_();
    og::SimpleSetup ss(si);
    ss.setPlanner(create_planner(si, planner_name));

    ob::ScopedState<> ompl_start(space_);
    ob::ScopedState<> ompl_goal(space_);
    for (int i = 0; i < active_dim_; ++i) {
      ompl_start[i] = start[i];
      ompl_goal[i] = goal[i];
    }
    ss.setStartAndGoalStates(ompl_start, ompl_goal);

    auto t0 = std::chrono::steady_clock::now();
    auto status = ss.solve(time_limit);
    auto t1 = std::chrono::steady_clock::now();

    PlanResult result;
    result.planning_time_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
    // Approximate solutions stop short of the goal: not solved.
    result.solved = status == ob::PlannerStatus::EXACT_SOLUTION;
    if (!result.solved) {
      result.path = Waypoints(0, active_dim_);
      result.path_cost = std::numeric_limits<double>::infinity();
      return result;
    }
    if (simplify) ss.simplifySolution();
    auto& path = ss.getSolutionPath();
    if (interpolate) densify(path, interpolate_count, resolution);
    result.path_cost = path.length();
    result.path = path_to_waypoints_(path);
    return result;
  }

  // ── Kinodynamic planning (FLASK) ──────────────────────────────────
  //
  // Plans a time-parameterised trajectory in the flat output space (see
  // plan_kinodynamic.hpp): closed-form cubic edges, SIMD-batched
  // collision checks, velocity / acceleration limits enforced along every
  // edge.  ``start_velocity`` is either empty (start at rest) or an
  // active-DOF velocity; the goal is always reached at rest.
  auto plan_kinodynamic(const Eigen::VectorXd& start,
                        const Eigen::VectorXd& start_velocity,
                        const Eigen::VectorXd& goal, double time_limit,
                        const KinodynamicSettings& settings)
      -> KinodynamicResult {
    check_dim_(start.size(), "plan_kinodynamic: start");
    check_dim_(goal.size(), "plan_kinodynamic: goal");
    if (start_velocity.size() != 0)
      check_dim_(start_velocity.size(), "plan_kinodynamic: start_velocity");
    for (const auto* limits :
         {&settings.min_velocity, &settings.max_velocity,
          &settings.min_acceleration, &settings.max_acceleration})
      check_dim_(static_cast<Eigen::Index>(limits->size()),
                 "plan_kinodynamic: settings limits");
    const auto lower = lower_bounds(), upper = upper_bounds();
    FlaskPlanner flask(env_, active_indices_, frozen_config_,
                       {lower.data(), lower.data() + lower.size()},
                       {upper.data(), upper.data() + upper.size()}, settings);
    return flask.plan(
        {start.data(), start.data() + start.size()},
        {start_velocity.data(), start_velocity.data() + start_velocity.size()},
        {goal.data(), goal.data() + goal.size()}, time_limit);
  }

  // Standalone path simplification — same pipeline OMPL's
  // ``SimpleSetup::simplifySolution`` runs (reduceVertices,
  // collapseCloseVertices, shortcutPath, B-spline smoothing) against the
  // current collision environment, but detached from ``plan(...)``.
  auto simplify_path(const Waypoints& path, double time_limit) -> Waypoints {
    if (path.rows() < 2) return path;
    auto si = make_space_information_();
    og::PathGeometric geo = waypoints_to_path_(path, si);
    og::PathSimplifier simp(si);
    simp.simplify(geo, time_limit);
    return path_to_waypoints_(geo);
  }

  // Standalone path densification — same three modes as
  // ``plan(..., interpolate=True, ...)``:
  //   * ``count > 0``        → exactly that many total waypoints.
  //   * ``resolution > 0.0`` → ``ceil(edge_length * resolution)`` waypoints
  //                            per edge (uniform density in state-space
  //                            distance).
  //   * both 0               → OMPL's default longest-valid-segment
  //                            fraction.
  // No collision check — densification only calls
  // ``StateSpace::interpolate`` on the existing edges.
  auto interpolate_path(const Waypoints& path, int count, double resolution)
      -> Waypoints {
    if (count > 0 && resolution > 0.0) {
      throw std::invalid_argument(
          "interpolate_path: pass at most one of count (>0) or resolution "
          "(>0), not both.");
    }
    if (path.rows() < 2) return path;
    auto si = make_space_information_();
    og::PathGeometric geo = waypoints_to_path_(path, si);
    densify(geo, count, resolution);
    return path_to_waypoints_(geo);
  }

  auto validate(const Eigen::VectorXd& config) -> bool {
    check_dim_(config.size(), "validate: config");
    auto q = build_full_config_(config);
    return vamp::planning::validate_motion<Robot, kRake, 1>(q, q, env_);
  }

  // Batched collision check — packs up to ``kRake`` distinct
  // configurations directly into a VAMP ``ConfigurationBlock<kRake>``
  // so a single ``Robot::fkcc<kRake>`` call sphere-FKs and
  // collision-checks them against the SIMD environment in one sweep.
  // If a packed block fails, its lanes are re-checked one by one so the
  // caller still gets one bool per input config.
  auto validate_batch(const Eigen::Ref<const Waypoints>& configs)
      -> Eigen::Matrix<bool, Eigen::Dynamic, 1> {
    const auto n = static_cast<std::size_t>(configs.rows());
    Eigen::Matrix<bool, Eigen::Dynamic, 1> result =
        Eigen::Matrix<bool, Eigen::Dynamic, 1>::Zero(configs.rows());
    if (n == 0) return result;
    check_dim_(configs.cols(), "validate_batch: configs");

    // ``ConfigurationBlock<kRake>`` is row-major over joints: row ``d``
    // holds the kRake lane values of joint ``d``, so
    //     buf[d * kRake + lane] = full_config(configs[lane])[d].
    alignas(vamp::FloatVectorAlignment)
        std::array<float, Robot::dimension * kRake>
            blk_buf{};

    auto write_lane = [&](std::size_t lane, std::size_t row) {
      const auto cfg = configs.row(static_cast<Eigen::Index>(row));
      for (std::size_t d = 0; d < Robot::dimension; ++d)
        blk_buf[d * kRake + lane] = frozen_config_[d];
      for (std::size_t k = 0; k < active_indices_.size(); ++k)
        blk_buf[active_indices_[k] * kRake + lane] = static_cast<float>(cfg[k]);
    };

    for (std::size_t i = 0; i < n; i += kRake) {
      const std::size_t chunk = std::min(kRake, n - i);
      for (std::size_t lane = 0; lane < chunk; ++lane)
        write_lane(lane, i + lane);
      // Pad the tail lanes with the first real config of the block: if
      // the block passes every real lane is valid, and padded lanes never
      // reach the output.
      for (std::size_t lane = chunk; lane < kRake; ++lane) write_lane(lane, i);

      typename Robot::template ConfigurationBlock<kRake> block(blk_buf.data());
      const bool block_valid =
          env_.attachments ? Robot::template fkcc_attach<kRake>(env_, block)
                           : Robot::template fkcc<kRake>(env_, block);

      for (std::size_t lane = 0; lane < chunk; ++lane) {
        if (block_valid) {
          result[i + lane] = true;
          continue;
        }
        auto q = build_full_config_(
            configs.row(static_cast<Eigen::Index>(i + lane)).transpose());
        result[i + lane] =
            vamp::planning::validate_motion<Robot, kRake, 1>(q, q, env_);
      }
    }
    return result;
  }

  // ── Point cloud filtering ───────────────────────────────────────

  /// Spatial down-sampling via Morton-curve sorting.
  auto filter_pointcloud(const Eigen::Ref<const Points>& points, float min_dist,
                         float max_range, const std::array<float, 3>& origin,
                         const std::array<float, 3>& workspace_min,
                         const std::array<float, 3>& workspace_max, bool cull)
      -> Points {
    vamp::collision::Point o{origin[0], origin[1], origin[2]};
    vamp::collision::Point ws_min{workspace_min[0], workspace_min[1],
                                  workspace_min[2]};
    vamp::collision::Point ws_max{workspace_max[0], workspace_max[1],
                                  workspace_max[2]};

    auto filtered = vamp::collision::filter_pointcloud(
        to_vamp_points_(points), min_dist, max_range, o, ws_min, ws_max, cull);

    Points out(static_cast<Eigen::Index>(filtered.size()), 3);
    for (std::size_t i = 0; i < filtered.size(); ++i)
      out.row(static_cast<Eigen::Index>(i)) << filtered[i][0], filtered[i][1],
          filtered[i][2];
    return out;
  }

  /// Remove points that collide with the robot body or the environment.
  auto filter_self_from_pointcloud(const Eigen::Ref<const Points>& points,
                                   float point_radius,
                                   const Eigen::VectorXd& config) -> Points {
    check_dim_(config.size(), "filter_self_from_pointcloud: config");

    auto full_arr = build_full_config_(config).to_array();
    typename Robot::template ConfigurationBlock<1> block;
    for (std::size_t i = 0; i < Robot::dimension; ++i) block[i] = full_arr[i];

    typename Robot::template Spheres<1> spheres;
    Robot::template sphere_fk<1>(block, spheres);

    std::vector<Eigen::Index> keep;
    keep.reserve(static_cast<std::size_t>(points.rows()));
    for (Eigen::Index p = 0; p < points.rows(); ++p) {
      const float x = points(p, 0), y = points(p, 1), z = points(p, 2),
                  r = point_radius;
      bool valid = true;
      for (std::size_t i = 0; i < Robot::n_spheres; ++i) {
        if (vamp::collision::sphere_sphere_sql2(
                spheres.x[{i, 0}], spheres.y[{i, 0}], spheres.z[{i, 0}],
                spheres.r[{i, 0}], x, y, z, r) < 0 ||
            vamp::sphere_environment_in_collision(env_, x, y, z, r)) {
          valid = false;
          break;
        }
      }
      if (valid) keep.push_back(p);
    }
    Points out(static_cast<Eigen::Index>(keep.size()), 3);
    for (std::size_t i = 0; i < keep.size(); ++i)
      out.row(static_cast<Eigen::Index>(i)) = points.row(keep[i]);
    return out;
  }

  auto dimension() const -> int { return active_dim_; }

  auto lower_bounds() const -> Eigen::VectorXd {
    const auto& low = space_->as<ob::RealVectorStateSpace>()->getBounds().low;
    return Eigen::Map<const Eigen::VectorXd>(low.data(), active_dim_);
  }

  auto upper_bounds() const -> Eigen::VectorXd {
    const auto& high = space_->as<ob::RealVectorStateSpace>()->getBounds().high;
    return Eigen::Map<const Eigen::VectorXd>(high.data(), active_dim_);
  }

  // ── Joint limits ──────────────────────────────────────────────────
  //
  // The planner keeps one (lower, upper) pair per joint of the full
  // body.  Every active state space takes its bounds from it, so limits
  // set here survive set_subgroup().  They shape where the planners
  // sample and which states they accept; collision checking never reads
  // them.

  /// The limits compiled into the robot model, in full-body order.
  static auto default_joint_limits() -> JointLimits {
    std::array<float, Robot::dimension> zeros{}, ones{};
    ones.fill(1.0f);
    Robot::Configuration lo(zeros.data()), hi(ones.data());
    Robot::scale_configuration(lo);
    Robot::scale_configuration(hi);
    const auto lo_arr = lo.to_array();
    const auto hi_arr = hi.to_array();
    JointLimits limits{Eigen::VectorXd(Robot::dimension),
                       Eigen::VectorXd(Robot::dimension)};
    for (std::size_t i = 0; i < Robot::dimension; ++i) {
      const auto k = static_cast<Eigen::Index>(i);
      limits.first[k] = std::min(lo_arr[i], hi_arr[i]);
      limits.second[k] = std::max(lo_arr[i], hi_arr[i]);
    }
    return limits;
  }

  /// The limits in effect, in full-body order.
  auto joint_limits() const -> JointLimits { return joint_limits_; }

  /// Replace the limits of every joint (full-body order).  Each joint
  /// needs finite bounds with lower < upper; on error nothing changes.
  void set_joint_limits(const Eigen::VectorXd& lower,
                        const Eigen::VectorXd& upper) {
    if (lower.size() != Robot::dimension || upper.size() != Robot::dimension)
      throw std::invalid_argument("set_joint_limits: lower and upper need " +
                                  std::to_string(Robot::dimension) +
                                  " entries, got " +
                                  std::to_string(lower.size()) + " and " +
                                  std::to_string(upper.size()) + ".");
    for (Eigen::Index i = 0; i < lower.size(); ++i) {
      if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) ||
          !(lower[i] < upper[i]))
        throw std::invalid_argument(
            "set_joint_limits: joint " + std::to_string(i) +
            " needs finite bounds with lower < upper, got [" +
            std::to_string(lower[i]) + ", " + std::to_string(upper[i]) + "].");
    }
    joint_limits_ = {lower, upper};
    rebuild_space_();
  }

  /// Switch active joints without rebuilding the collision environment.
  ///
  /// @param active_indices  Joint indices into the full-body config.
  /// @param frozen_config   Full-body stance for joints outside
  ///     ``active_indices``.
  void set_subgroup(std::vector<int> active_indices,
                    std::vector<double> frozen_config) {
    for (int index : active_indices)
      if (index < 0 || index >= static_cast<int>(Robot::dimension))
        throw std::invalid_argument("set_subgroup: joint index " +
                                    std::to_string(index) + " out of range.");
    if (frozen_config.size() != Robot::dimension)
      throw std::invalid_argument("set_subgroup: frozen_config needs " +
                                  std::to_string(Robot::dimension) +
                                  " entries, got " +
                                  std::to_string(frozen_config.size()) + ".");
    active_dim_ = static_cast<int>(active_indices.size());
    active_indices_ = std::move(active_indices);
    frozen_config_.assign(frozen_config.begin(), frozen_config.end());
    rebuild_space_();
  }

 private:
  int active_dim_ = 0;
  std::vector<int> active_indices_;
  std::vector<float> frozen_config_;
  JointLimits joint_limits_;
  ob::StateSpacePtr space_;
  FloatEnv float_env_;
  VampEnv env_;

  static auto all_joints() -> std::vector<int> {
    std::vector<int> joints(Robot::dimension);
    std::iota(joints.begin(), joints.end(), 0);
    return joints;
  }

  void sync_env() { env_ = VampEnv(float_env_); }

  void check_dim_(Eigen::Index n, const char* what) const {
    if (n != active_dim_)
      throw std::invalid_argument(std::string(what) + " has " +
                                  std::to_string(n) + " entries, expected " +
                                  std::to_string(active_dim_) + ".");
  }

  static auto to_vamp_points_(const Eigen::Ref<const Points>& points)
      -> std::vector<vamp::collision::Point> {
    std::vector<vamp::collision::Point> out(
        static_cast<std::size_t>(points.rows()));
    for (Eigen::Index i = 0; i < points.rows(); ++i)
      out[static_cast<std::size_t>(i)] = {points(i, 0), points(i, 1),
                                          points(i, 2)};
    return out;
  }

  // Expand an active-DOF config into a full-body VAMP Configuration,
  // injecting the frozen pose for joints outside ``active_indices_``.
  auto build_full_config_(const Eigen::VectorXd& config) const
      -> Robot::Configuration {
    alignas(Robot::Configuration::S::Alignment)
        std::array<float, Robot::Configuration::num_scalars_rounded>
            buf{};
    std::copy(frozen_config_.begin(), frozen_config_.end(), buf.begin());
    for (std::size_t i = 0; i < active_indices_.size(); ++i)
      buf[active_indices_[i]] = static_cast<float>(config[i]);
    return Robot::Configuration(buf.data());
  }

  // Rebuild the active state space with the stored joint limits of the
  // active joints.
  void rebuild_space_() {
    auto space = std::make_shared<ob::RealVectorStateSpace>(active_dim_);
    ob::RealVectorBounds bounds(active_dim_);
    for (int i = 0; i < active_dim_; ++i) {
      bounds.setLow(i, joint_limits_.first[active_indices_[i]]);
      bounds.setHigh(i, joint_limits_.second[active_indices_[i]]);
    }
    space->setBounds(bounds);
    space_ = space;
  }

  // A ``SpaceInformation`` with our validity / motion checkers, shared by
  // ``plan`` and the standalone simplify / interpolate.
  auto make_space_information_() -> ob::SpaceInformationPtr {
    auto si = std::make_shared<ob::SpaceInformation>(space_);
    si->setStateValidityChecker(std::make_shared<SubgroupValidityChecker>(
        si, env_, active_indices_, frozen_config_));
    si->setMotionValidator(std::make_shared<SubgroupMotionValidator>(
        si, env_, active_indices_, frozen_config_));
    si->setup();
    return si;
  }

  auto waypoints_to_path_(const Waypoints& waypoints,
                          const ob::SpaceInformationPtr& si)
      -> og::PathGeometric {
    check_dim_(waypoints.cols(), "path waypoints");
    og::PathGeometric path(si);
    ob::ScopedState<> state(space_);
    for (Eigen::Index i = 0; i < waypoints.rows(); ++i) {
      for (int j = 0; j < active_dim_; ++j) state[j] = waypoints(i, j);
      path.append(state.get());
    }
    return path;
  }

  auto path_to_waypoints_(const og::PathGeometric& path) -> Waypoints {
    Waypoints out(static_cast<Eigen::Index>(path.getStateCount()), active_dim_);
    for (std::size_t i = 0; i < path.getStateCount(); ++i) {
      const auto* rv =
          path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
      for (int j = 0; j < active_dim_; ++j)
        out(static_cast<Eigen::Index>(i), j) = rv->values[j];
    }
    return out;
  }

  // Densify ``path``: ``count > 0`` total waypoints, else each edge of
  // state-space length ``d`` split into ``ceil(d * resolution)`` equal
  // segments, else OMPL's default longest-valid-segment fraction.
  void densify(og::PathGeometric& path, int count, double resolution) const {
    if (count > 0) {
      path.interpolate(static_cast<unsigned int>(count));
      return;
    }
    if (resolution <= 0.0) {
      path.interpolate();
      return;
    }
    auto& states = path.getStates();
    if (states.size() < 2) return;
    const auto& si = path.getSpaceInformation();
    std::vector<ob::State*> dense{states.front()};
    for (std::size_t i = 1; i < states.size(); ++i) {
      const double d = si->distance(states[i - 1], states[i]);
      const int n = std::max(1, static_cast<int>(std::ceil(d * resolution)));
      for (int k = 1; k < n; ++k) {
        auto* s = si->allocState();
        si->getStateSpace()->interpolate(states[i - 1], states[i],
                                         static_cast<double>(k) / n, s);
        dense.push_back(s);
      }
      dense.push_back(states[i]);
    }
    states = std::move(dense);
  }

  static auto create_planner(const ob::SpaceInformationPtr& si,
                             const std::string& name) -> ob::PlannerPtr {
    // RRT family
    if (name == "rrtc" || name == "rrtconnect")
      return std::make_shared<og::RRTConnect>(si);
    if (name == "rrt") return std::make_shared<og::RRT>(si);
    if (name == "rrtstar") return std::make_shared<og::RRTstar>(si);
    if (name == "informed_rrtstar")
      return std::make_shared<og::InformedRRTstar>(si);
    if (name == "rrtsharp") return std::make_shared<og::RRTsharp>(si);
    if (name == "rrtxstatic") return std::make_shared<og::RRTXstatic>(si);
    if (name == "strrtstar") return std::make_shared<og::STRRTstar>(si);
    if (name == "lbtrrt") return std::make_shared<og::LBTRRT>(si);
    if (name == "trrt") return std::make_shared<og::TRRT>(si);
    if (name == "bitrrt") return std::make_shared<og::BiTRRT>(si);
    // Informed trees (asymptotically optimal)
    if (name == "bitstar") return std::make_shared<og::BITstar>(si);
    if (name == "abitstar") return std::make_shared<og::ABITstar>(si);
    if (name == "aitstar") return std::make_shared<og::AITstar>(si);
    if (name == "eitstar") return std::make_shared<og::EITstar>(si);
    if (name == "blitstar") return std::make_shared<og::BLITstar>(si);
    // FMT
    if (name == "fmt") return std::make_shared<og::FMT>(si);
    if (name == "bfmt") return std::make_shared<og::BFMT>(si);
    // KPIECE
    if (name == "kpiece") return std::make_shared<og::KPIECE1>(si);
    if (name == "bkpiece") return std::make_shared<og::BKPIECE1>(si);
    if (name == "lbkpiece") return std::make_shared<og::LBKPIECE1>(si);
    // PRM family
    if (name == "prm") return std::make_shared<og::PRM>(si);
    if (name == "prmstar") return std::make_shared<og::PRMstar>(si);
    if (name == "lazyprm") return std::make_shared<og::LazyPRM>(si);
    if (name == "lazyprmstar") return std::make_shared<og::LazyPRMstar>(si);
    if (name == "spars") return std::make_shared<og::SPARS>(si);
    if (name == "spars2") return std::make_shared<og::SPARStwo>(si);
    // Exploration-based
    if (name == "est") return std::make_shared<og::EST>(si);
    if (name == "biest") return std::make_shared<og::BiEST>(si);
    if (name == "sbl") return std::make_shared<og::SBL>(si);
    if (name == "stride") return std::make_shared<og::STRIDE>(si);
    if (name == "pdst") return std::make_shared<og::PDST>(si);
    throw std::invalid_argument("Unknown planner: " + name);
  }
};

}  // namespace drone_planning
