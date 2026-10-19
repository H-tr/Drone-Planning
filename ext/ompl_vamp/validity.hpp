/**
 * OMPL ↔ VAMP state validity and motion validators for the X500.
 *
 * The full-body robot is 6-DOF: the virtual base joints (x, y, z, yaw)
 * and the gimbal (pan, tilt).  Every joint is a plain bounded real, so
 * the OMPL state is a ``RealVectorStateSpace`` over the active joints.
 * It is expanded to the full body via ``active_indices`` +
 * ``frozen_config`` and the collision check is delegated to VAMP's SIMD
 * ``validate_motion`` (resolution 1 for single states, full robot
 * resolution for motion edges).  For a failed edge,
 * ``checkMotion(s1, s2, last_valid)`` also reports the last valid state
 * along it, which planners such as PDST, KPIECE and STRIDE grow from.
 */

#pragma once

#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>
#include <vamp/planning/validate.hh>
#include <vamp/robots/x500.hh>
#include <vector>

namespace drone_planning {

namespace ob = ompl::base;

using Robot = vamp::robots::X500;
inline constexpr std::size_t kRake = vamp::FloatVectorWidth;
using VampEnv = vamp::collision::Environment<vamp::FloatVector<kRake>>;
using FloatEnv = vamp::collision::Environment<float>;

/// Full-body VAMP configuration from an active-DOF OMPL state.
inline auto expand_state(const ob::State* state, const std::vector<int>& active,
                         const std::vector<float>& frozen)
    -> Robot::Configuration {
  alignas(Robot::Configuration::S::Alignment)
      std::array<float, Robot::Configuration::num_scalars_rounded>
          buf{};
  std::copy(frozen.begin(), frozen.end(), buf.begin());
  const auto* rv = state->as<ob::RealVectorStateSpace::StateType>();
  for (std::size_t i = 0; i < active.size(); ++i)
    buf[active[i]] = static_cast<float>(rv->values[i]);
  return Robot::Configuration(buf.data());
}

// For a motion that failed its check: walks the states VAMP's
// ``validate_motion`` samples on the straight line (``t = k / n``,
// ``k = 1..n``) in order and reports the last valid one, as OMPL's
// ``checkMotion(s1, s2, last_valid)`` contract asks: its fraction in
// ``last_valid.second`` and, when ``last_valid.first`` is given, the state.
inline void find_last_valid(const ob::SpaceInformation* si, const ob::State* s1,
                            const ob::State* s2,
                            std::pair<ob::State*, double>& last_valid) {
  const auto& space = si->getStateSpace();
  const double steps =
      std::ceil(space->distance(s1, s2) / kRake * Robot::resolution);
  const int n = static_cast<int>(kRake * std::max(steps, 1.0));
  auto* s = space->allocState();
  int k = 1;
  for (; k <= n; ++k) {
    space->interpolate(s1, s2, static_cast<double>(k) / n, s);
    if (!si->isValid(s)) break;
  }
  space->freeState(s);
  last_valid.second = static_cast<double>(k - 1) / n;
  if (last_valid.first)
    space->interpolate(s1, s2, last_valid.second, last_valid.first);
}

class SubgroupValidityChecker : public ob::StateValidityChecker {
 public:
  SubgroupValidityChecker(const ob::SpaceInformationPtr& si, const VampEnv& env,
                          std::vector<int> active_indices,
                          std::vector<float> frozen_config)
      : ob::StateValidityChecker(si),
        env_(env),
        active_(std::move(active_indices)),
        frozen_(std::move(frozen_config)) {}

  auto isValid(const ob::State* state) const -> bool override {
    auto config = expand_state(state, active_, frozen_);
    return vamp::planning::validate_motion<Robot, kRake, 1>(config, config,
                                                            env_);
  }

 private:
  const VampEnv& env_;
  std::vector<int> active_;
  std::vector<float> frozen_;
};

class SubgroupMotionValidator : public ob::MotionValidator {
 public:
  SubgroupMotionValidator(const ob::SpaceInformationPtr& si, const VampEnv& env,
                          std::vector<int> active_indices,
                          std::vector<float> frozen_config)
      : ob::MotionValidator(si),
        env_(env),
        active_(std::move(active_indices)),
        frozen_(std::move(frozen_config)) {}

  auto checkMotion(const ob::State* s1, const ob::State* s2) const
      -> bool override {
    return vamp::planning::validate_motion<Robot, kRake, Robot::resolution>(
        expand_state(s1, active_, frozen_), expand_state(s2, active_, frozen_),
        env_);
  }

  auto checkMotion(const ob::State* s1, const ob::State* s2,
                   std::pair<ob::State*, double>& last_valid) const
      -> bool override {
    if (checkMotion(s1, s2)) return true;
    find_last_valid(si_, s1, s2, last_valid);
    return false;
  }

 private:
  const VampEnv& env_;
  std::vector<int> active_;
  std::vector<float> frozen_;
};

}  // namespace drone_planning
