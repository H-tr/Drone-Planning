/**
 * OMPL ↔ VAMP state validity and motion validators for the X500.
 *
 * The full-body robot is 6-DOF: the virtual base joints (x, y, z, yaw)
 * and the gimbal (pan, tilt).  Every joint is a plain bounded real, so
 * the OMPL state is a ``RealVectorStateSpace`` over the active joints.
 * It is expanded to the full body via ``active_indices`` +
 * ``frozen_config`` and the collision check is delegated to VAMP's SIMD
 * ``validate_motion`` (resolution 1 for single states, full robot
 * resolution for motion edges).
 */

#pragma once

#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <algorithm>
#include <array>
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

/// Largest sphere radius ``Robot::fkcc`` queries the environment with.
/// Before a link's spheres, the cricket-generated check tests the link's
/// bounding sphere (the smallest sphere around them); base_link's, 0.2864 m
/// (``y[507]`` in x500.hh), is the largest.  A point cloud's CAPT finds
/// every point within ``r`` of a query only for ``r`` up to the ``r_max``
/// it was built with, so built with ``Robot::max_radius`` (the largest
/// single sphere, 0.0795 m) a bounding-sphere query can miss a point one
/// of the link's spheres touches, and the check passes a colliding
/// configuration.  Point clouds are built with this radius instead.
/// Regenerating x500.hh changes it: take the largest bounding-sphere radius
/// ``y[(n_spheres + i) * 4 + 3]`` of the new header.
inline constexpr float kMaxQueryRadius = 0.286441683769226f;

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
    last_valid.first = nullptr;
    last_valid.second = 0.0;
    return checkMotion(s1, s2);
  }

 private:
  const VampEnv& env_;
  std::vector<int> active_;
  std::vector<float> frozen_;
};

}  // namespace drone_planning
