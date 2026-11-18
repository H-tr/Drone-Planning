/**
 * OMPL + VAMP Python extension — nanobind bindings.
 *
 * The actual planner, validity checkers and the FLASK kinodynamic planner
 * live in self-contained internal headers under this directory.  This
 * file is intentionally kept thin: it only imports those headers and
 * exposes the C++ API to Python via nanobind.  If you find yourself adding
 * more than a few lines of non-binding code here, that's a sign it belongs
 * in one of the internal headers instead.
 */

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>

#include <Eigen/Core>
#include <memory>

#include "planner.hpp"

namespace nb = nanobind;
using drone_planning::KinodynamicResult;
using drone_planning::KinodynamicSettings;
using drone_planning::make_planner;
using drone_planning::OmplVampPlanner;
using drone_planning::PlanResult;
using drone_planning::flat::FlatTrajectory;

namespace {

/// A distance field from NumPy: float32, (X, Y, Z), C order.
using Field =
    nb::ndarray<const float, nb::ndim<3>, nb::c_contig, nb::device::cpu>;

/// The field's values, read in place: the pointer holds a reference to the
/// array and drops it under the GIL.
auto in_place(const Field& field) -> std::shared_ptr<const float> {
  auto* held = new Field(field);
  return std::shared_ptr<const float>(held->data(), [held](const float*) {
    nb::gil_scoped_acquire gil;
    delete held;
  });
}

/// ``OmplVampPlanner.default_joint_limits`` is this descriptor.  On a
/// planner it gives the limits of that planner's robot.  On the class it
/// is the static function it was before there was more than one robot:
/// ``OmplVampPlanner.default_joint_limits()`` gives the X500's limits and
/// ``OmplVampPlanner.default_joint_limits("drone")`` the drone's.
struct DefaultJointLimits {};

auto to_matrix(const FlatTrajectory::Matrix& rows, int ndof)
    -> Eigen::MatrixXd {
  Eigen::MatrixXd out(static_cast<Eigen::Index>(rows.size()), ndof);
  for (std::size_t i = 0; i < rows.size(); ++i)
    for (int j = 0; j < ndof; ++j)
      out(static_cast<Eigen::Index>(i), j) = rows[i][j];
  return out;
}

// Pointwise evaluation of one output (0 = position, 1 = velocity,
// 2 = acceleration) as a numpy vector.
auto eval_one(const FlatTrajectory& traj, double t, int which)
    -> Eigen::VectorXd {
  Eigen::VectorXd out(traj.active_dim());
  double* ptr = out.data();
  traj.evaluate(t, which == 0 ? ptr : nullptr, which == 1 ? ptr : nullptr,
                which == 2 ? ptr : nullptr);
  return out;
}

}  // namespace

NB_MODULE(_ompl_vamp, m) {
  m.doc() =
      "OMPL + VAMP C++ planning extension for the Holybro X500 V2 and "
      "PHANTOM's drone (6 DOF: x, y, z, yaw + gimbal pan, tilt).";

  nb::class_<PlanResult>(m, "PlanResult")
      .def_ro("solved", &PlanResult::solved)
      .def_ro("path", &PlanResult::path)
      .def_ro("planning_time_ns", &PlanResult::planning_time_ns)
      .def_ro("path_cost", &PlanResult::path_cost);

  nb::class_<KinodynamicSettings>(m, "KinodynamicSettings")
      .def(nb::init<>())
      .def_rw("min_velocity", &KinodynamicSettings::min_velocity)
      .def_rw("max_velocity", &KinodynamicSettings::max_velocity)
      .def_rw("min_acceleration", &KinodynamicSettings::min_acceleration)
      .def_rw("max_acceleration", &KinodynamicSettings::max_acceleration)
      .def_rw("max_horizontal_speed",
              &KinodynamicSettings::max_horizontal_speed)
      .def_rw("max_horizontal_acceleration",
              &KinodynamicSettings::max_horizontal_acceleration)
      .def_rw("rho", &KinodynamicSettings::rho)
      .def_rw("limit_aware_duration",
              &KinodynamicSettings::limit_aware_duration)
      .def_rw("max_extension_time", &KinodynamicSettings::max_extension_time)
      .def_rw("velocity_sample_scale",
              &KinodynamicSettings::velocity_sample_scale)
      .def_rw("velocity_metric_weight",
              &KinodynamicSettings::velocity_metric_weight)
      .def_rw("rest_sample_probability",
              &KinodynamicSettings::rest_sample_probability)
      .def_rw("simplify", &KinodynamicSettings::simplify)
      .def_rw("simplify_iterations", &KinodynamicSettings::simplify_iterations)
      .def_rw("simplify_time_limit", &KinodynamicSettings::simplify_time_limit)
      .def_rw("max_iterations", &KinodynamicSettings::max_iterations)
      .def_rw("seed", &KinodynamicSettings::seed);

  nb::class_<FlatTrajectory>(m, "FlatTrajectory")
      .def_prop_ro("duration", &FlatTrajectory::duration)
      .def_prop_ro("active_dim", &FlatTrajectory::active_dim)
      .def_prop_ro("num_segments",
                   [](const FlatTrajectory& t) { return t.segments().size(); })
      .def(
          "position",
          [](const FlatTrajectory& t, double time) {
            return eval_one(t, time, 0);
          },
          nb::arg("t"))
      .def(
          "velocity",
          [](const FlatTrajectory& t, double time) {
            return eval_one(t, time, 1);
          },
          nb::arg("t"))
      .def(
          "acceleration",
          [](const FlatTrajectory& t, double time) {
            return eval_one(t, time, 2);
          },
          nb::arg("t"))
      .def(
          "sample",
          [](const FlatTrajectory& t, const std::vector<double>& times) {
            auto [P, V, A] = t.sample(times);
            const int n = t.active_dim();
            return std::make_tuple(to_matrix(P, n), to_matrix(V, n),
                                   to_matrix(A, n));
          },
          nb::arg("times"))
      .def(
          "sample_uniform",
          [](const FlatTrajectory& t, double dt) {
            auto [times, P, V, A] = t.sample_uniform(dt);
            const int n = t.active_dim();
            Eigen::VectorXd tv = Eigen::Map<const Eigen::VectorXd>(
                times.data(), static_cast<Eigen::Index>(times.size()));
            return std::make_tuple(tv, to_matrix(P, n), to_matrix(V, n),
                                   to_matrix(A, n));
          },
          nb::arg("dt"))
      .def("knot_times", &FlatTrajectory::knot_times);

  nb::class_<KinodynamicResult>(m, "KinodynamicResult")
      .def_ro("solved", &KinodynamicResult::solved)
      .def_ro("trajectory", &KinodynamicResult::trajectory)
      .def_ro("planning_time_ns", &KinodynamicResult::planning_time_ns)
      .def_ro("simplify_time_ns", &KinodynamicResult::simplify_time_ns)
      .def_ro("cost", &KinodynamicResult::cost)
      .def_ro("iterations", &KinodynamicResult::iterations)
      .def_ro("start_tree_size", &KinodynamicResult::start_tree_size)
      .def_ro("goal_tree_size", &KinodynamicResult::goal_tree_size)
      .def_ro("edges_checked", &KinodynamicResult::edges_checked);

  // Closed-form LQMT helper, exposed for tests and diagnostics.
  m.def(
      "flat_optimal_time",
      [](const std::vector<double>& y0, const std::vector<double>& v0,
         const std::vector<double>& y1, const std::vector<double>& v1,
         const std::vector<double>& weights, double rho) {
        drone_planning::flat::LqmtTerms k;
        for (std::size_t i = 0; i < y0.size(); ++i)
          k.add(weights[i], y0[i], v0[i], y1[i], v1[i]);
        const double T = drone_planning::flat::optimal_time(k, rho);
        return std::make_pair(T, k.cost(T, rho));
      },
      nb::arg("y0"), nb::arg("v0"), nb::arg("y1"), nb::arg("v1"),
      nb::arg("weights"), nb::arg("rho"),
      "Minimum-cost duration T* and cost J(T*) of the LQMT (cubic) motion "
      "between two flat states, J(T) = sum_i w_i int y_i''^2 dt + rho T.");

  auto planner_class =
      nb::class_<OmplVampPlanner>(m, "OmplVampPlanner")
          .def(nb::new_([] { return make_planner("x500"); }),
               "Create a full-body X500 planner (6 DOF).")
          .def(nb::new_([](const std::string& robot) {
                 return make_planner(robot);
               }),
               nb::arg("robot"),
               "Create a full-body planner (6 DOF) of ``robot``: \"x500\" or "
               "\"drone\".")
          .def(nb::new_([](std::vector<int> active_indices,
                           std::vector<double> frozen_config) {
                 auto planner = make_planner("x500");
                 planner->set_subgroup(std::move(active_indices),
                                       std::move(frozen_config));
                 return planner;
               }),
               nb::arg("active_indices"), nb::arg("frozen_config"),
               "Create an X500 subgroup planner.")
          .def(nb::new_([](const std::string& robot,
                           std::vector<int> active_indices,
                           std::vector<double> frozen_config) {
                 auto planner = make_planner(robot);
                 planner->set_subgroup(std::move(active_indices),
                                       std::move(frozen_config));
                 return planner;
               }),
               nb::arg("robot"), nb::arg("active_indices"),
               nb::arg("frozen_config"),
               "Create a subgroup planner of ``robot`` over "
               "``active_indices``; the other joints are pinned to "
               "``frozen_config`` (full body).")
          .def("add_pointcloud", &OmplVampPlanner::add_pointcloud,
               nb::arg("points"), nb::arg("point_radius"))
          .def("remove_pointcloud", &OmplVampPlanner::remove_pointcloud)
          .def("has_pointcloud", &OmplVampPlanner::has_pointcloud)
          .def(
              "set_distance_grid",
              [](OmplVampPlanner& planner, const Field& distances,
                 const std::array<float, 3>& origin,
                 const std::array<float, 3>& spacing) {
                planner.set_distance_grid(
                    in_place(distances),
                    {distances.shape(0), distances.shape(1),
                     distances.shape(2)},
                    origin, spacing);
              },
              // No implicit conversion: a converted array would be a copy.
              nb::arg("distances").noconvert(), nb::arg("origin"),
              nb::arg("spacing"),
              "Set the scene distance grid (float32 (X, Y, Z), C order, read "
              "in place), replacing any previous one.")
          .def("remove_distance_grid", &OmplVampPlanner::remove_distance_grid)
          .def("has_distance_grid", &OmplVampPlanner::has_distance_grid)
          .def("add_sphere", &OmplVampPlanner::add_sphere, nb::arg("center"),
               nb::arg("radius"))
          .def("clear_environment", &OmplVampPlanner::clear_environment)
          .def("plan", &OmplVampPlanner::plan, nb::arg("start"),
               nb::arg("goal"), nb::arg("planner_name") = "rrtc",
               nb::arg("time_limit") = 10.0, nb::arg("simplify") = true,
               nb::arg("interpolate") = true, nb::arg("resolution") = 64.0)
          .def("plan_kinodynamic", &OmplVampPlanner::plan_kinodynamic,
               nb::arg("start"), nb::arg("start_velocity"), nb::arg("goal"),
               nb::arg("time_limit"), nb::arg("settings"),
               "FLASK kinodynamic planning in the flat output space.  "
               "Returns a KinodynamicResult whose trajectory is "
               "time-parameterised.")
          .def("simplify_path", &OmplVampPlanner::simplify_path,
               nb::arg("path"), nb::arg("time_limit") = 1.0)
          .def("interpolate_path", &OmplVampPlanner::interpolate_path,
               nb::arg("path"), nb::arg("count") = 0,
               nb::arg("resolution") = 64.0)
          .def("validate", &OmplVampPlanner::validate, nb::arg("config"))
          .def("validate_batch", &OmplVampPlanner::validate_batch,
               nb::arg("configs"))
          .def("dimension", &OmplVampPlanner::dimension)
          .def("lower_bounds", &OmplVampPlanner::lower_bounds)
          .def("upper_bounds", &OmplVampPlanner::upper_bounds)
          .def("joint_limits", &OmplVampPlanner::joint_limits)
          .def("set_joint_limits", &OmplVampPlanner::set_joint_limits,
               nb::arg("lower"), nb::arg("upper"))
          .def("filter_pointcloud", &OmplVampPlanner::filter_pointcloud,
               nb::arg("points"), nb::arg("min_dist"), nb::arg("max_range"),
               nb::arg("origin"), nb::arg("workspace_min"),
               nb::arg("workspace_max"), nb::arg("cull") = true)
          .def("filter_self_from_pointcloud",
               &OmplVampPlanner::filter_self_from_pointcloud, nb::arg("points"),
               nb::arg("point_radius"), nb::arg("config"))
          .def("set_subgroup", &OmplVampPlanner::set_subgroup,
               nb::arg("active_indices"), nb::arg("frozen_config"));

  nb::class_<DefaultJointLimits>(m, "_DefaultJointLimits")
      .def(
          "__get__",
          [](const DefaultJointLimits&, nb::handle planner,
             nb::handle) -> nb::object {
            if (planner.is_none())
              return nb::cpp_function(
                  [](const std::string& robot) {
                    return make_planner(robot)->default_joint_limits();
                  },
                  nb::arg("robot") = "x500", nb::name("default_joint_limits"),
                  "``(lower, upper)`` limits compiled into the model of "
                  "``robot`` (default \"x500\"), full-body order.");
            return nb::cpp_function(
                [self = nb::borrow(planner)] {
                  return nb::cast<const OmplVampPlanner&>(self)
                      .default_joint_limits();
                },
                nb::name("default_joint_limits"),
                "``(lower, upper)`` limits compiled into this planner's "
                "robot model, full-body order.");
          },
          nb::arg("planner").none(), nb::arg("owner").none() = nb::none());
  planner_class.attr("default_joint_limits") = DefaultJointLimits{};
}
