#include "centraltd/centralizer.hpp"
#include "centraltd/solver_stub.hpp"
#include "centraltd/string_section.hpp"
#include "centraltd/version.hpp"
#include "centraltd/well.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(_core, m) {
  m.doc() = "centraltd C++ core";
  m.attr("__version__") = centraltd::version_string();

  m.def("version_string", &centraltd::version_string);

  py::class_<centraltd::WellTrajectoryPoint>(m, "WellTrajectoryPoint")
      .def(py::init<>())
      .def_readwrite("measured_depth_m", &centraltd::WellTrajectoryPoint::measured_depth_m)
      .def_readwrite("inclination_rad", &centraltd::WellTrajectoryPoint::inclination_rad)
      .def_readwrite("azimuth_rad", &centraltd::WellTrajectoryPoint::azimuth_rad)
      .def_readwrite("tvd_m", &centraltd::WellTrajectoryPoint::tvd_m)
      .def_readwrite("northing_m", &centraltd::WellTrajectoryPoint::northing_m)
      .def_readwrite("easting_m", &centraltd::WellTrajectoryPoint::easting_m);

  py::class_<centraltd::TrajectoryGeometryNode>(m, "TrajectoryGeometryNode")
      .def(py::init<>())
      .def_readwrite("measured_depth_m", &centraltd::TrajectoryGeometryNode::measured_depth_m)
      .def_readwrite("inclination_rad", &centraltd::TrajectoryGeometryNode::inclination_rad)
      .def_readwrite("azimuth_rad", &centraltd::TrajectoryGeometryNode::azimuth_rad)
      .def_readwrite("tvd_m", &centraltd::TrajectoryGeometryNode::tvd_m)
      .def_readwrite("northing_m", &centraltd::TrajectoryGeometryNode::northing_m)
      .def_readwrite("easting_m", &centraltd::TrajectoryGeometryNode::easting_m)
      .def_readwrite(
          "tangent_north_east_tvd",
          &centraltd::TrajectoryGeometryNode::tangent_north_east_tvd)
      .def_readwrite(
          "discrete_curvature_rad_per_m",
          &centraltd::TrajectoryGeometryNode::discrete_curvature_rad_per_m);

  py::class_<centraltd::TrajectorySummary>(m, "TrajectorySummary")
      .def(py::init<>())
      .def_readwrite("point_count", &centraltd::TrajectorySummary::point_count)
      .def_readwrite(
          "start_measured_depth_m",
          &centraltd::TrajectorySummary::start_measured_depth_m)
      .def_readwrite(
          "final_measured_depth_m",
          &centraltd::TrajectorySummary::final_measured_depth_m)
      .def_readwrite(
          "total_course_length_m",
          &centraltd::TrajectorySummary::total_course_length_m)
      .def_readwrite("vertical_depth_m", &centraltd::TrajectorySummary::vertical_depth_m)
      .def_readwrite(
          "lateral_displacement_m",
          &centraltd::TrajectorySummary::lateral_displacement_m)
      .def_readwrite(
          "max_inclination_rad",
          &centraltd::TrajectorySummary::max_inclination_rad)
      .def_readwrite(
          "max_curvature_rad_per_m",
          &centraltd::TrajectorySummary::max_curvature_rad_per_m)
      .def_readwrite(
          "coordinates_are_approximate",
          &centraltd::TrajectorySummary::coordinates_are_approximate);

  py::class_<centraltd::WellTrajectory>(m, "WellTrajectory")
      .def(py::init<>())
      .def(py::init<std::vector<centraltd::WellTrajectoryPoint>>(), py::arg("points"))
      .def("validate", &centraltd::WellTrajectory::validate)
      .def("points", [](const centraltd::WellTrajectory& well) { return well.points(); })
      .def("size", &centraltd::WellTrajectory::size)
      .def("empty", &centraltd::WellTrajectory::empty)
      .def("final_measured_depth_m", &centraltd::WellTrajectory::final_measured_depth_m)
      .def("local_tangent", &centraltd::WellTrajectory::local_tangent, py::arg("point_index"))
      .def(
          "discrete_curvature_rad_per_m",
          &centraltd::WellTrajectory::discrete_curvature_rad_per_m,
          py::arg("point_index"))
      .def("derived_geometry", &centraltd::WellTrajectory::derived_geometry)
      .def("interpolate", &centraltd::WellTrajectory::interpolate, py::arg("measured_depth_m"))
      .def("summary", &centraltd::WellTrajectory::summary);

  py::class_<centraltd::StringSection>(m, "StringSection")
      .def(py::init<>())
      .def_readwrite("name", &centraltd::StringSection::name)
      .def_readwrite("md_start_m", &centraltd::StringSection::md_start_m)
      .def_readwrite("md_end_m", &centraltd::StringSection::md_end_m)
      .def_readwrite("outer_diameter_m", &centraltd::StringSection::outer_diameter_m)
      .def_readwrite("inner_diameter_m", &centraltd::StringSection::inner_diameter_m)
      .def_readwrite(
          "linear_weight_n_per_m",
          &centraltd::StringSection::linear_weight_n_per_m)
      .def_readwrite("young_modulus_pa", &centraltd::StringSection::young_modulus_pa)
      .def_readwrite("shear_modulus_pa", &centraltd::StringSection::shear_modulus_pa)
      .def_readwrite("density_kg_per_m3", &centraltd::StringSection::density_kg_per_m3)
      .def_readwrite(
          "friction_coefficient",
          &centraltd::StringSection::friction_coefficient)
      .def("length_m", &centraltd::StringSection::length_m)
      .def("contains_md", &centraltd::StringSection::contains_md, py::arg("measured_depth_m"))
      .def("validate", &centraltd::StringSection::validate);

  py::class_<centraltd::CentralizerSpec>(m, "CentralizerSpec")
      .def(py::init<>())
      .def_readwrite("name", &centraltd::CentralizerSpec::name)
      .def_readwrite("type", &centraltd::CentralizerSpec::type)
      .def_readwrite("outer_diameter_m", &centraltd::CentralizerSpec::outer_diameter_m)
      .def_readwrite(
          "nominal_restoring_force_n",
          &centraltd::CentralizerSpec::nominal_restoring_force_n)
      .def_readwrite(
          "nominal_running_force_n",
          &centraltd::CentralizerSpec::nominal_running_force_n)
      .def_readwrite("spacing_hint_m", &centraltd::CentralizerSpec::spacing_hint_m)
      .def_readwrite("count_hint", &centraltd::CentralizerSpec::count_hint)
      .def_readwrite("installation_md_m", &centraltd::CentralizerSpec::installation_md_m)
      .def(
          "explicit_installation_count",
          &centraltd::CentralizerSpec::explicit_installation_count)
      .def("validate", &centraltd::CentralizerSpec::validate);

  py::class_<centraltd::StringSectionSummary>(m, "StringSectionSummary")
      .def(py::init<>())
      .def_readwrite("name", &centraltd::StringSectionSummary::name)
      .def_readwrite("md_start_m", &centraltd::StringSectionSummary::md_start_m)
      .def_readwrite("md_end_m", &centraltd::StringSectionSummary::md_end_m)
      .def_readwrite("length_m", &centraltd::StringSectionSummary::length_m)
      .def_readwrite(
          "outer_diameter_m",
          &centraltd::StringSectionSummary::outer_diameter_m)
      .def_readwrite(
          "inner_diameter_m",
          &centraltd::StringSectionSummary::inner_diameter_m)
      .def_readwrite(
          "linear_weight_n_per_m",
          &centraltd::StringSectionSummary::linear_weight_n_per_m)
      .def_readwrite(
          "friction_coefficient",
          &centraltd::StringSectionSummary::friction_coefficient)
      .def_readwrite(
          "nominal_radial_clearance_m",
          &centraltd::StringSectionSummary::nominal_radial_clearance_m);

  py::class_<centraltd::StringSummary>(m, "StringSummary")
      .def(py::init<>())
      .def_readwrite("section_count", &centraltd::StringSummary::section_count)
      .def_readwrite("total_length_m", &centraltd::StringSummary::total_length_m)
      .def_readwrite("total_weight_n", &centraltd::StringSummary::total_weight_n)
      .def_readwrite("max_outer_diameter_m", &centraltd::StringSummary::max_outer_diameter_m)
      .def_readwrite("min_inner_diameter_m", &centraltd::StringSummary::min_inner_diameter_m)
      .def_readwrite(
          "average_friction_coefficient",
          &centraltd::StringSummary::average_friction_coefficient)
      .def_readwrite(
          "average_density_kg_per_m3",
          &centraltd::StringSummary::average_density_kg_per_m3);

  py::class_<centraltd::CentralizerSummary>(m, "CentralizerSummary")
      .def(py::init<>())
      .def_readwrite("spec_count", &centraltd::CentralizerSummary::spec_count)
      .def_readwrite(
          "explicit_installation_count",
          &centraltd::CentralizerSummary::explicit_installation_count)
      .def_readwrite(
          "count_hint_total",
          &centraltd::CentralizerSummary::count_hint_total)
      .def_readwrite(
          "spacing_based_installation_estimate",
          &centraltd::CentralizerSummary::spacing_based_installation_estimate)
      .def_readwrite(
          "max_outer_diameter_m",
          &centraltd::CentralizerSummary::max_outer_diameter_m)
      .def_readwrite(
          "min_nominal_radial_clearance_m",
          &centraltd::CentralizerSummary::min_nominal_radial_clearance_m);

  py::class_<centraltd::SolverStubInput>(m, "SolverStubInput")
      .def(py::init<>())
      .def_readwrite("well", &centraltd::SolverStubInput::well)
      .def_readwrite(
          "reference_hole_diameter_m",
          &centraltd::SolverStubInput::reference_hole_diameter_m)
      .def_readwrite("string_sections", &centraltd::SolverStubInput::string_sections)
      .def_readwrite("centralizers", &centraltd::SolverStubInput::centralizers)
      .def("validate", &centraltd::SolverStubInput::validate);

  py::class_<centraltd::SolverStubResult>(m, "SolverStubResult")
      .def(py::init<>())
      .def_readwrite("status", &centraltd::SolverStubResult::status)
      .def_readwrite("message", &centraltd::SolverStubResult::message)
      .def_readwrite(
          "geometry_is_approximate",
          &centraltd::SolverStubResult::geometry_is_approximate)
      .def_readwrite(
          "trajectory_summary",
          &centraltd::SolverStubResult::trajectory_summary)
      .def_readwrite("string_summary", &centraltd::SolverStubResult::string_summary)
      .def_readwrite(
          "centralizer_summary",
          &centraltd::SolverStubResult::centralizer_summary)
      .def_readwrite(
          "section_summaries",
          &centraltd::SolverStubResult::section_summaries)
      .def_readwrite("estimated_hookload_n", &centraltd::SolverStubResult::estimated_hookload_n)
      .def_readwrite(
          "estimated_surface_torque_n_m",
          &centraltd::SolverStubResult::estimated_surface_torque_n_m)
      .def_readwrite("minimum_standoff_ratio", &centraltd::SolverStubResult::minimum_standoff_ratio)
      .def_readwrite(
          "minimum_nominal_radial_clearance_m",
          &centraltd::SolverStubResult::minimum_nominal_radial_clearance_m)
      .def_readwrite("contact_nodes", &centraltd::SolverStubResult::contact_nodes)
      .def_readwrite("warnings", &centraltd::SolverStubResult::warnings)
      .def_readwrite("todos", &centraltd::SolverStubResult::todos);

  m.def("run_solver_stub", &centraltd::run_solver_stub, py::arg("input"));
}
