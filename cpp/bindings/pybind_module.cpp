#include "centraltd/centralizer.hpp"
#include "centraltd/discretization.hpp"
#include "centraltd/mechanical_solver.hpp"
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
      .def("outer_radius_m", &centraltd::StringSection::outer_radius_m)
      .def("cross_sectional_area_m2", &centraltd::StringSection::cross_sectional_area_m2)
      .def("second_moment_of_area_m4", &centraltd::StringSection::second_moment_of_area_m4)
      .def("bending_stiffness_n_m2", &centraltd::StringSection::bending_stiffness_n_m2)
      .def("displaced_area_m2", &centraltd::StringSection::displaced_area_m2)
      .def(
          "buoyancy_force_n_per_m",
          &centraltd::StringSection::buoyancy_force_n_per_m,
          py::arg("fluid_density_kg_per_m3"))
      .def(
          "effective_line_weight_n_per_m",
          &centraltd::StringSection::effective_line_weight_n_per_m,
          py::arg("fluid_density_kg_per_m3"))
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
      .def_readwrite(
          "total_effective_weight_n",
          &centraltd::StringSummary::total_effective_weight_n)
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
          "expanded_installation_count",
          &centraltd::CentralizerSummary::expanded_installation_count)
      .def_readwrite(
          "max_outer_diameter_m",
          &centraltd::CentralizerSummary::max_outer_diameter_m)
      .def_readwrite(
          "min_nominal_radial_clearance_m",
          &centraltd::CentralizerSummary::min_nominal_radial_clearance_m);

  py::class_<centraltd::DiscretizationSettings>(m, "DiscretizationSettings")
      .def(py::init<>())
      .def_readwrite(
          "target_segment_length_m",
          &centraltd::DiscretizationSettings::target_segment_length_m)
      .def_readwrite(
          "global_solver_max_iterations",
          &centraltd::DiscretizationSettings::global_solver_max_iterations)
      .def_readwrite(
          "contact_penalty_scale",
          &centraltd::DiscretizationSettings::contact_penalty_scale)
      .def("validate", &centraltd::DiscretizationSettings::validate);

  py::class_<centraltd::MechanicalSegmentResult>(m, "MechanicalSegmentResult")
      .def(py::init<>())
      .def_readwrite(
          "measured_depth_start_m",
          &centraltd::MechanicalSegmentResult::measured_depth_start_m)
      .def_readwrite(
          "measured_depth_end_m",
          &centraltd::MechanicalSegmentResult::measured_depth_end_m)
      .def_readwrite(
          "measured_depth_center_m",
          &centraltd::MechanicalSegmentResult::measured_depth_center_m)
      .def_readwrite("segment_length_m", &centraltd::MechanicalSegmentResult::segment_length_m)
      .def_readwrite("section_name", &centraltd::MechanicalSegmentResult::section_name)
      .def_readwrite("inclination_rad", &centraltd::MechanicalSegmentResult::inclination_rad)
      .def_readwrite(
          "curvature_rad_per_m",
          &centraltd::MechanicalSegmentResult::curvature_rad_per_m)
      .def_readwrite(
          "effective_line_weight_n_per_m",
          &centraltd::MechanicalSegmentResult::effective_line_weight_n_per_m)
      .def_readwrite(
          "effective_axial_load_n",
          &centraltd::MechanicalSegmentResult::effective_axial_load_n)
      .def_readwrite(
          "bending_stiffness_n_m2",
          &centraltd::MechanicalSegmentResult::bending_stiffness_n_m2)
      .def_readwrite(
          "bending_moment_n_m",
          &centraltd::MechanicalSegmentResult::bending_moment_n_m)
      .def_readwrite(
          "bending_stress_pa",
          &centraltd::MechanicalSegmentResult::bending_stress_pa)
      .def_readwrite(
          "bending_strain_estimate",
          &centraltd::MechanicalSegmentResult::bending_strain_estimate)
      .def_readwrite(
          "gravity_lateral_load_n_per_m",
          &centraltd::MechanicalSegmentResult::gravity_lateral_load_n_per_m)
      .def_readwrite(
          "curvature_lateral_load_n_per_m",
          &centraltd::MechanicalSegmentResult::curvature_lateral_load_n_per_m)
      .def_readwrite(
          "equivalent_lateral_load_n_per_m",
          &centraltd::MechanicalSegmentResult::equivalent_lateral_load_n_per_m)
      .def_readwrite(
          "equivalent_lateral_force_n",
          &centraltd::MechanicalSegmentResult::equivalent_lateral_force_n)
      .def_readwrite(
          "bending_lateral_stiffness_n_per_m",
          &centraltd::MechanicalSegmentResult::bending_lateral_stiffness_n_per_m)
      .def_readwrite(
          "axial_tension_lateral_stiffness_n_per_m",
          &centraltd::MechanicalSegmentResult::axial_tension_lateral_stiffness_n_per_m)
      .def_readwrite(
          "structural_lateral_stiffness_n_per_m",
          &centraltd::MechanicalSegmentResult::structural_lateral_stiffness_n_per_m)
      .def_readwrite(
          "centralizer_centering_stiffness_n_per_m",
          &centraltd::MechanicalSegmentResult::centralizer_centering_stiffness_n_per_m)
      .def_readwrite(
          "support_contact_penalty_n_per_m",
          &centraltd::MechanicalSegmentResult::support_contact_penalty_n_per_m)
      .def_readwrite(
          "body_contact_penalty_n_per_m",
          &centraltd::MechanicalSegmentResult::body_contact_penalty_n_per_m)
      .def_readwrite(
          "support_outer_diameter_m",
          &centraltd::MechanicalSegmentResult::support_outer_diameter_m)
      .def_readwrite(
          "pipe_body_clearance_m",
          &centraltd::MechanicalSegmentResult::pipe_body_clearance_m)
      .def_readwrite(
          "support_contact_clearance_m",
          &centraltd::MechanicalSegmentResult::support_contact_clearance_m)
      .def_readwrite(
          "free_eccentricity_estimate_m",
          &centraltd::MechanicalSegmentResult::free_eccentricity_estimate_m)
      .def_readwrite(
          "eccentricity_estimate_m",
          &centraltd::MechanicalSegmentResult::eccentricity_estimate_m)
      .def_readwrite(
          "eccentricity_ratio",
          &centraltd::MechanicalSegmentResult::eccentricity_ratio)
      .def_readwrite("standoff_estimate", &centraltd::MechanicalSegmentResult::standoff_estimate)
      .def_readwrite(
          "support_normal_reaction_estimate_n",
          &centraltd::MechanicalSegmentResult::support_normal_reaction_estimate_n)
      .def_readwrite(
          "body_normal_reaction_estimate_n",
          &centraltd::MechanicalSegmentResult::body_normal_reaction_estimate_n)
      .def_readwrite(
          "normal_reaction_estimate_n",
          &centraltd::MechanicalSegmentResult::normal_reaction_estimate_n)
      .def_readwrite(
          "normal_reaction_estimate_n_per_m",
          &centraltd::MechanicalSegmentResult::normal_reaction_estimate_n_per_m)
      .def_readwrite(
          "nearby_centralizer_count",
          &centraltd::MechanicalSegmentResult::nearby_centralizer_count)
      .def_readwrite(
          "contact_iteration_count",
          &centraltd::MechanicalSegmentResult::contact_iteration_count)
      .def_readwrite("contact_state", &centraltd::MechanicalSegmentResult::contact_state)
      .def_readwrite("support_in_contact", &centraltd::MechanicalSegmentResult::support_in_contact)
      .def_readwrite(
          "pipe_body_in_contact",
          &centraltd::MechanicalSegmentResult::pipe_body_in_contact)
      .def_readwrite("standoff_proxy", &centraltd::MechanicalSegmentResult::standoff_estimate);

  py::class_<centraltd::MechanicalSummary>(m, "MechanicalSummary")
      .def(py::init<>())
      .def_readwrite("segment_count", &centraltd::MechanicalSummary::segment_count)
      .def_readwrite(
          "target_segment_length_m",
          &centraltd::MechanicalSummary::target_segment_length_m)
      .def_readwrite(
          "global_solver_iteration_count",
          &centraltd::MechanicalSummary::global_solver_iteration_count)
      .def_readwrite(
          "global_solver_final_update_norm_m",
          &centraltd::MechanicalSummary::global_solver_final_update_norm_m)
      .def_readwrite(
          "top_effective_axial_load_n",
          &centraltd::MechanicalSummary::top_effective_axial_load_n)
      .def_readwrite(
          "minimum_effective_axial_load_n",
          &centraltd::MechanicalSummary::minimum_effective_axial_load_n)
      .def_readwrite(
          "maximum_effective_axial_load_n",
          &centraltd::MechanicalSummary::maximum_effective_axial_load_n)
      .def_readwrite(
          "maximum_bending_moment_n_m",
          &centraltd::MechanicalSummary::maximum_bending_moment_n_m)
      .def_readwrite(
          "maximum_bending_stress_pa",
          &centraltd::MechanicalSummary::maximum_bending_stress_pa)
      .def_readwrite(
          "maximum_bending_strain_estimate",
          &centraltd::MechanicalSummary::maximum_bending_strain_estimate)
      .def_readwrite(
          "maximum_equivalent_lateral_load_n_per_m",
          &centraltd::MechanicalSummary::maximum_equivalent_lateral_load_n_per_m)
      .def_readwrite(
          "maximum_eccentricity_estimate_m",
          &centraltd::MechanicalSummary::maximum_eccentricity_estimate_m)
      .def_readwrite(
          "maximum_eccentricity_ratio",
          &centraltd::MechanicalSummary::maximum_eccentricity_ratio)
      .def_readwrite(
          "minimum_standoff_estimate",
          &centraltd::MechanicalSummary::minimum_standoff_estimate)
      .def_readwrite(
          "maximum_normal_reaction_estimate_n",
          &centraltd::MechanicalSummary::maximum_normal_reaction_estimate_n)
      .def_readwrite(
          "contact_segment_count",
          &centraltd::MechanicalSummary::contact_segment_count)
      .def_readwrite(
          "support_contact_segment_count",
          &centraltd::MechanicalSummary::support_contact_segment_count)
      .def_readwrite(
          "pipe_body_contact_segment_count",
          &centraltd::MechanicalSummary::pipe_body_contact_segment_count)
      .def_readwrite(
          "minimum_standoff_proxy",
          &centraltd::MechanicalSummary::minimum_standoff_estimate);

  py::class_<centraltd::SolverStubInput>(m, "SolverStubInput")
      .def(py::init<>())
      .def_readwrite("well", &centraltd::SolverStubInput::well)
      .def_readwrite(
          "reference_hole_diameter_m",
          &centraltd::SolverStubInput::reference_hole_diameter_m)
      .def_readwrite(
          "fluid_density_kg_per_m3",
          &centraltd::SolverStubInput::fluid_density_kg_per_m3)
      .def_readwrite(
          "discretization_settings",
          &centraltd::SolverStubInput::discretization_settings)
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
          "mechanical_summary",
          &centraltd::SolverStubResult::mechanical_summary)
      .def_readwrite(
          "section_summaries",
          &centraltd::SolverStubResult::section_summaries)
      .def_readwrite(
          "mechanical_profile",
          &centraltd::SolverStubResult::mechanical_profile)
      .def_readwrite("estimated_hookload_n", &centraltd::SolverStubResult::estimated_hookload_n)
      .def_readwrite(
          "estimated_surface_torque_n_m",
          &centraltd::SolverStubResult::estimated_surface_torque_n_m)
      .def_readwrite(
          "minimum_standoff_estimate",
          &centraltd::SolverStubResult::minimum_standoff_estimate)
      .def_readwrite(
          "minimum_standoff_ratio",
          &centraltd::SolverStubResult::minimum_standoff_estimate)
      .def_readwrite(
          "minimum_nominal_radial_clearance_m",
          &centraltd::SolverStubResult::minimum_nominal_radial_clearance_m)
      .def_readwrite("contact_nodes", &centraltd::SolverStubResult::contact_nodes)
      .def_readwrite(
          "torque_and_drag_real_implemented",
          &centraltd::SolverStubResult::torque_and_drag_real_implemented)
      .def_readwrite(
          "torque_and_drag_status",
          &centraltd::SolverStubResult::torque_and_drag_status)
      .def_readwrite("warnings", &centraltd::SolverStubResult::warnings)
      .def_readwrite("todos", &centraltd::SolverStubResult::todos);

  m.def("run_solver_stub", &centraltd::run_solver_stub, py::arg("input"));
}
