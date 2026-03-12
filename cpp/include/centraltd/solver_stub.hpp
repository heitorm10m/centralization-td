#pragma once

#include "centraltd/centralizer.hpp"
#include "centraltd/coupling_driver.hpp"
#include "centraltd/discretization.hpp"
#include "centraltd/string_section.hpp"
#include "centraltd/well.hpp"

#include <optional>
#include <string>
#include <vector>

namespace centraltd {

struct SolverStubInput {
  WellTrajectory well;
  Scalar reference_hole_diameter_m{0.0};
  Scalar fluid_density_kg_per_m3{1000.0};
  DiscretizationSettings discretization_settings;
  std::string operation_mode{"run_in"};
  std::vector<StringSection> string_sections;
  std::vector<CentralizerSpec> centralizers;

  void validate() const;
};

struct StringSectionSummary {
  std::string name;
  Scalar md_start_m{0.0};
  Scalar md_end_m{0.0};
  Scalar length_m{0.0};
  Scalar outer_diameter_m{0.0};
  Scalar inner_diameter_m{0.0};
  Scalar linear_weight_n_per_m{0.0};
  Scalar friction_coefficient{0.0};
  Scalar nominal_radial_clearance_m{0.0};
};

struct StringSummary {
  std::size_t section_count{0};
  Scalar total_length_m{0.0};
  Scalar total_weight_n{0.0};
  Scalar total_effective_weight_n{0.0};
  Scalar max_outer_diameter_m{0.0};
  Scalar min_inner_diameter_m{0.0};
  Scalar average_friction_coefficient{0.0};
  Scalar average_density_kg_per_m3{0.0};
};

struct CentralizerSummary {
  std::size_t spec_count{0};
  std::size_t explicit_installation_count{0};
  std::size_t count_hint_total{0};
  std::size_t spacing_based_installation_estimate{0};
  std::size_t expanded_installation_count{0};
  Scalar max_outer_diameter_m{0.0};
  Scalar min_nominal_radial_clearance_m{0.0};
};

struct SolverStubResult {
  std::string status{"phase9-vector-bow-spring-td-baseline"};
  std::string message;
  std::string operation_mode{"run_in"};
  bool geometry_is_approximate{true};
  TrajectorySummary trajectory_summary;
  StringSummary string_summary;
  CentralizerSummary centralizer_summary;
  MechanicalSummary mechanical_summary;
  std::vector<StringSectionSummary> section_summaries;
  std::vector<MechanicalSegmentResult> mechanical_profile;
  Scalar estimated_hookload_n{0.0};
  Scalar hookload_run_in_n{0.0};
  Scalar hookload_pull_out_n{0.0};
  Scalar drag_run_in_n{0.0};
  Scalar drag_pull_out_n{0.0};
  std::vector<AxialForcePoint> axial_force_run_in_profile;
  std::vector<AxialForcePoint> axial_force_pull_out_profile;
  std::vector<TorquePoint> torque_profile;
  std::vector<CentralizerFrictionPoint> centralizer_axial_friction_profile;
  std::vector<CentralizerFrictionPoint> centralizer_tangential_friction_profile;
  std::vector<TorquePoint> centralizer_torque_profile;
  std::string coupling_status{"max_iterations_reached"};
  std::size_t coupling_iterations{0};
  Scalar coupling_final_max_profile_update_n{0.0};
  bool coupling_converged{false};
  std::vector<AxialForcePoint> converged_axial_profile;
  std::vector<NormalReactionPoint> converged_normal_reaction_profile;
  std::vector<TorquePoint> converged_torque_profile;
  std::optional<Scalar> estimated_surface_torque_n_m;
  Scalar minimum_standoff_estimate{0.0};
  Scalar minimum_nominal_radial_clearance_m{0.0};
  std::size_t contact_nodes{0};
  std::string centralizer_model_status{"phase9-detailed-bow-spring"};
  bool torque_and_drag_real_implemented{false};
  std::string torque_and_drag_status{"phase9-reduced-bow-spring-td-baseline"};
  std::string torque_drag_status{"phase9-reduced-bow-spring-td-baseline"};
  std::vector<std::string> warnings;
  std::vector<std::string> todos;
};

SolverStubResult run_solver_stub(const SolverStubInput& input);

}  // namespace centraltd
