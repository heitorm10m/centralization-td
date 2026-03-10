#include "centraltd/solver_stub.hpp"

#include <optional>
#include <vector>

int main() {
  centraltd::WellTrajectory well({
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {400.0, 0.05, 0.0, 0.0, 0.0, 0.0},
      {1200.0, 0.30, 0.40, 0.0, 0.0, 0.0},
      {1800.0, 0.60, 0.85, 0.0, 0.0, 0.0},
  });

  std::vector<centraltd::StringSection> sections = {
      {"casing-top", 0.0, 600.0, 0.1778, 0.1524, 680.0, 2.07e11, 8.0e10, 7850.0, 0.24},
      {"casing-mid", 600.0, 1300.0, 0.1778, 0.1524, 700.0, 2.07e11, 8.0e10, 7850.0, 0.26},
      {"casing-bottom", 1300.0, 1750.0, 0.1778, 0.1524, 730.0, 2.07e11, 8.0e10, 7850.0, 0.28},
  };

  std::vector<centraltd::CentralizerSpec> centralizers = {
      {"bow-spring-standard", "bow-spring", 0.208, 1800.0, 900.0, 25.0, 18U, {}},
      {"bow-spring-targeted", "bow-spring", 0.208, 2200.0, 1050.0, 15.0, std::nullopt, {1500.0, 1600.0, 1700.0}},
  };

  centraltd::SolverStubInput input;
  input.well = well;
  input.reference_hole_diameter_m = 0.216;
  input.fluid_density_kg_per_m3 = 1100.0;
  input.discretization_settings.target_segment_length_m = 50.0;
  input.string_sections = sections;
  input.centralizers = centralizers;

  const auto result = centraltd::run_solver_stub(input);

  if (result.status != "phase5-global-stiff-string-baseline") {
    return 1;
  }
  if (result.todos.size() != 6U) {
    return 1;
  }
  if (result.warnings.size() < 4U) {
    return 1;
  }
  if (!(result.estimated_hookload_n > 0.0)) {
    return 1;
  }
  if (result.estimated_surface_torque_n_m.has_value()) {
    return 1;
  }
  if (!(result.minimum_standoff_estimate >= 0.0 && result.minimum_standoff_estimate <= 1.0)) {
    return 1;
  }
  if (result.section_summaries.size() != 3U) {
    return 1;
  }
  if (!(result.trajectory_summary.vertical_depth_m > 0.0)) {
    return 1;
  }
  if (result.centralizer_summary.explicit_installation_count != 3U) {
    return 1;
  }
  if (result.centralizer_summary.expanded_installation_count < 3U) {
    return 1;
  }
  if (result.mechanical_summary.segment_count == 0U) {
    return 1;
  }
  if (result.mechanical_summary.global_solver_iteration_count == 0U) {
    return 1;
  }
  if (result.mechanical_profile.size() != result.mechanical_summary.segment_count) {
    return 1;
  }
  if (!(result.mechanical_summary.maximum_bending_moment_n_m >= 0.0)) {
    return 1;
  }
  if (!(result.mechanical_summary.maximum_bending_stress_pa >= 0.0)) {
    return 1;
  }
  if (!(result.mechanical_summary.maximum_normal_reaction_estimate_n >= 0.0)) {
    return 1;
  }
  if (result.mechanical_profile.front().contact_state.empty()) {
    return 1;
  }
  if (result.torque_and_drag_real_implemented) {
    return 1;
  }
  if (result.torque_and_drag_status != "not-implemented-yet") {
    return 1;
  }

  return 0;
}
