#include "centraltd/solver_stub.hpp"

#include <algorithm>
#include <array>
#include <iostream>
#include <optional>
#include <vector>

int main() {
  const auto require = [](bool condition, const char* message) {
    if (!condition) {
      std::cerr << message << '\n';
      return false;
    }
    return true;
  };

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

  centraltd::CentralizerSpec standard_centralizer;
  standard_centralizer.name = "bow-spring-standard";
  standard_centralizer.type = "bow-spring";
  standard_centralizer.outer_diameter_m = 0.208;
  standard_centralizer.support_outer_diameter_m = 0.208;
  standard_centralizer.number_of_bows = 6U;
  standard_centralizer.angular_orientation_reference_deg = 0.0;
  standard_centralizer.inner_clearance_to_pipe_m = 0.0005;
  standard_centralizer.nominal_restoring_force_n = 1800.0;
  standard_centralizer.nominal_running_force_n = 900.0;
  standard_centralizer.blade_power_law_p = 1.15;
  standard_centralizer.spacing_hint_m = 25.0;
  standard_centralizer.count_hint = 18U;

  centraltd::CentralizerSpec targeted_centralizer;
  targeted_centralizer.name = "bow-spring-targeted";
  targeted_centralizer.type = "bow-spring";
  targeted_centralizer.outer_diameter_m = 0.208;
  targeted_centralizer.support_outer_diameter_m = 0.208;
  targeted_centralizer.number_of_bows = 7U;
  targeted_centralizer.angular_orientation_reference_deg = 15.0;
  targeted_centralizer.inner_clearance_to_pipe_m = 0.0003;
  targeted_centralizer.nominal_restoring_force_n = 2200.0;
  targeted_centralizer.nominal_running_force_n = 1050.0;
  targeted_centralizer.blade_power_law_p = 1.2;
  targeted_centralizer.spacing_hint_m = 15.0;
  targeted_centralizer.installation_md_m = {1500.0, 1600.0, 1700.0};

  std::vector<centraltd::CentralizerSpec> centralizers = {
      standard_centralizer,
      targeted_centralizer,
  };

  centraltd::SolverStubInput input;
  input.well = well;
  input.reference_hole_diameter_m = 0.216;
  input.fluid_density_kg_per_m3 = 1100.0;
  input.discretization_settings.target_segment_length_m = 50.0;
  input.operation_mode = "run_in";
  input.string_sections = sections;
  input.centralizers = centralizers;

  const auto result = centraltd::run_solver_stub(input);

  if (!require(
          result.status == "phase9-vector-bow-spring-td-baseline",
          "Unexpected phase9 status.")) {
    return 1;
  }
  if (!require(result.todos.size() == 6U, "Unexpected TODO count.")) {
    return 1;
  }
  if (!require(result.warnings.size() >= 4U, "Unexpected warning count.")) {
    return 1;
  }
  if (!require(
          result.estimated_hookload_n >= 0.0,
          "Estimated hookload must be non-negative.")) {
    return 1;
  }
  if (!require(
          result.hookload_pull_out_n >= result.hookload_run_in_n,
          "Pull-out hookload must not be below run-in hookload.")) {
    return 1;
  }
  if (!require(
          result.estimated_surface_torque_n_m.has_value(),
          "Surface torque should be populated.")) {
    return 1;
  }
  if (!require(
          result.estimated_surface_torque_n_m.value() >= 0.0,
          "Surface torque should be non-negative.")) {
    return 1;
  }
  if (!require(
          result.minimum_standoff_estimate >= 0.0 && result.minimum_standoff_estimate <= 1.0,
          "Minimum standoff must stay within [0, 1].")) {
    return 1;
  }
  if (!require(result.section_summaries.size() == 3U, "Unexpected section summary count.")) {
    return 1;
  }
  if (!require(result.trajectory_summary.vertical_depth_m > 0.0, "Vertical depth must be positive.")) {
    return 1;
  }
  if (!require(
          result.centralizer_summary.explicit_installation_count == 3U,
          "Unexpected explicit centralizer installation count.")) {
    return 1;
  }
  if (!require(
          result.centralizer_summary.expanded_installation_count >= 3U,
          "Expanded centralizer installation count should be at least explicit count.")) {
    return 1;
  }
  if (!require(result.mechanical_summary.segment_count > 0U, "Mechanical profile should contain segments.")) {
    return 1;
  }
  if (!require(
          result.mechanical_summary.global_solver_iteration_count > 0U,
          "Global vector solver should perform at least one iteration.")) {
    return 1;
  }
  if (!require(
          result.mechanical_profile.size() == result.mechanical_summary.segment_count,
          "Mechanical profile size must match summary segment count.")) {
    return 1;
  }
  if (!require(
          result.axial_force_run_in_profile.size() == result.mechanical_summary.segment_count,
          "Run-in axial profile size must match segment count.")) {
    return 1;
  }
  if (!require(
          result.axial_force_pull_out_profile.size() == result.mechanical_summary.segment_count,
          "Pull-out axial profile size must match segment count.")) {
    return 1;
  }
  if (!require(
          result.torque_profile.size() == result.mechanical_summary.segment_count,
          "Torque profile size must match segment count.")) {
    return 1;
  }
  if (!require(
          result.converged_axial_profile.size() == result.mechanical_summary.segment_count,
          "Converged axial profile size must match segment count.")) {
    return 1;
  }
  if (!require(
          result.converged_normal_reaction_profile.size() == result.mechanical_summary.segment_count,
          "Converged normal reaction profile size must match segment count.")) {
    return 1;
  }
  if (!require(
          result.converged_torque_profile.size() == result.mechanical_summary.segment_count,
          "Converged torque profile size must match segment count.")) {
    return 1;
  }
  if (!require(result.coupling_iterations > 0U, "Coupling loop should run at least once.")) {
    return 1;
  }
  if (!require(!result.coupling_status.empty(), "Coupling status should not be empty.")) {
    return 1;
  }
  if (!require(
          result.mechanical_summary.maximum_bending_moment_n_m >= 0.0,
          "Maximum bending moment must be non-negative.")) {
    return 1;
  }
  if (!require(
          result.mechanical_summary.maximum_bending_stress_pa >= 0.0,
          "Maximum bending stress must be non-negative.")) {
    return 1;
  }
  if (!require(
          result.mechanical_summary.maximum_normal_reaction_estimate_n >= 0.0,
          "Maximum normal reaction must be non-negative.")) {
    return 1;
  }
  if (!require(
          !result.mechanical_profile.front().contact_state.empty(),
          "Contact state should not be empty.")) {
    return 1;
  }
  if (!require(
          !result.torque_and_drag_real_implemented,
          "Reduced vector torque and drag must remain flagged as non-final.")) {
    return 1;
  }
  if (!require(
          result.torque_and_drag_status == "phase9-reduced-bow-spring-td-baseline",
          "Unexpected torque and drag status.")) {
    return 1;
  }
  if (!require(
          result.centralizer_model_status == "phase9-detailed-bow-spring",
          "Unexpected centralizer model status.")) {
    return 1;
  }
  if (!require(
          result.centralizer_axial_friction_profile.size() == result.mechanical_summary.segment_count,
          "Centralizer axial friction profile size must match segment count.")) {
    return 1;
  }
  if (!require(
          result.centralizer_torque_profile.size() == result.mechanical_summary.segment_count,
          "Centralizer torque profile size must match segment count.")) {
    return 1;
  }
  const bool any_bow_force_output = std::any_of(
      result.mechanical_profile.begin(),
      result.mechanical_profile.end(),
      [](const centraltd::MechanicalSegmentResult& segment) {
        return !segment.bow_force_details.empty();
      });
  if (!require(
          any_bow_force_output,
          "Detailed bow-force outputs should not be empty in the smoke case.")) {
    return 1;
  }
  const auto bow_active_segment = std::find_if(
      result.mechanical_profile.begin(),
      result.mechanical_profile.end(),
      [](const centraltd::MechanicalSegmentResult& segment) {
        return segment.bow_resultant_magnitude_n > 0.0 || segment.centralizer_torque_increment_n_m > 0.0;
      });
  if (!require(
          bow_active_segment != result.mechanical_profile.end(),
          "At least one smoke-test segment should develop a bow resultant or centralizer torque increment.")) {
    return 1;
  }
  if (!require(
          bow_active_segment->bow_resultant_magnitude_n >= 0.0,
          "Bow resultant magnitude must be non-negative.")) {
    return 1;
  }
  if (!require(
          bow_active_segment->centralizer_torque_increment_n_m >= 0.0,
          "Centralizer torque increment must be non-negative.")) {
    return 1;
  }
  if (!require(
          result.mechanical_profile.front().tangent_north_east_tvd[2] >= 0.0,
          "The front tangent should still point downward in TVD.")) {
    return 1;
  }
  if (!require(
          result.mechanical_profile.front().eccentricity_estimate_m >= 0.0,
          "Front eccentricity magnitude must be non-negative.")) {
    return 1;
  }

  return 0;
}
