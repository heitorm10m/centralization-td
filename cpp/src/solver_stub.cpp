#include "centraltd/solver_stub.hpp"

#include "centraltd/types.hpp"

#include <algorithm>
#include <limits>
#include <numeric>

namespace centraltd {
namespace {

StringSummary summarize_string(
    const std::vector<StringSection>& sections,
    Scalar fluid_density_kg_per_m3) {
  StringSummary summary;
  summary.section_count = sections.size();
  summary.min_inner_diameter_m = std::numeric_limits<Scalar>::max();

  for (const auto& section : sections) {
    summary.total_length_m += section.length_m();
    summary.total_weight_n += section.length_m() * section.linear_weight_n_per_m;
    summary.total_effective_weight_n +=
        section.length_m() * section.effective_line_weight_n_per_m(fluid_density_kg_per_m3);
    summary.max_outer_diameter_m = std::max(summary.max_outer_diameter_m, section.outer_diameter_m);
    summary.min_inner_diameter_m = std::min(summary.min_inner_diameter_m, section.inner_diameter_m);
    summary.average_friction_coefficient += section.friction_coefficient * section.length_m();
    summary.average_density_kg_per_m3 += section.density_kg_per_m3 * section.length_m();
  }

  if (summary.total_length_m > 0.0) {
    summary.average_friction_coefficient /= summary.total_length_m;
    summary.average_density_kg_per_m3 /= summary.total_length_m;
  } else {
    summary.min_inner_diameter_m = 0.0;
  }

  return summary;
}

std::size_t spacing_based_estimate(const CentralizerSpec& spec, Scalar coverage_length_m) {
  if (spec.count_hint.has_value()) {
    return spec.count_hint.value();
  }
  if (!spec.installation_md_m.empty()) {
    return spec.installation_md_m.size();
  }
  if (coverage_length_m <= 0.0 || spec.spacing_hint_m <= 0.0) {
    return 0U;
  }

  return std::max<std::size_t>(
      1U,
      static_cast<std::size_t>(coverage_length_m / spec.spacing_hint_m) + 1U);
}

CentralizerSummary summarize_centralizers(
    const std::vector<CentralizerSpec>& centralizers,
    const std::vector<CentralizerPlacement>& placements,
    Scalar coverage_length_m,
    Scalar reference_hole_diameter_m) {
  CentralizerSummary summary;
  summary.spec_count = centralizers.size();
  summary.expanded_installation_count = placements.size();
  summary.min_nominal_radial_clearance_m = std::numeric_limits<Scalar>::max();

  for (const auto& spec : centralizers) {
    summary.explicit_installation_count += spec.explicit_installation_count();
    summary.count_hint_total += spec.count_hint.value_or(0U);
    summary.spacing_based_installation_estimate += spacing_based_estimate(spec, coverage_length_m);
    summary.max_outer_diameter_m = std::max(
        summary.max_outer_diameter_m,
        spec.resolved_support_outer_diameter_m());

    if (reference_hole_diameter_m > 0.0) {
      const Scalar nominal_radial_clearance_m =
          0.5 * (reference_hole_diameter_m - spec.effective_contact_diameter_m());
      summary.min_nominal_radial_clearance_m = std::min(
          summary.min_nominal_radial_clearance_m,
          nominal_radial_clearance_m);
    }
  }

  if (summary.min_nominal_radial_clearance_m == std::numeric_limits<Scalar>::max()) {
    summary.min_nominal_radial_clearance_m = 0.0;
  }

  return summary;
}

std::vector<StringSectionSummary> summarize_sections(
    const std::vector<StringSection>& sections,
    Scalar reference_hole_diameter_m) {
  std::vector<StringSectionSummary> summaries;
  summaries.reserve(sections.size());

  for (const auto& section : sections) {
    StringSectionSummary summary;
    summary.name = section.name;
    summary.md_start_m = section.md_start_m;
    summary.md_end_m = section.md_end_m;
    summary.length_m = section.length_m();
    summary.outer_diameter_m = section.outer_diameter_m;
    summary.inner_diameter_m = section.inner_diameter_m;
    summary.linear_weight_n_per_m = section.linear_weight_n_per_m;
    summary.friction_coefficient = section.friction_coefficient;
    summary.nominal_radial_clearance_m =
        reference_hole_diameter_m > 0.0
            ? 0.5 * (reference_hole_diameter_m - section.outer_diameter_m)
            : 0.0;
    summaries.push_back(summary);
  }

  return summaries;
}

Scalar minimum_section_clearance_m(const std::vector<StringSectionSummary>& section_summaries) {
  Scalar minimum_clearance_m = std::numeric_limits<Scalar>::max();
  for (const auto& section_summary : section_summaries) {
    minimum_clearance_m = std::min(minimum_clearance_m, section_summary.nominal_radial_clearance_m);
  }
  return minimum_clearance_m == std::numeric_limits<Scalar>::max() ? 0.0 : minimum_clearance_m;
}

}  // namespace

void SolverStubInput::validate() const {
  well.validate();
  discretization_settings.validate();

  if (reference_hole_diameter_m < 0.0) {
    throw ValidationError("Reference hole diameter must be non-negative.");
  }
  if (fluid_density_kg_per_m3 < 0.0) {
    throw ValidationError("Fluid density must be non-negative.");
  }
  if (!is_supported_operation_mode(operation_mode)) {
    throw ValidationError("Operation mode must be one of: run_in, pull_out, rotate_in_place.");
  }
  if (string_sections.empty()) {
    throw ValidationError("At least one string section is required.");
  }

  Scalar previous_section_end_m = -1.0;
  const Scalar final_well_md_m = well.final_measured_depth_m();
  for (const auto& section : string_sections) {
    section.validate();
    if (section.md_start_m < previous_section_end_m) {
      throw ValidationError("String sections must be ordered and non-overlapping.");
    }
    if (reference_hole_diameter_m > 0.0 && section.outer_diameter_m > reference_hole_diameter_m) {
      throw ValidationError("String outer diameter cannot exceed the reference hole diameter.");
    }
    if (section.md_end_m > final_well_md_m) {
      throw ValidationError("String section measured depth cannot exceed the final well MD.");
    }
    previous_section_end_m = section.md_end_m;
  }

  for (const auto& spec : centralizers) {
    spec.validate();
    if (reference_hole_diameter_m > 0.0 &&
        spec.effective_contact_diameter_m() > reference_hole_diameter_m) {
      throw ValidationError("Centralizer outer diameter cannot exceed the reference hole diameter.");
    }
    for (const Scalar installation_md : spec.installation_md_m) {
      if (installation_md > final_well_md_m) {
        throw ValidationError("Centralizer installation MD cannot exceed the final well MD.");
      }
    }
  }
}

SolverStubResult run_solver_stub(const SolverStubInput& input) {
  input.validate();

  const auto section_summaries = summarize_sections(
      input.string_sections,
      input.reference_hole_diameter_m);
  const auto discretized_problem = discretize_problem(
      input.well,
      input.reference_hole_diameter_m,
      input.fluid_density_kg_per_m3,
      input.discretization_settings,
      input.string_sections,
      input.centralizers);
  const auto coupled_result = run_coupled_global_baseline(
      discretized_problem,
      input.string_sections,
      input.operation_mode);
  const auto string_summary = summarize_string(
      input.string_sections,
      input.fluid_density_kg_per_m3);
  const auto centralizer_summary = summarize_centralizers(
      input.centralizers,
      discretized_problem.centralizer_placements,
      string_summary.total_length_m,
      input.reference_hole_diameter_m);
  const Scalar minimum_nominal_clearance_m =
      input.centralizers.empty()
          ? minimum_section_clearance_m(section_summaries)
          : std::min(
                minimum_section_clearance_m(section_summaries),
                centralizer_summary.min_nominal_radial_clearance_m);

  SolverStubResult result;
  result.status = "phase9-vector-bow-spring-td-baseline";
  result.message =
      "Phase 9 reduced vector-frame torque and drag baseline with detailed bow-spring "
      "centralizer modeling. The lateral/contact solve still uses two transverse displacement "
      "components in the local trajectory frame, but support reactions and centralizer torque "
      "now come from bow-by-bow reduced resultants. This remains a reduced engineering baseline, "
      "not a full commercial stiff-string/contact solver.";
  result.operation_mode = input.operation_mode;
  result.geometry_is_approximate = true;
  result.trajectory_summary = discretized_problem.trajectory_summary;
  result.string_summary = string_summary;
  result.centralizer_summary = centralizer_summary;
  result.mechanical_summary = coupled_result.mechanical_result.summary;
  result.section_summaries = section_summaries;
  result.mechanical_profile = coupled_result.mechanical_result.segment_results;
  result.hookload_run_in_n = coupled_result.torque_drag_result.hookload_run_in_n;
  result.hookload_pull_out_n = coupled_result.torque_drag_result.hookload_pull_out_n;
  result.drag_run_in_n = coupled_result.torque_drag_result.drag_run_in_n;
  result.drag_pull_out_n = coupled_result.torque_drag_result.drag_pull_out_n;
  result.axial_force_run_in_profile = coupled_result.torque_drag_result.axial_force_run_in_profile;
  result.axial_force_pull_out_profile = coupled_result.torque_drag_result.axial_force_pull_out_profile;
  result.torque_profile = coupled_result.torque_drag_result.torque_profile;
  result.centralizer_axial_friction_profile =
      coupled_result.torque_drag_result.centralizer_axial_friction_profile;
  result.centralizer_tangential_friction_profile =
      coupled_result.torque_drag_result.centralizer_tangential_friction_profile;
  result.centralizer_torque_profile =
      coupled_result.torque_drag_result.centralizer_torque_profile;
  result.coupling_status = coupled_result.status;
  result.coupling_iterations = coupled_result.iteration_count;
  result.coupling_converged = coupled_result.converged;
  result.converged_axial_profile = coupled_result.converged_axial_profile;
  result.converged_normal_reaction_profile = coupled_result.converged_normal_reaction_profile;
  result.converged_torque_profile = coupled_result.converged_torque_profile;
  result.estimated_hookload_n =
      input.operation_mode == "pull_out"
          ? coupled_result.torque_drag_result.hookload_pull_out_n
          : input.operation_mode == "rotate_in_place"
                ? coupled_result.mechanical_result.summary.top_effective_axial_load_n
                : coupled_result.torque_drag_result.hookload_run_in_n;
  result.estimated_surface_torque_n_m =
      coupled_result.torque_drag_result.estimated_surface_torque_n_m;
  result.minimum_standoff_estimate =
      coupled_result.mechanical_result.summary.minimum_standoff_estimate;
  result.minimum_nominal_radial_clearance_m = minimum_nominal_clearance_m;
  result.contact_nodes = coupled_result.mechanical_result.summary.contact_segment_count;
  result.centralizer_model_status = "phase9-detailed-bow-spring";
  result.torque_and_drag_real_implemented = false;
  result.torque_and_drag_status = "phase9-reduced-bow-spring-td-baseline";
  result.torque_drag_status = "phase9-reduced-bow-spring-td-baseline";
  result.warnings = {
      "Trajectory coordinates remain approximated by balanced-tangent integration of MD, "
      "inclination, and azimuth.",
      "The local trajectory frame uses a reduced parallel-transport construction, not a full "
      "differential-geometry reference implementation.",
      "The lateral equilibrium is now solved with two transverse displacement components in the "
      "local normal/binormal plane, but it is still not a full 6-DOF beam formulation.",
      "The bending stiffness term uses the simply supported beam equivalence "
      "delta_max = 5 q L^4 / (384 E I), so the 384/5 factor is a reduced structural hypothesis.",
      "Contact is still handled with reduced vector active-set penalty iterations against annular "
      "clearance, but centralizer support loads are now post-processed bow-by-bow in the local "
      "frame instead of using only a nominal support proxy.",
      "Each bow is distributed uniformly in angle from the centralizer angular reference, and "
      "its reduced deflection is evaluated as max(0, e · r_i - (c_support + c_inner)).",
      "The bow constitutive law is reduced to F_i = k_blade * delta_i^p, with optional direct "
      "k/p input or fallback calibration from nominal restoring force at a reference deflection.",
      "In the reduced torque-drag model, pipe-body friction still uses mu * N_body, while the "
      "centralizer contribution uses the bow-spring resultant magnitude scaled by the nominal "
      "running/restoring-force ratio.",
      "Run in/slackoff uses friction opposite the downward motion and therefore subtracts the "
      "body plus centralizer axial friction increments; pull out/pickup adds them because "
      "friction opposes the upward motion.",
      "The coupling loop only converges the axial profile associated with the selected "
      "operation mode; the non-selected operational profiles remain reduced post-processing "
      "against the converged normal-reaction field.",
      "Torque and drag remain reduced and do not yet use a full nonlinear 3D tangential-contact "
      "solve or a 6-DOF beam formulation.",
  };
  if (input.reference_hole_diameter_m <= 0.0) {
    result.warnings.push_back(
        "Reference hole diameter is absent, so radial clearance and contact metrics are "
        "reported as zero or conservative defaults.");
  }
  result.todos = {
      "TODO: evolve the reduced 2-DOF transverse model toward a fuller 3D beam formulation with "
      "rotations and torsion.",
      "TODO: refine vector contact with stronger nonlinear iteration and wall-reaction handling.",
      "TODO: calibrate the bow-spring power-law parameters against manufacturer/API data.",
      "TODO: refine axial drag with stronger bidirectional coupling between contact state and "
      "friction propagation.",
      "TODO: extend reduced torque and drag toward fuller bow-resolved nonlinear workflows and "
      "vector tangential contact.",
      "TODO: implement design-space optimization workflow.",
  };

  return result;
}

}  // namespace centraltd
