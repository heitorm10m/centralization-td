#include "centraltd/solver_stub.hpp"

#include "centraltd/types.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace centraltd {
namespace {

StringSummary summarize_string(const std::vector<StringSection>& sections) {
  StringSummary summary;
  summary.section_count = sections.size();
  summary.min_inner_diameter_m = std::numeric_limits<Scalar>::max();

  for (const auto& section : sections) {
    summary.total_length_m += section.length_m();
    summary.total_weight_n += section.length_m() * section.linear_weight_n_per_m;
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
      static_cast<std::size_t>(std::floor(coverage_length_m / spec.spacing_hint_m)) + 1U);
}

CentralizerSummary summarize_centralizers(
    const std::vector<CentralizerSpec>& centralizers,
    Scalar coverage_length_m,
    Scalar reference_hole_diameter_m) {
  CentralizerSummary summary;
  summary.spec_count = centralizers.size();
  summary.min_nominal_radial_clearance_m = std::numeric_limits<Scalar>::max();

  for (const auto& spec : centralizers) {
    summary.explicit_installation_count += spec.explicit_installation_count();
    summary.count_hint_total += spec.count_hint.value_or(0U);
    summary.spacing_based_installation_estimate += spacing_based_estimate(spec, coverage_length_m);
    summary.max_outer_diameter_m = std::max(summary.max_outer_diameter_m, spec.outer_diameter_m);

    if (reference_hole_diameter_m > 0.0) {
      const Scalar nominal_radial_clearance_m =
          0.5 * (reference_hole_diameter_m - spec.outer_diameter_m);
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

std::size_t curvature_risk_count(const std::vector<TrajectoryGeometryNode>& geometry_nodes) {
  constexpr Scalar kCurvatureRiskThresholdRadPerM = 3.5e-4;
  return std::count_if(
      geometry_nodes.begin(),
      geometry_nodes.end(),
      [](const TrajectoryGeometryNode& node) {
        return node.discrete_curvature_rad_per_m >= kCurvatureRiskThresholdRadPerM;
      });
}

}  // namespace

void SolverStubInput::validate() const {
  well.validate();
  if (reference_hole_diameter_m < 0.0) {
    throw ValidationError("Reference hole diameter must be non-negative.");
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
    if (section.md_end_m > final_well_md_m) {
      throw ValidationError("String section measured depth cannot exceed the final well MD.");
    }
    previous_section_end_m = section.md_end_m;
  }

  for (const auto& spec : centralizers) {
    spec.validate();
    if (reference_hole_diameter_m > 0.0 && spec.outer_diameter_m > reference_hole_diameter_m) {
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

  const auto trajectory_summary = input.well.summary();
  const auto geometry_nodes = input.well.derived_geometry();
  const auto string_summary = summarize_string(input.string_sections);
  const auto centralizer_summary = summarize_centralizers(
      input.centralizers,
      string_summary.total_length_m,
      input.reference_hole_diameter_m);
  const auto section_summaries =
      summarize_sections(input.string_sections, input.reference_hole_diameter_m);
  const Scalar minimum_nominal_clearance_m =
      input.centralizers.empty()
          ? minimum_section_clearance_m(section_summaries)
          : std::min(
                minimum_section_clearance_m(section_summaries),
                centralizer_summary.min_nominal_radial_clearance_m);
  const Scalar curvature_factor =
      1.0 + (250.0 * trajectory_summary.max_curvature_rad_per_m);
  const Scalar lateral_factor =
      trajectory_summary.final_measured_depth_m > 0.0
          ? 1.0 +
                (0.5 * trajectory_summary.lateral_displacement_m /
                 trajectory_summary.final_measured_depth_m)
          : 1.0;
  const Scalar reference_radius_m =
      0.5 * std::max(string_summary.max_outer_diameter_m, 0.0);

  SolverStubResult result;
  result.status = "phase2-baseline";
  result.message =
      "Phase 2 geometry baseline. Outputs derive from survey and section geometry only, "
      "not from a stiff-string or torque and drag solver.";
  result.geometry_is_approximate = true;
  result.trajectory_summary = trajectory_summary;
  result.string_summary = string_summary;
  result.centralizer_summary = centralizer_summary;
  result.section_summaries = section_summaries;
  result.estimated_hookload_n = string_summary.total_weight_n;
  result.estimated_surface_torque_n_m =
      string_summary.total_weight_n * string_summary.average_friction_coefficient *
      curvature_factor * lateral_factor * reference_radius_m;
  result.minimum_nominal_radial_clearance_m = minimum_nominal_clearance_m;
  result.minimum_standoff_ratio =
      input.reference_hole_diameter_m > 0.0 && centralizer_summary.max_outer_diameter_m > 0.0
          ? std::min<Scalar>(
                1.0,
                centralizer_summary.max_outer_diameter_m / input.reference_hole_diameter_m)
          : 0.0;
  result.contact_nodes = curvature_risk_count(geometry_nodes);
  result.warnings = {
      "Phase 2 trajectory coordinates are approximated by balanced-tangent integration of MD, "
      "inclination, and azimuth.",
      "Nominal radial clearance is purely geometric and does not represent real contact or "
      "standoff.",
      "contact_nodes is a curvature-based risk counter, not real contact detection.",
      "estimated_surface_torque_n_m is a geometry-derived placeholder, not a torque and drag "
      "calculation.",
  };
  if (input.reference_hole_diameter_m <= 0.0) {
    result.warnings.push_back(
        "Reference hole diameter is absent, so nominal clearance metrics are reported as zero.");
  }
  result.todos = {
      "TODO: implement stiff-string formulation.",
      "TODO: implement contact detection and reaction forces.",
      "TODO: implement real side force prediction.",
      "TODO: implement real standoff evaluation from contact geometry.",
      "TODO: implement real torque and drag calculations.",
      "TODO: implement design-space optimization workflow.",
  };

  return result;
}

}  // namespace centraltd
