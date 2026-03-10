#include "centraltd/discretization.hpp"

#include "centraltd/types.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace centraltd {
namespace {

std::array<Scalar, 3> tangent_from_angles(Scalar inclination_rad, Scalar azimuth_rad) {
  return {
      std::sin(inclination_rad) * std::cos(azimuth_rad),
      std::sin(inclination_rad) * std::sin(azimuth_rad),
      std::cos(inclination_rad),
  };
}

Scalar clamp_scalar(Scalar value, Scalar lower, Scalar upper) {
  return std::max(lower, std::min(value, upper));
}

Scalar angle_between(
    const std::array<Scalar, 3>& lhs,
    const std::array<Scalar, 3>& rhs) {
  const Scalar dot_product =
      (lhs[0] * rhs[0]) + (lhs[1] * rhs[1]) + (lhs[2] * rhs[2]);
  const Scalar lhs_norm = std::sqrt((lhs[0] * lhs[0]) + (lhs[1] * lhs[1]) + (lhs[2] * lhs[2]));
  const Scalar rhs_norm = std::sqrt((rhs[0] * rhs[0]) + (rhs[1] * rhs[1]) + (rhs[2] * rhs[2]));
  if (lhs_norm <= 0.0 || rhs_norm <= 0.0) {
    return 0.0;
  }

  return std::acos(clamp_scalar(dot_product / (lhs_norm * rhs_norm), -1.0, 1.0));
}

const StringSection& section_for_md(
    const std::vector<StringSection>& sections,
    Scalar measured_depth_m,
    Index* section_index) {
  for (Index index = 0; index < sections.size(); ++index) {
    if (sections.at(index).contains_md(measured_depth_m)) {
      *section_index = index;
      return sections.at(index);
    }
  }

  throw ValidationError("No string section covers the requested measured depth.");
}

std::vector<Scalar> evenly_spaced_positions(
    Scalar start_md_m,
    Scalar end_md_m,
    std::size_t count) {
  std::vector<Scalar> positions;
  if (count == 0U || end_md_m <= start_md_m) {
    return positions;
  }

  positions.reserve(count);
  const Scalar spacing_m = (end_md_m - start_md_m) / static_cast<Scalar>(count + 1U);
  for (std::size_t index = 0; index < count; ++index) {
    positions.push_back(start_md_m + (spacing_m * static_cast<Scalar>(index + 1U)));
  }
  return positions;
}

}  // namespace

void DiscretizationSettings::validate() const {
  if (target_segment_length_m <= 0.0) {
    throw ValidationError("Discretization step must be positive.");
  }
  if (global_solver_max_iterations == 0U) {
    throw ValidationError("Global solver iteration count must be at least one.");
  }
  if (contact_penalty_scale <= 0.0) {
    throw ValidationError("Contact penalty scale must be positive.");
  }
}

std::vector<CentralizerPlacement> expand_centralizer_placements(
    const std::vector<CentralizerSpec>& centralizers,
    Scalar coverage_start_md_m,
    Scalar coverage_end_md_m) {
  std::vector<CentralizerPlacement> placements;
  if (coverage_end_md_m <= coverage_start_md_m) {
    return placements;
  }

  for (const auto& spec : centralizers) {
    std::vector<Scalar> positions;
    if (!spec.installation_md_m.empty()) {
      positions = spec.installation_md_m;
    } else if (spec.count_hint.has_value()) {
      positions = evenly_spaced_positions(
          coverage_start_md_m,
          coverage_end_md_m,
          spec.count_hint.value());
    } else {
      for (Scalar md_m = coverage_start_md_m + (0.5 * spec.spacing_hint_m);
           md_m < coverage_end_md_m;
           md_m += spec.spacing_hint_m) {
        positions.push_back(md_m);
      }
    }

    for (const Scalar measured_depth_m : positions) {
      placements.push_back(CentralizerPlacement{
          spec.name,
          spec.type,
          measured_depth_m,
          spec.outer_diameter_m,
          spec.nominal_restoring_force_n,
          spec.nominal_running_force_n,
          spec.spacing_hint_m,
      });
    }
  }

  std::sort(
      placements.begin(),
      placements.end(),
      [](const CentralizerPlacement& lhs, const CentralizerPlacement& rhs) {
        return lhs.measured_depth_m < rhs.measured_depth_m;
      });

  return placements;
}

DiscretizedProblem discretize_problem(
    const WellTrajectory& well,
    Scalar reference_hole_diameter_m,
    Scalar fluid_density_kg_per_m3,
    const DiscretizationSettings& settings,
    const std::vector<StringSection>& string_sections,
    const std::vector<CentralizerSpec>& centralizers) {
  settings.validate();
  if (string_sections.empty()) {
    throw ValidationError("Discretization requires at least one string section.");
  }

  const Scalar coverage_start_md_m = string_sections.front().md_start_m;
  const Scalar coverage_end_md_m = string_sections.back().md_end_m;
  if (coverage_end_md_m <= coverage_start_md_m) {
    throw ValidationError("String coverage must span a positive MD interval.");
  }

  DiscretizedProblem problem;
  problem.settings = settings;
  problem.trajectory_summary = well.summary();
  problem.coverage_start_md_m = coverage_start_md_m;
  problem.coverage_end_md_m = coverage_end_md_m;
  problem.centralizer_placements =
      expand_centralizer_placements(centralizers, coverage_start_md_m, coverage_end_md_m);

  for (Scalar segment_start_md_m = coverage_start_md_m; segment_start_md_m < coverage_end_md_m;) {
    const Scalar segment_end_md_m =
        std::min(segment_start_md_m + settings.target_segment_length_m, coverage_end_md_m);
    const Scalar center_md_m = 0.5 * (segment_start_md_m + segment_end_md_m);
    const Scalar segment_length_m = segment_end_md_m - segment_start_md_m;

    const auto start_point = well.interpolate(segment_start_md_m);
    const auto center_point = well.interpolate(center_md_m);
    const auto end_point = well.interpolate(segment_end_md_m);
    const auto start_tangent = tangent_from_angles(start_point.inclination_rad, start_point.azimuth_rad);
    const auto end_tangent = tangent_from_angles(end_point.inclination_rad, end_point.azimuth_rad);
    const Scalar curvature_rad_per_m =
        segment_length_m > 0.0 ? angle_between(start_tangent, end_tangent) / segment_length_m : 0.0;

    Index section_index = 0U;
    const auto& section = section_for_md(string_sections, center_md_m, &section_index);

    MechanicalSegmentInput segment;
    segment.segment_index = problem.segments.size();
    segment.section_index = section_index;
    segment.measured_depth_start_m = segment_start_md_m;
    segment.measured_depth_end_m = segment_end_md_m;
    segment.measured_depth_center_m = center_md_m;
    segment.segment_length_m = segment_length_m;
    segment.inclination_rad = center_point.inclination_rad;
    segment.azimuth_rad = center_point.azimuth_rad;
    segment.curvature_rad_per_m = curvature_rad_per_m;
    segment.tvd_m = center_point.tvd_m;
    segment.northing_m = center_point.northing_m;
    segment.easting_m = center_point.easting_m;
    segment.reference_hole_diameter_m = reference_hole_diameter_m;
    segment.fluid_density_kg_per_m3 = fluid_density_kg_per_m3;
    segment.section = section;
    segment.effective_line_weight_n_per_m =
        section.effective_line_weight_n_per_m(fluid_density_kg_per_m3);
    segment.second_moment_of_area_m4 = section.second_moment_of_area_m4();
    segment.bending_stiffness_n_m2 = section.bending_stiffness_n_m2();

    problem.segments.push_back(std::move(segment));
    segment_start_md_m = segment_end_md_m;
  }

  return problem;
}

}  // namespace centraltd
