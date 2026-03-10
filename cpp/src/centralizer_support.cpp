#include "centraltd/centralizer_support.hpp"

#include <algorithm>
#include <cmath>

namespace centraltd {
namespace {

constexpr Scalar kMinimumReferenceDeflectionM = 1.0e-4;
constexpr Scalar kMinimumPenaltyStiffnessNPerM = 1.0;

}  // namespace

CentralizerSupportEffect evaluate_centralizer_support_effect(
    const MechanicalSegmentInput& segment,
    const std::vector<CentralizerPlacement>& placements,
    Scalar structural_lateral_stiffness_n_per_m,
    Scalar contact_penalty_scale) {
  CentralizerSupportEffect effect;
  effect.support_outer_diameter_m = segment.section.outer_diameter_m;

  if (segment.reference_hole_diameter_m <= 0.0) {
    return effect;
  }

  const Scalar hole_radius_m = 0.5 * segment.reference_hole_diameter_m;
  const Scalar pipe_radius_m = segment.section.outer_radius_m();
  effect.pipe_body_clearance_m = std::max(0.0, hole_radius_m - pipe_radius_m);

  const Scalar reference_deflection_m = std::max(
      effect.pipe_body_clearance_m,
      std::max(0.05 * segment.section.outer_diameter_m, kMinimumReferenceDeflectionM));

  for (const auto& placement : placements) {
    const Scalar half_influence_length_m =
        std::max(0.5 * placement.influence_length_m, 0.5 * segment.segment_length_m);
    const Scalar distance_to_segment_center_m =
        std::abs(placement.measured_depth_m - segment.measured_depth_center_m);
    if (distance_to_segment_center_m > half_influence_length_m) {
      continue;
    }

    const Scalar proximity_weight =
        std::max(0.0, 1.0 - (distance_to_segment_center_m / half_influence_length_m));
    effect.nearby_centralizer_count += 1U;
    effect.support_present = true;
    effect.support_outer_diameter_m =
        std::max(effect.support_outer_diameter_m, placement.outer_diameter_m);
    effect.centering_stiffness_n_per_m +=
        proximity_weight * placement.nominal_restoring_force_n / reference_deflection_m;
  }

  effect.support_contact_clearance_m = std::max(
      0.0,
      hole_radius_m - (0.5 * effect.support_outer_diameter_m));

  if (effect.support_present) {
    effect.support_contact_penalty_n_per_m = contact_penalty_scale * std::max(
        structural_lateral_stiffness_n_per_m + effect.centering_stiffness_n_per_m,
        kMinimumPenaltyStiffnessNPerM);
  }

  return effect;
}

}  // namespace centraltd
