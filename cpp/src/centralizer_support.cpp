#include "centraltd/centralizer_support.hpp"

#include "centraltd/bow_spring_constitutive_model.hpp"
#include "centraltd/bow_spring_geometry.hpp"

#include <algorithm>

namespace centraltd {
namespace {

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

  for (const auto& placement : placements) {
    const Scalar proximity_weight = placement_proximity_weight(
        placement,
        segment.measured_depth_center_m,
        segment.segment_length_m);
    if (proximity_weight <= 0.0) {
      continue;
    }

    effect.nearby_centralizer_count += 1U;
    effect.support_present = true;
    effect.support_outer_diameter_m =
        std::max(effect.support_outer_diameter_m, centralizer_effective_contact_diameter_m(placement));
    effect.centering_stiffness_n_per_m +=
        proximity_weight *
        equivalent_bow_support_stiffness_n_per_m(placement, hole_radius_m);
    const Scalar placement_support_contact_clearance_m =
        bow_contact_onset_clearance_m(placement, hole_radius_m);
    effect.support_contact_clearance_m =
        effect.nearby_centralizer_count == 1U
            ? placement_support_contact_clearance_m
            : std::min(effect.support_contact_clearance_m, placement_support_contact_clearance_m);
  }

  if (effect.support_present) {
    effect.support_contact_penalty_n_per_m = contact_penalty_scale * std::max(
        structural_lateral_stiffness_n_per_m + effect.centering_stiffness_n_per_m,
        kMinimumPenaltyStiffnessNPerM);
  }

  return effect;
}

}  // namespace centraltd
