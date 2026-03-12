#include "centraltd/bow_spring_resultants.hpp"

#include "centraltd/bow_spring_constitutive_model.hpp"

#include <algorithm>
#include <cmath>

namespace centraltd {
namespace {

Scalar vector_norm(const Vector2& vector) {
  return std::sqrt((vector[0] * vector[0]) + (vector[1] * vector[1]));
}

}  // namespace

BowSpringSegmentResult evaluate_bow_spring_segment_result(
    const MechanicalSegmentInput& segment,
    const std::vector<CentralizerPlacement>& placements,
    const Vector2& eccentricity_n_b_m) {
  BowSpringSegmentResult result;
  result.support_outer_diameter_m = segment.section.outer_diameter_m;

  if (segment.reference_hole_diameter_m <= 0.0) {
    return result;
  }

  const Scalar hole_radius_m = 0.5 * segment.reference_hole_diameter_m;

  for (const auto& placement : placements) {
    const Scalar proximity_weight = placement_proximity_weight(
        placement,
        segment.measured_depth_center_m,
        segment.segment_length_m);
    if (proximity_weight <= 0.0) {
      continue;
    }

    result.nearby_centralizer_count += 1U;
    result.support_present = true;
    result.support_outer_diameter_m = std::max(
        result.support_outer_diameter_m,
        centralizer_effective_contact_diameter_m(placement));
    const Scalar placement_support_clearance_m =
        bow_contact_onset_clearance_m(placement, hole_radius_m);
    result.support_contact_clearance_m =
        result.nearby_centralizer_count == 1U
            ? placement_support_clearance_m
            : std::min(result.support_contact_clearance_m, placement_support_clearance_m);
    result.equivalent_centering_stiffness_n_per_m +=
        proximity_weight * equivalent_bow_support_stiffness_n_per_m(placement, hole_radius_m);

    const auto bow_directions = build_bow_directions(placement);
    Vector2 placement_resultant_vector_n_b{0.0, 0.0};
    for (const auto& bow_direction : bow_directions) {
      const Scalar deflection_m = bow_deflection_m(
          placement,
          hole_radius_m,
          eccentricity_n_b_m,
          bow_direction);
      const Scalar force_magnitude_n =
          proximity_weight * bow_force_magnitude_n(placement, hole_radius_m, deflection_m);
      const Vector2 force_vector_n_b{
          force_magnitude_n * bow_direction.radial_direction_n_b[0],
          force_magnitude_n * bow_direction.radial_direction_n_b[1],
      };

      placement_resultant_vector_n_b[0] += force_vector_n_b[0];
      placement_resultant_vector_n_b[1] += force_vector_n_b[1];
      result.bow_force_details.push_back(BowForceDetail{
          placement.source_name,
          placement.measured_depth_m,
          bow_direction.bow_index,
          bow_direction.angle_rad,
          bow_direction.radial_direction_n_b,
          deflection_m,
          force_magnitude_n,
          force_vector_n_b,
      });
    }

    const Scalar placement_resultant_magnitude_n = vector_norm(placement_resultant_vector_n_b);
    const Scalar placement_axial_force_ratio = centralizer_axial_force_ratio(placement);
    const Scalar placement_tangential_force_ratio = centralizer_tangential_force_ratio(placement);
    const Scalar placement_effective_contact_radius_m =
        0.5 * centralizer_effective_contact_diameter_m(placement);

    result.placement_resultant_details.push_back(CentralizerPlacementResultantDetail{
        placement.source_name,
        placement.measured_depth_m,
        placement_effective_contact_radius_m,
        placement_axial_force_ratio,
        placement_tangential_force_ratio,
        placement_resultant_vector_n_b,
        placement_resultant_magnitude_n,
    });
    result.bow_resultant_vector_n_b[0] += placement_resultant_vector_n_b[0];
    result.bow_resultant_vector_n_b[1] += placement_resultant_vector_n_b[1];
    result.centralizer_axial_friction_n +=
        placement_axial_force_ratio * placement_resultant_magnitude_n;
    result.centralizer_tangential_friction_n +=
        placement_tangential_force_ratio * placement_resultant_magnitude_n;
    result.centralizer_torque_increment_n_m +=
        placement_tangential_force_ratio *
        placement_resultant_magnitude_n *
        placement_effective_contact_radius_m;
    result.effective_contact_radius_m = std::max(
        result.effective_contact_radius_m,
        placement_effective_contact_radius_m);
  }

  result.bow_resultant_magnitude_n = vector_norm(result.bow_resultant_vector_n_b);
  return result;
}

}  // namespace centraltd
