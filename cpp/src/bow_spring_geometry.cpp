#include "centraltd/bow_spring_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace centraltd {
namespace {

constexpr Scalar kPi = 3.14159265358979323846;

}  // namespace

Scalar placement_proximity_weight(
    const CentralizerPlacement& placement,
    Scalar segment_center_md_m,
    Scalar segment_length_m) {
  const Scalar half_influence_length_m =
      std::max(0.5 * placement.influence_length_m, 0.5 * segment_length_m);
  if (half_influence_length_m <= 0.0) {
    return 0.0;
  }

  const Scalar distance_to_segment_center_m =
      std::abs(placement.measured_depth_m - segment_center_md_m);
  if (distance_to_segment_center_m > half_influence_length_m) {
    return 0.0;
  }

  return std::max(0.0, 1.0 - (distance_to_segment_center_m / half_influence_length_m));
}

Scalar centralizer_effective_contact_diameter_m(const CentralizerPlacement& placement) {
  Scalar diameter_m = placement.support_outer_diameter_m;
  if (placement.min_contact_diameter_m.has_value()) {
    diameter_m = std::max(diameter_m, placement.min_contact_diameter_m.value());
  }
  if (placement.max_contact_diameter_m.has_value()) {
    diameter_m = std::min(diameter_m, placement.max_contact_diameter_m.value());
  }
  return diameter_m;
}

Scalar centralizer_support_clearance_m(
    const CentralizerPlacement& placement,
    Scalar hole_radius_m) {
  return std::max(
      0.0,
      hole_radius_m - (0.5 * centralizer_effective_contact_diameter_m(placement)));
}

Scalar bow_contact_onset_clearance_m(
    const CentralizerPlacement& placement,
    Scalar hole_radius_m) {
  return placement.inner_clearance_to_pipe_m +
         centralizer_support_clearance_m(placement, hole_radius_m);
}

std::vector<BowDirection> build_bow_directions(const CentralizerPlacement& placement) {
  std::vector<BowDirection> directions;
  directions.reserve(placement.number_of_bows);

  const Scalar angle_reference_rad =
      placement.angular_orientation_reference_deg * kPi / 180.0;
  const Scalar angle_increment_rad =
      (2.0 * kPi) / static_cast<Scalar>(placement.number_of_bows);

  for (Index bow_index = 0; bow_index < placement.number_of_bows; ++bow_index) {
    const Scalar angle_rad =
        angle_reference_rad + (static_cast<Scalar>(bow_index) * angle_increment_rad);
    directions.push_back(BowDirection{
        bow_index,
        angle_rad,
        {std::cos(angle_rad), std::sin(angle_rad)},
    });
  }

  return directions;
}

Scalar bow_projection_m(
    const Vector2& eccentricity_n_b_m,
    const BowDirection& bow_direction) {
  return (eccentricity_n_b_m[0] * bow_direction.radial_direction_n_b[0]) +
         (eccentricity_n_b_m[1] * bow_direction.radial_direction_n_b[1]);
}

Scalar bow_deflection_m(
    const CentralizerPlacement& placement,
    Scalar hole_radius_m,
    const Vector2& eccentricity_n_b_m,
    const BowDirection& bow_direction) {
  return std::max(
      0.0,
      bow_projection_m(eccentricity_n_b_m, bow_direction) -
          bow_contact_onset_clearance_m(placement, hole_radius_m));
}

}  // namespace centraltd
