#pragma once

#include "centraltd/discretization.hpp"

#include <vector>

namespace centraltd {

struct BowDirection {
  Index bow_index{0};
  Scalar angle_rad{0.0};
  Vector2 radial_direction_n_b{1.0, 0.0};
};

Scalar placement_proximity_weight(
    const CentralizerPlacement& placement,
    Scalar segment_center_md_m,
    Scalar segment_length_m);

Scalar centralizer_effective_contact_diameter_m(const CentralizerPlacement& placement);

Scalar centralizer_support_clearance_m(
    const CentralizerPlacement& placement,
    Scalar hole_radius_m);

Scalar bow_contact_onset_clearance_m(
    const CentralizerPlacement& placement,
    Scalar hole_radius_m);

std::vector<BowDirection> build_bow_directions(const CentralizerPlacement& placement);

Scalar bow_projection_m(
    const Vector2& eccentricity_n_b_m,
    const BowDirection& bow_direction);

Scalar bow_deflection_m(
    const CentralizerPlacement& placement,
    Scalar hole_radius_m,
    const Vector2& eccentricity_n_b_m,
    const BowDirection& bow_direction);

}  // namespace centraltd
