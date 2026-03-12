#pragma once

#include "centraltd/bow_spring_geometry.hpp"

namespace centraltd {

Scalar bow_reference_deflection_m(
    const CentralizerPlacement& placement,
    Scalar hole_radius_m);

Scalar resolved_blade_power_law_k(
    const CentralizerPlacement& placement,
    Scalar hole_radius_m);

Scalar bow_force_magnitude_n(
    const CentralizerPlacement& placement,
    Scalar hole_radius_m,
    Scalar deflection_m);

Scalar equivalent_bow_support_stiffness_n_per_m(
    const CentralizerPlacement& placement,
    Scalar hole_radius_m);

Scalar centralizer_running_force_ratio(const CentralizerPlacement& placement);
Scalar centralizer_axial_force_ratio(const CentralizerPlacement& placement);
Scalar centralizer_tangential_force_ratio(const CentralizerPlacement& placement);

}  // namespace centraltd
