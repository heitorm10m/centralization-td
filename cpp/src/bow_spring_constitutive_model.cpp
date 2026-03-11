#include "centraltd/bow_spring_constitutive_model.hpp"

#include <algorithm>
#include <cmath>

namespace centraltd {
namespace {

constexpr Scalar kMinimumReferenceDeflectionM = 1.0e-4;

Scalar reference_loaded_bow_count(const CentralizerPlacement& placement) {
  return std::max<Scalar>(
      1.0,
      std::ceil(0.5 * static_cast<Scalar>(placement.number_of_bows)));
}

}  // namespace

Scalar bow_reference_deflection_m(
    const CentralizerPlacement& placement,
    Scalar hole_radius_m) {
  return std::max(
      bow_contact_onset_clearance_m(placement, hole_radius_m),
      std::max(
          0.05 * centralizer_effective_contact_diameter_m(placement),
          kMinimumReferenceDeflectionM));
}

Scalar resolved_blade_power_law_k(
    const CentralizerPlacement& placement,
    Scalar hole_radius_m) {
  if (placement.blade_power_law_k.has_value()) {
    return placement.blade_power_law_k.value();
  }

  const Scalar reference_deflection_m = bow_reference_deflection_m(placement, hole_radius_m);
  return placement.nominal_restoring_force_n /
         (reference_loaded_bow_count(placement) *
          std::pow(reference_deflection_m, placement.blade_power_law_p));
}

Scalar bow_force_magnitude_n(
    const CentralizerPlacement& placement,
    Scalar hole_radius_m,
    Scalar deflection_m) {
  if (deflection_m <= 0.0) {
    return 0.0;
  }

  return resolved_blade_power_law_k(placement, hole_radius_m) *
         std::pow(deflection_m, placement.blade_power_law_p);
}

Scalar equivalent_bow_support_stiffness_n_per_m(
    const CentralizerPlacement& placement,
    Scalar hole_radius_m) {
  const Scalar reference_deflection_m = bow_reference_deflection_m(placement, hole_radius_m);
  const Scalar tangent_stiffness_per_bow_n_per_m =
      resolved_blade_power_law_k(placement, hole_radius_m) *
      placement.blade_power_law_p *
      std::pow(reference_deflection_m, placement.blade_power_law_p - 1.0);
  return reference_loaded_bow_count(placement) * tangent_stiffness_per_bow_n_per_m;
}

Scalar centralizer_running_force_ratio(const CentralizerPlacement& placement) {
  if (placement.nominal_restoring_force_n <= 0.0) {
    return 0.0;
  }
  return std::max(0.0, placement.nominal_running_force_n / placement.nominal_restoring_force_n);
}

}  // namespace centraltd
