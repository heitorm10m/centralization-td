#include "centraltd/centralizer.hpp"

#include "centraltd/types.hpp"

namespace centraltd {

std::size_t CentralizerSpec::explicit_installation_count() const noexcept {
  return installation_md_m.size();
}

Scalar CentralizerSpec::resolved_support_outer_diameter_m() const noexcept {
  return support_outer_diameter_m > 0.0 ? support_outer_diameter_m : outer_diameter_m;
}

Scalar CentralizerSpec::effective_contact_diameter_m() const noexcept {
  Scalar diameter_m = resolved_support_outer_diameter_m();
  if (min_contact_diameter_m.has_value()) {
    diameter_m = std::max(diameter_m, min_contact_diameter_m.value());
  }
  if (max_contact_diameter_m.has_value()) {
    diameter_m = std::min(diameter_m, max_contact_diameter_m.value());
  }
  return diameter_m;
}

void CentralizerSpec::validate() const {
  if (name.empty()) {
    throw ValidationError("Centralizer name is required.");
  }
  if (type.empty()) {
    throw ValidationError("Centralizer type is required.");
  }
  if (resolved_support_outer_diameter_m() <= 0.0) {
    throw ValidationError("Centralizer support outer diameter must be positive.");
  }
  if (number_of_bows == 0U) {
    throw ValidationError("Centralizer number_of_bows must be at least one.");
  }
  if (inner_clearance_to_pipe_m < 0.0) {
    throw ValidationError("Centralizer inner_clearance_to_pipe_m must be non-negative.");
  }
  if (nominal_restoring_force_n <= 0.0) {
    throw ValidationError("Centralizer nominal restoring force must be positive.");
  }
  if (nominal_running_force_n <= 0.0) {
    throw ValidationError("Centralizer nominal running force must be positive.");
  }
  if (blade_power_law_k.has_value() && blade_power_law_k.value() <= 0.0) {
    throw ValidationError("Centralizer blade_power_law_k must be positive when provided.");
  }
  if (blade_power_law_p <= 0.0) {
    throw ValidationError("Centralizer blade_power_law_p must be positive.");
  }
  if (min_contact_diameter_m.has_value() && min_contact_diameter_m.value() <= 0.0) {
    throw ValidationError("Centralizer min_contact_diameter_m must be positive when provided.");
  }
  if (max_contact_diameter_m.has_value() && max_contact_diameter_m.value() <= 0.0) {
    throw ValidationError("Centralizer max_contact_diameter_m must be positive when provided.");
  }
  if (min_contact_diameter_m.has_value() &&
      max_contact_diameter_m.has_value() &&
      min_contact_diameter_m.value() > max_contact_diameter_m.value()) {
    throw ValidationError(
        "Centralizer min_contact_diameter_m cannot exceed max_contact_diameter_m.");
  }
  if (spacing_hint_m <= 0.0) {
    throw ValidationError("Centralizer spacing hint must be positive.");
  }
  if (count_hint.has_value() && count_hint.value() == 0U) {
    throw ValidationError("Centralizer count hint must be at least one when provided.");
  }

  Scalar previous_md = -1.0;
  for (const Scalar installation_md : installation_md_m) {
    if (installation_md < 0.0) {
      throw ValidationError("Centralizer installation measured depths must be non-negative.");
    }
    if (installation_md <= previous_md) {
      throw ValidationError(
          "Centralizer installation measured depths must be strictly increasing.");
    }
    previous_md = installation_md;
  }
}

}  // namespace centraltd
