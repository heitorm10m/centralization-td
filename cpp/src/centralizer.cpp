#include "centraltd/centralizer.hpp"

#include "centraltd/types.hpp"

namespace centraltd {

std::size_t CentralizerSpec::explicit_installation_count() const noexcept {
  return installation_md_m.size();
}

void CentralizerSpec::validate() const {
  if (name.empty()) {
    throw ValidationError("Centralizer name is required.");
  }
  if (type.empty()) {
    throw ValidationError("Centralizer type is required.");
  }
  if (outer_diameter_m <= 0.0) {
    throw ValidationError("Centralizer outer diameter must be positive.");
  }
  if (nominal_restoring_force_n <= 0.0) {
    throw ValidationError("Centralizer nominal restoring force must be positive.");
  }
  if (nominal_running_force_n <= 0.0) {
    throw ValidationError("Centralizer nominal running force must be positive.");
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
