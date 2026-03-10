#include "centraltd/centralizer.hpp"

#include "centraltd/types.hpp"

namespace centraltd {

void CentralizerSpec::validate() const {
  if (name.empty()) {
    throw ValidationError("Centralizer name is required.");
  }
  if (outer_diameter_m <= 0.0) {
    throw ValidationError("Centralizer outer diameter must be positive.");
  }
  if (start_md_m < 0.0) {
    throw ValidationError("Centralizer start measured depth must be non-negative.");
  }
  if (spacing_m < 0.0) {
    throw ValidationError("Centralizer spacing must be non-negative.");
  }
  if (count == 0) {
    throw ValidationError("Centralizer count must be at least one.");
  }
}

}  // namespace centraltd

