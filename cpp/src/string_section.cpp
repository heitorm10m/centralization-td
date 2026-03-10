#include "centraltd/string_section.hpp"

#include "centraltd/types.hpp"

namespace centraltd {

void StringSection::validate() const {
  if (name.empty()) {
    throw ValidationError("String section name is required.");
  }
  if (length_m <= 0.0) {
    throw ValidationError("String section length must be positive.");
  }
  if (outer_diameter_m <= 0.0 || inner_diameter_m <= 0.0) {
    throw ValidationError("String section diameters must be positive.");
  }
  if (inner_diameter_m >= outer_diameter_m) {
    throw ValidationError("String section inner diameter must be smaller than outer diameter.");
  }
  if (unit_weight_n_per_m <= 0.0) {
    throw ValidationError("String section unit weight must be positive.");
  }
}

}  // namespace centraltd

