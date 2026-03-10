#include "centraltd/string_section.hpp"

#include "centraltd/types.hpp"

namespace centraltd {

Scalar StringSection::length_m() const noexcept {
  return md_end_m - md_start_m;
}

bool StringSection::contains_md(Scalar measured_depth_m) const noexcept {
  return measured_depth_m >= md_start_m && measured_depth_m <= md_end_m;
}

void StringSection::validate() const {
  if (name.empty()) {
    throw ValidationError("String section name is required.");
  }
  if (md_start_m < 0.0) {
    throw ValidationError("String section start measured depth must be non-negative.");
  }
  if (md_end_m <= md_start_m) {
    throw ValidationError("String section end measured depth must be greater than the start.");
  }
  if (outer_diameter_m <= 0.0 || inner_diameter_m <= 0.0) {
    throw ValidationError("String section diameters must be positive.");
  }
  if (inner_diameter_m >= outer_diameter_m) {
    throw ValidationError("String section inner diameter must be smaller than outer diameter.");
  }
  if (linear_weight_n_per_m <= 0.0) {
    throw ValidationError("String section linear weight must be positive.");
  }
  if (young_modulus_pa <= 0.0) {
    throw ValidationError("String section Young modulus must be positive.");
  }
  if (shear_modulus_pa <= 0.0) {
    throw ValidationError("String section shear modulus must be positive.");
  }
  if (density_kg_per_m3 <= 0.0) {
    throw ValidationError("String section density must be positive.");
  }
  if (friction_coefficient < 0.0 || friction_coefficient > 1.5) {
    throw ValidationError("String section friction coefficient must stay within [0, 1.5].");
  }
}

}  // namespace centraltd
