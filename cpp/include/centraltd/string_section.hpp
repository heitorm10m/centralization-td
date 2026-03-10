#pragma once

#include "centraltd/types.hpp"

#include <string>

namespace centraltd {

struct StringSection {
  std::string name;
  Scalar md_start_m{0.0};
  Scalar md_end_m{0.0};
  Scalar outer_diameter_m{0.0};
  Scalar inner_diameter_m{0.0};
  Scalar linear_weight_n_per_m{0.0};
  Scalar young_modulus_pa{0.0};
  Scalar shear_modulus_pa{0.0};
  Scalar density_kg_per_m3{0.0};
  Scalar friction_coefficient{0.0};

  Scalar length_m() const noexcept;
  bool contains_md(Scalar measured_depth_m) const noexcept;
  void validate() const;
};

}  // namespace centraltd
