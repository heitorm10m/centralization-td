#pragma once

#include "centraltd/types.hpp"

#include <string>

namespace centraltd {

struct StringSection {
  std::string name;
  Scalar length_m{0.0};
  Scalar outer_diameter_m{0.0};
  Scalar inner_diameter_m{0.0};
  Scalar unit_weight_n_per_m{0.0};
  std::string grade{"unspecified"};

  void validate() const;
};

}  // namespace centraltd

