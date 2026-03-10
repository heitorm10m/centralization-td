#pragma once

#include "centraltd/types.hpp"

#include <string>

namespace centraltd {

struct CentralizerSpec {
  std::string name;
  Scalar outer_diameter_m{0.0};
  Scalar start_md_m{0.0};
  Scalar spacing_m{0.0};
  std::size_t count{0};
  std::string type{"bow-spring"};

  void validate() const;
};

}  // namespace centraltd

