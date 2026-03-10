#pragma once

#include "centraltd/types.hpp"

#include <optional>
#include <string>
#include <vector>

namespace centraltd {

struct CentralizerSpec {
  std::string name;
  std::string type{"bow-spring"};
  Scalar outer_diameter_m{0.0};
  Scalar nominal_restoring_force_n{0.0};
  Scalar nominal_running_force_n{0.0};
  Scalar spacing_hint_m{0.0};
  std::optional<std::size_t> count_hint;
  std::vector<Scalar> installation_md_m;

  std::size_t explicit_installation_count() const noexcept;
  void validate() const;
};

}  // namespace centraltd
