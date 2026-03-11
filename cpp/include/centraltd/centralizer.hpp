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
  Scalar support_outer_diameter_m{0.0};
  std::size_t number_of_bows{6U};
  Scalar angular_orientation_reference_deg{0.0};
  Scalar inner_clearance_to_pipe_m{0.0};
  Scalar nominal_restoring_force_n{0.0};
  Scalar nominal_running_force_n{0.0};
  std::optional<Scalar> blade_power_law_k;
  Scalar blade_power_law_p{1.0};
  std::optional<Scalar> min_contact_diameter_m;
  std::optional<Scalar> max_contact_diameter_m;
  Scalar spacing_hint_m{0.0};
  std::optional<std::size_t> count_hint;
  std::vector<Scalar> installation_md_m;

  std::size_t explicit_installation_count() const noexcept;
  Scalar resolved_support_outer_diameter_m() const noexcept;
  Scalar effective_contact_diameter_m() const noexcept;
  void validate() const;
};

}  // namespace centraltd
