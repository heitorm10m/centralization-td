#pragma once

#include <cstddef>
#include <stdexcept>

namespace centraltd {

using Scalar = double;
using Index = std::size_t;

inline constexpr Scalar kStandardGravityMPerS2 = 9.80665;

class ValidationError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

}  // namespace centraltd
