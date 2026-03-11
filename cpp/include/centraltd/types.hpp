#pragma once

#include <array>
#include <cstddef>
#include <stdexcept>

namespace centraltd {

using Scalar = double;
using Index = std::size_t;
using Vector2 = std::array<Scalar, 2>;
using Vector3 = std::array<Scalar, 3>;

inline constexpr Scalar kStandardGravityMPerS2 = 9.80665;

class ValidationError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

}  // namespace centraltd
