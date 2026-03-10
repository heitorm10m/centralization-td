#pragma once

#include <string>

namespace centraltd {

inline constexpr int kVersionMajor = 0;
inline constexpr int kVersionMinor = 1;
inline constexpr int kVersionPatch = 0;

std::string version_string();

}  // namespace centraltd

