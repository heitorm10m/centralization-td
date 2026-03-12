#pragma once

#include "centraltd/types.hpp"

#include <optional>
#include <string>
#include <vector>

namespace centraltd {

struct BowSpringCalibrationPoint {
  Scalar deflection_m{0.0};
  Scalar force_n{0.0};
};

struct BowSpringCalibrationEvaluationPoint {
  Scalar deflection_m{0.0};
  Scalar measured_force_n{0.0};
  Scalar predicted_force_n{0.0};
  Scalar absolute_error_n{0.0};
  Scalar relative_error{0.0};
};

struct BowSpringCalibrationResult {
  std::string status{"invalid_input"};
  Scalar blade_power_law_k{0.0};
  Scalar blade_power_law_p{1.0};
  std::size_t point_count{0};
  std::size_t fitted_parameter_count{0};
  Scalar rmse_force_n{0.0};
  Scalar mean_absolute_error_n{0.0};
  Scalar mean_absolute_relative_error{0.0};
  std::vector<BowSpringCalibrationEvaluationPoint> evaluation_points;
  std::vector<std::string> warnings;
};

BowSpringCalibrationResult calibrate_bow_spring_power_law(
    const std::vector<BowSpringCalibrationPoint>& points,
    std::optional<Scalar> fixed_blade_power_law_p = std::nullopt);

BowSpringCalibrationResult calibrate_bow_spring_from_nominal_points(
    Scalar nominal_restoring_force_n,
    Scalar restoring_deflection_m,
    std::optional<Scalar> nominal_running_force_n = std::nullopt,
    std::optional<Scalar> running_deflection_m = std::nullopt,
    std::optional<Scalar> fixed_blade_power_law_p = std::nullopt);

}  // namespace centraltd
