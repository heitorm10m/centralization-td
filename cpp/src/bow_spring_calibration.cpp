#include "centraltd/bow_spring_calibration.hpp"

#include <algorithm>
#include <cmath>

namespace centraltd {
namespace {

constexpr Scalar kMinimumPositiveValue = 1.0e-12;

BowSpringCalibrationResult invalid_result(const std::string& message) {
  BowSpringCalibrationResult result;
  result.warnings.push_back(message);
  return result;
}

BowSpringCalibrationResult finalize_result(
    BowSpringCalibrationResult result,
    const std::vector<BowSpringCalibrationPoint>& points) {
  result.point_count = points.size();
  result.evaluation_points.reserve(points.size());

  Scalar squared_error_sum_n2 = 0.0;
  Scalar absolute_error_sum_n = 0.0;
  Scalar absolute_relative_error_sum = 0.0;
  for (const auto& point : points) {
    const Scalar predicted_force_n =
        result.blade_power_law_k * std::pow(point.deflection_m, result.blade_power_law_p);
    const Scalar absolute_error_n = std::abs(predicted_force_n - point.force_n);
    const Scalar relative_error =
        point.force_n > kMinimumPositiveValue ? absolute_error_n / point.force_n : 0.0;
    squared_error_sum_n2 += absolute_error_n * absolute_error_n;
    absolute_error_sum_n += absolute_error_n;
    absolute_relative_error_sum += relative_error;
    result.evaluation_points.push_back(BowSpringCalibrationEvaluationPoint{
        point.deflection_m,
        point.force_n,
        predicted_force_n,
        absolute_error_n,
        relative_error,
    });
  }

  if (!points.empty()) {
    result.rmse_force_n = std::sqrt(squared_error_sum_n2 / static_cast<Scalar>(points.size()));
    result.mean_absolute_error_n = absolute_error_sum_n / static_cast<Scalar>(points.size());
    result.mean_absolute_relative_error =
        absolute_relative_error_sum / static_cast<Scalar>(points.size());
  }

  return result;
}

}  // namespace

BowSpringCalibrationResult calibrate_bow_spring_power_law(
    const std::vector<BowSpringCalibrationPoint>& points,
    std::optional<Scalar> fixed_blade_power_law_p) {
  if (points.empty()) {
    return invalid_result("At least one deflection-force point is required for calibration.");
  }
  for (const auto& point : points) {
    if (point.deflection_m <= 0.0) {
      return invalid_result("Calibration deflections must be strictly positive.");
    }
    if (point.force_n <= 0.0) {
      return invalid_result("Calibration forces must be strictly positive.");
    }
  }
  if (fixed_blade_power_law_p.has_value() && fixed_blade_power_law_p.value() <= 0.0) {
    return invalid_result("The fixed blade power-law exponent must be positive.");
  }

  BowSpringCalibrationResult result;
  if (fixed_blade_power_law_p.has_value()) {
    const Scalar resolved_p = fixed_blade_power_law_p.value();
    Scalar numerator = 0.0;
    Scalar denominator = 0.0;
    for (const auto& point : points) {
      const Scalar transformed_deflection = std::pow(point.deflection_m, resolved_p);
      numerator += point.force_n * transformed_deflection;
      denominator += transformed_deflection * transformed_deflection;
    }
    if (denominator <= kMinimumPositiveValue) {
      return invalid_result("Fixed-exponent calibration encountered a near-singular fit.");
    }
    result.status = "calibrated";
    result.blade_power_law_k = numerator / denominator;
    result.blade_power_law_p = resolved_p;
    result.fitted_parameter_count = 1U;
    return finalize_result(std::move(result), points);
  }

  if (points.size() == 1U) {
    result.status = "calibrated_with_assumed_p";
    result.blade_power_law_p = 1.0;
    result.blade_power_law_k = points.front().force_n / points.front().deflection_m;
    result.fitted_parameter_count = 1U;
    result.warnings.push_back(
        "Only one calibration point was supplied, so p was assumed to be 1.0.");
    return finalize_result(std::move(result), points);
  }

  Scalar x_mean = 0.0;
  Scalar y_mean = 0.0;
  for (const auto& point : points) {
    x_mean += std::log(point.deflection_m);
    y_mean += std::log(point.force_n);
  }
  x_mean /= static_cast<Scalar>(points.size());
  y_mean /= static_cast<Scalar>(points.size());

  Scalar x_variance = 0.0;
  Scalar covariance = 0.0;
  for (const auto& point : points) {
    const Scalar x = std::log(point.deflection_m) - x_mean;
    const Scalar y = std::log(point.force_n) - y_mean;
    x_variance += x * x;
    covariance += x * y;
  }
  if (x_variance <= kMinimumPositiveValue) {
    return invalid_result(
        "Calibration deflections must contain at least two distinct positive values.");
  }

  result.status = "calibrated";
  result.blade_power_law_p = covariance / x_variance;
  result.blade_power_law_k = std::exp(y_mean - (result.blade_power_law_p * x_mean));
  result.fitted_parameter_count = 2U;
  return finalize_result(std::move(result), points);
}

BowSpringCalibrationResult calibrate_bow_spring_from_nominal_points(
    Scalar nominal_restoring_force_n,
    Scalar restoring_deflection_m,
    std::optional<Scalar> nominal_running_force_n,
    std::optional<Scalar> running_deflection_m,
    std::optional<Scalar> fixed_blade_power_law_p) {
  if (nominal_restoring_force_n <= 0.0 || restoring_deflection_m <= 0.0) {
    return invalid_result(
        "Nominal restoring force and restoring deflection must both be strictly positive.");
  }

  const bool has_running_force = nominal_running_force_n.has_value();
  const bool has_running_deflection = running_deflection_m.has_value();
  if (has_running_force != has_running_deflection) {
    return invalid_result(
        "Running-force calibration requires both running force and running deflection.");
  }

  std::vector<BowSpringCalibrationPoint> points = {
      {restoring_deflection_m, nominal_restoring_force_n},
  };
  if (has_running_force) {
    points.push_back(BowSpringCalibrationPoint{
        running_deflection_m.value(),
        nominal_running_force_n.value(),
    });
  }

  auto result = calibrate_bow_spring_power_law(points, fixed_blade_power_law_p);
  if (has_running_force) {
    result.warnings.push_back(
        "Running-force calibration is treated as a reduced proxy point on the same power-law curve.");
  } else if (!fixed_blade_power_law_p.has_value()) {
    result.warnings.push_back(
        "Only a restoring-force point was supplied, so the nominal-point fit remains underdetermined without an assumed p.");
  }
  return result;
}

}  // namespace centraltd
