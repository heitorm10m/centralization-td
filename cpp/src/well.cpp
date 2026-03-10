#include "centraltd/well.hpp"

#include "centraltd/types.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace centraltd {
namespace {

constexpr Scalar kPi = 3.14159265358979323846;
constexpr Scalar kTwoPi = 2.0 * kPi;

Scalar clamp_scalar(Scalar value, Scalar lower, Scalar upper) {
  return std::max(lower, std::min(value, upper));
}

std::array<Scalar, 3> tangent_from_angles(Scalar inclination_rad, Scalar azimuth_rad) {
  return {
      std::sin(inclination_rad) * std::cos(azimuth_rad),
      std::sin(inclination_rad) * std::sin(azimuth_rad),
      std::cos(inclination_rad),
  };
}

Scalar vector_norm(const std::array<Scalar, 3>& vector) {
  return std::sqrt(
      (vector[0] * vector[0]) + (vector[1] * vector[1]) + (vector[2] * vector[2]));
}

std::array<Scalar, 3> normalize(const std::array<Scalar, 3>& vector) {
  const Scalar norm = vector_norm(vector);
  if (norm <= 0.0) {
    return {0.0, 0.0, 1.0};
  }

  return {vector[0] / norm, vector[1] / norm, vector[2] / norm};
}

Scalar angle_between(const std::array<Scalar, 3>& lhs, const std::array<Scalar, 3>& rhs) {
  const auto lhs_normalized = normalize(lhs);
  const auto rhs_normalized = normalize(rhs);
  const Scalar dot_product =
      (lhs_normalized[0] * rhs_normalized[0]) +
      (lhs_normalized[1] * rhs_normalized[1]) +
      (lhs_normalized[2] * rhs_normalized[2]);
  return std::acos(clamp_scalar(dot_product, -1.0, 1.0));
}

std::array<Scalar, 3> interpolate_tangent(
    const std::array<Scalar, 3>& lhs,
    const std::array<Scalar, 3>& rhs,
    Scalar ratio) {
  return normalize({
      lhs[0] + (ratio * (rhs[0] - lhs[0])),
      lhs[1] + (ratio * (rhs[1] - lhs[1])),
      lhs[2] + (ratio * (rhs[2] - lhs[2])),
  });
}

WellTrajectoryPoint make_point_from_geometry(
    Scalar measured_depth_m,
    const std::array<Scalar, 3>& tangent,
    Scalar tvd_m,
    Scalar northing_m,
    Scalar easting_m) {
  const auto normalized = normalize(tangent);
  Scalar azimuth_rad = std::atan2(normalized[1], normalized[0]);
  if (azimuth_rad < 0.0) {
    azimuth_rad += kTwoPi;
  }

  WellTrajectoryPoint point;
  point.measured_depth_m = measured_depth_m;
  point.inclination_rad = std::acos(clamp_scalar(normalized[2], -1.0, 1.0));
  point.azimuth_rad = azimuth_rad;
  point.tvd_m = tvd_m;
  point.northing_m = northing_m;
  point.easting_m = easting_m;
  return point;
}

}  // namespace

WellTrajectory::WellTrajectory(std::vector<WellTrajectoryPoint> points)
    : points_(std::move(points)) {}

const std::vector<WellTrajectoryPoint>& WellTrajectory::points() const noexcept {
  return points_;
}

std::size_t WellTrajectory::size() const noexcept {
  return points_.size();
}

bool WellTrajectory::empty() const noexcept {
  return points_.empty();
}

Scalar WellTrajectory::final_measured_depth_m() const noexcept {
  return points_.empty() ? 0.0 : points_.back().measured_depth_m;
}

std::array<Scalar, 3> WellTrajectory::local_tangent(Index point_index) const {
  if (point_index >= points_.size()) {
    throw ValidationError("Well trajectory tangent index is out of range.");
  }

  const auto& point = points_.at(point_index);
  return tangent_from_angles(point.inclination_rad, point.azimuth_rad);
}

Scalar WellTrajectory::discrete_curvature_rad_per_m(Index point_index) const {
  validate();

  if (point_index >= points_.size()) {
    throw ValidationError("Well trajectory curvature index is out of range.");
  }
  if (point_index == 0U) {
    return 0.0;
  }

  const Scalar segment_length_m =
      points_.at(point_index).measured_depth_m - points_.at(point_index - 1U).measured_depth_m;
  if (segment_length_m <= 0.0) {
    return 0.0;
  }

  return angle_between(local_tangent(point_index - 1U), local_tangent(point_index)) /
         segment_length_m;
}

std::vector<TrajectoryGeometryNode> WellTrajectory::derived_geometry() const {
  validate();

  std::vector<TrajectoryGeometryNode> nodes;
  nodes.reserve(points_.size());

  for (Index index = 0; index < points_.size(); ++index) {
    const auto& point = points_.at(index);

    TrajectoryGeometryNode node;
    node.measured_depth_m = point.measured_depth_m;
    node.inclination_rad = point.inclination_rad;
    node.azimuth_rad = point.azimuth_rad;
    node.tangent_north_east_tvd = tangent_from_angles(point.inclination_rad, point.azimuth_rad);
    node.discrete_curvature_rad_per_m = discrete_curvature_rad_per_m(index);

    if (!nodes.empty()) {
      const auto& previous = nodes.back();
      const Scalar segment_length_m = point.measured_depth_m - previous.measured_depth_m;
      node.northing_m =
          previous.northing_m +
          (0.5 * segment_length_m *
           (previous.tangent_north_east_tvd[0] + node.tangent_north_east_tvd[0]));
      node.easting_m =
          previous.easting_m +
          (0.5 * segment_length_m *
           (previous.tangent_north_east_tvd[1] + node.tangent_north_east_tvd[1]));
      node.tvd_m =
          previous.tvd_m +
          (0.5 * segment_length_m *
           (previous.tangent_north_east_tvd[2] + node.tangent_north_east_tvd[2]));
    }

    nodes.push_back(node);
  }

  return nodes;
}

WellTrajectoryPoint WellTrajectory::interpolate(Scalar measured_depth_m) const {
  validate();

  if (measured_depth_m < points_.front().measured_depth_m ||
      measured_depth_m > points_.back().measured_depth_m) {
    throw ValidationError("Interpolation measured depth is outside the trajectory range.");
  }

  const auto nodes = derived_geometry();
  if (measured_depth_m == nodes.front().measured_depth_m) {
    const auto& first = nodes.front();
    return make_point_from_geometry(
        first.measured_depth_m,
        first.tangent_north_east_tvd,
        first.tvd_m,
        first.northing_m,
        first.easting_m);
  }
  if (measured_depth_m == nodes.back().measured_depth_m) {
    const auto& last = nodes.back();
    return make_point_from_geometry(
        last.measured_depth_m,
        last.tangent_north_east_tvd,
        last.tvd_m,
        last.northing_m,
        last.easting_m);
  }

  auto upper = std::lower_bound(
      nodes.begin(),
      nodes.end(),
      measured_depth_m,
      [](const TrajectoryGeometryNode& node, Scalar target_md_m) {
        return node.measured_depth_m < target_md_m;
      });

  const auto lower = std::prev(upper);
  const Scalar segment_length_m = upper->measured_depth_m - lower->measured_depth_m;
  const Scalar ratio =
      segment_length_m > 0.0 ? (measured_depth_m - lower->measured_depth_m) / segment_length_m : 0.0;
  const auto tangent = interpolate_tangent(
      lower->tangent_north_east_tvd,
      upper->tangent_north_east_tvd,
      ratio);

  return make_point_from_geometry(
      measured_depth_m,
      tangent,
      lower->tvd_m + (ratio * (upper->tvd_m - lower->tvd_m)),
      lower->northing_m + (ratio * (upper->northing_m - lower->northing_m)),
      lower->easting_m + (ratio * (upper->easting_m - lower->easting_m)));
}

TrajectorySummary WellTrajectory::summary() const {
  validate();

  const auto nodes = derived_geometry();

  TrajectorySummary result;
  result.point_count = nodes.size();
  result.start_measured_depth_m = nodes.front().measured_depth_m;
  result.final_measured_depth_m = nodes.back().measured_depth_m;
  result.total_course_length_m = result.final_measured_depth_m - result.start_measured_depth_m;
  result.vertical_depth_m = nodes.back().tvd_m;
  result.lateral_displacement_m = std::sqrt(
      (nodes.back().northing_m * nodes.back().northing_m) +
      (nodes.back().easting_m * nodes.back().easting_m));
  result.coordinates_are_approximate = true;

  for (const auto& node : nodes) {
    result.max_inclination_rad = std::max(result.max_inclination_rad, node.inclination_rad);
    result.max_curvature_rad_per_m =
        std::max(result.max_curvature_rad_per_m, node.discrete_curvature_rad_per_m);
  }

  return result;
}

void WellTrajectory::validate() const {
  if (points_.empty()) {
    throw ValidationError("Well trajectory must contain at least one point.");
  }

  Scalar previous_md = -1.0;
  for (const auto& point : points_) {
    if (point.measured_depth_m < 0.0) {
      throw ValidationError("Trajectory measured depth must be non-negative.");
    }
    if (point.inclination_rad < 0.0 || point.inclination_rad > kPi) {
      throw ValidationError("Trajectory inclination must stay within [0, pi] radians.");
    }
    if (point.azimuth_rad < 0.0 || point.azimuth_rad > kTwoPi) {
      throw ValidationError("Trajectory azimuth must stay within [0, 2*pi] radians.");
    }
    if (point.measured_depth_m <= previous_md) {
      throw ValidationError("Trajectory measured depth must be strictly increasing.");
    }
    previous_md = point.measured_depth_m;
  }
}

}  // namespace centraltd
