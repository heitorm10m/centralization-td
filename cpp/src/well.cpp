#include "centraltd/well.hpp"

#include "centraltd/types.hpp"

#include <utility>

namespace centraltd {

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

void WellTrajectory::validate() const {
  if (points_.empty()) {
    throw ValidationError("Well trajectory must contain at least one point.");
  }

  Scalar previous_md = -1.0;
  for (const auto& point : points_) {
    if (point.measured_depth_m < 0.0) {
      throw ValidationError("Trajectory measured depth must be non-negative.");
    }
    if (point.tvd_m < 0.0) {
      throw ValidationError("Trajectory TVD must be non-negative.");
    }
    if (point.measured_depth_m <= previous_md) {
      throw ValidationError("Trajectory measured depth must be strictly increasing.");
    }
    previous_md = point.measured_depth_m;
  }
}

}  // namespace centraltd
