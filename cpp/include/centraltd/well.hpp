#pragma once

#include "centraltd/types.hpp"

#include <array>
#include <vector>

namespace centraltd {

struct WellTrajectoryPoint {
  Scalar measured_depth_m{0.0};
  Scalar inclination_rad{0.0};
  Scalar azimuth_rad{0.0};
  Scalar tvd_m{0.0};
  Scalar northing_m{0.0};
  Scalar easting_m{0.0};
};

struct TrajectoryGeometryNode {
  Scalar measured_depth_m{0.0};
  Scalar inclination_rad{0.0};
  Scalar azimuth_rad{0.0};
  Scalar tvd_m{0.0};
  Scalar northing_m{0.0};
  Scalar easting_m{0.0};
  std::array<Scalar, 3> tangent_north_east_tvd{0.0, 0.0, 1.0};
  Scalar discrete_curvature_rad_per_m{0.0};
};

struct TrajectorySummary {
  std::size_t point_count{0};
  Scalar start_measured_depth_m{0.0};
  Scalar final_measured_depth_m{0.0};
  Scalar total_course_length_m{0.0};
  Scalar vertical_depth_m{0.0};
  Scalar lateral_displacement_m{0.0};
  Scalar max_inclination_rad{0.0};
  Scalar max_curvature_rad_per_m{0.0};
  bool coordinates_are_approximate{true};
};

class WellTrajectory {
 public:
  WellTrajectory() = default;
  explicit WellTrajectory(std::vector<WellTrajectoryPoint> points);

  const std::vector<WellTrajectoryPoint>& points() const noexcept;
  std::size_t size() const noexcept;
  bool empty() const noexcept;
  Scalar final_measured_depth_m() const noexcept;
  std::array<Scalar, 3> local_tangent(Index point_index) const;
  Scalar discrete_curvature_rad_per_m(Index point_index) const;
  std::vector<TrajectoryGeometryNode> derived_geometry() const;
  WellTrajectoryPoint interpolate(Scalar measured_depth_m) const;
  TrajectorySummary summary() const;
  void validate() const;

 private:
  std::vector<WellTrajectoryPoint> points_;
};

}  // namespace centraltd
