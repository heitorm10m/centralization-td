#pragma once

#include "centraltd/types.hpp"

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

class WellTrajectory {
 public:
  WellTrajectory() = default;
  explicit WellTrajectory(std::vector<WellTrajectoryPoint> points);

  const std::vector<WellTrajectoryPoint>& points() const noexcept;
  std::size_t size() const noexcept;
  bool empty() const noexcept;
  Scalar final_measured_depth_m() const noexcept;
  void validate() const;

 private:
  std::vector<WellTrajectoryPoint> points_;
};

}  // namespace centraltd

