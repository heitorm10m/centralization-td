#pragma once

#include "centraltd/well.hpp"

#include <vector>

namespace centraltd {

struct TrajectoryFrameNode {
  Scalar measured_depth_m{0.0};
  Vector3 tangent_north_east_tvd{0.0, 0.0, 1.0};
  Vector3 normal_north_east_tvd{1.0, 0.0, 0.0};
  Vector3 binormal_north_east_tvd{0.0, 1.0, 0.0};
  Scalar discrete_curvature_rad_per_m{0.0};
  Scalar frame_rotation_change_rad{0.0};
};

std::vector<TrajectoryFrameNode> build_trajectory_frame_nodes(const WellTrajectory& well);

TrajectoryFrameNode interpolate_trajectory_frame(
    const std::vector<TrajectoryFrameNode>& frame_nodes,
    Scalar measured_depth_m);

}  // namespace centraltd
