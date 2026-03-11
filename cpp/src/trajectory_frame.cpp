#include "centraltd/trajectory_frame.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace centraltd {
namespace {

constexpr Scalar kMinimumNorm = 1.0e-12;
constexpr Scalar kReferenceAlignmentLimit = 0.9;

Scalar clamp_scalar(Scalar value, Scalar lower, Scalar upper) {
  return std::max(lower, std::min(value, upper));
}

Scalar dot_product(const Vector3& lhs, const Vector3& rhs) {
  return (lhs[0] * rhs[0]) + (lhs[1] * rhs[1]) + (lhs[2] * rhs[2]);
}

Vector3 cross_product(const Vector3& lhs, const Vector3& rhs) {
  return {
      (lhs[1] * rhs[2]) - (lhs[2] * rhs[1]),
      (lhs[2] * rhs[0]) - (lhs[0] * rhs[2]),
      (lhs[0] * rhs[1]) - (lhs[1] * rhs[0]),
  };
}

Scalar vector_norm(const Vector3& vector) {
  return std::sqrt(dot_product(vector, vector));
}

Vector3 normalize(const Vector3& vector) {
  const Scalar norm = vector_norm(vector);
  if (norm <= kMinimumNorm) {
    return {0.0, 0.0, 0.0};
  }

  return {vector[0] / norm, vector[1] / norm, vector[2] / norm};
}

Vector3 project_onto_normal_plane(const Vector3& vector, const Vector3& tangent) {
  const Scalar tangent_component = dot_product(vector, tangent);
  return {
      vector[0] - (tangent_component * tangent[0]),
      vector[1] - (tangent_component * tangent[1]),
      vector[2] - (tangent_component * tangent[2]),
  };
}

Vector3 fallback_reference_axis(const Vector3& tangent) {
  const Vector3 vertical_axis{0.0, 0.0, 1.0};
  if (std::abs(dot_product(tangent, vertical_axis)) < kReferenceAlignmentLimit) {
    return vertical_axis;
  }
  return {1.0, 0.0, 0.0};
}

Vector3 stable_normal_from_reference(const Vector3& tangent, const Vector3& reference_axis) {
  auto normal = normalize(project_onto_normal_plane(reference_axis, tangent));
  if (vector_norm(normal) <= kMinimumNorm) {
    normal = normalize(project_onto_normal_plane(fallback_reference_axis(tangent), tangent));
  }
  return normal;
}

Vector3 stable_binormal_from_tangent_normal(const Vector3& tangent, const Vector3& normal) {
  auto binormal = normalize(cross_product(tangent, normal));
  if (vector_norm(binormal) <= kMinimumNorm) {
    binormal = normalize(cross_product(tangent, fallback_reference_axis(tangent)));
  }
  return binormal;
}

Scalar angle_between(const Vector3& lhs, const Vector3& rhs) {
  const auto lhs_normalized = normalize(lhs);
  const auto rhs_normalized = normalize(rhs);
  const Scalar dot = dot_product(lhs_normalized, rhs_normalized);
  return std::acos(clamp_scalar(dot, -1.0, 1.0));
}

}  // namespace

std::vector<TrajectoryFrameNode> build_trajectory_frame_nodes(const WellTrajectory& well) {
  well.validate();
  const auto geometry_nodes = well.derived_geometry();
  std::vector<TrajectoryFrameNode> frame_nodes;
  frame_nodes.reserve(geometry_nodes.size());

  Vector3 previous_normal{1.0, 0.0, 0.0};
  Vector3 previous_binormal{0.0, 1.0, 0.0};
  bool has_previous_frame = false;

  for (const auto& geometry_node : geometry_nodes) {
    const auto tangent = normalize(geometry_node.tangent_north_east_tvd);
    Vector3 normal = {0.0, 0.0, 0.0};

    if (!has_previous_frame) {
      normal = stable_normal_from_reference(tangent, fallback_reference_axis(tangent));
    } else {
      normal = normalize(project_onto_normal_plane(previous_normal, tangent));
      if (vector_norm(normal) <= kMinimumNorm) {
        normal = stable_normal_from_reference(tangent, previous_binormal);
      }
      if (dot_product(normal, previous_normal) < 0.0) {
        normal = {-normal[0], -normal[1], -normal[2]};
      }
    }

    auto binormal = stable_binormal_from_tangent_normal(tangent, normal);
    if (has_previous_frame && dot_product(binormal, previous_binormal) < 0.0) {
      normal = {-normal[0], -normal[1], -normal[2]};
      binormal = {-binormal[0], -binormal[1], -binormal[2]};
    }

    TrajectoryFrameNode frame_node;
    frame_node.measured_depth_m = geometry_node.measured_depth_m;
    frame_node.tangent_north_east_tvd = tangent;
    frame_node.normal_north_east_tvd = normal;
    frame_node.binormal_north_east_tvd = binormal;
    frame_node.discrete_curvature_rad_per_m = geometry_node.discrete_curvature_rad_per_m;
    frame_node.frame_rotation_change_rad =
        has_previous_frame ? angle_between(previous_normal, normal) : 0.0;
    frame_nodes.push_back(frame_node);

    previous_normal = normal;
    previous_binormal = binormal;
    has_previous_frame = true;
  }

  return frame_nodes;
}

TrajectoryFrameNode interpolate_trajectory_frame(
    const std::vector<TrajectoryFrameNode>& frame_nodes,
    Scalar measured_depth_m) {
  if (frame_nodes.empty()) {
    throw ValidationError("Trajectory frame interpolation requires at least one frame node.");
  }
  if (measured_depth_m < frame_nodes.front().measured_depth_m ||
      measured_depth_m > frame_nodes.back().measured_depth_m) {
    throw ValidationError("Trajectory frame interpolation MD is outside the available range.");
  }
  if (measured_depth_m == frame_nodes.front().measured_depth_m) {
    return frame_nodes.front();
  }
  if (measured_depth_m == frame_nodes.back().measured_depth_m) {
    return frame_nodes.back();
  }

  const auto upper = std::lower_bound(
      frame_nodes.begin(),
      frame_nodes.end(),
      measured_depth_m,
      [](const TrajectoryFrameNode& frame_node, Scalar target_md_m) {
        return frame_node.measured_depth_m < target_md_m;
      });
  const auto lower = std::prev(upper);
  const Scalar interval_m = upper->measured_depth_m - lower->measured_depth_m;
  const Scalar ratio = interval_m > 0.0 ? (measured_depth_m - lower->measured_depth_m) / interval_m : 0.0;

  const auto tangent = normalize({
      lower->tangent_north_east_tvd[0] + (ratio * (upper->tangent_north_east_tvd[0] - lower->tangent_north_east_tvd[0])),
      lower->tangent_north_east_tvd[1] + (ratio * (upper->tangent_north_east_tvd[1] - lower->tangent_north_east_tvd[1])),
      lower->tangent_north_east_tvd[2] + (ratio * (upper->tangent_north_east_tvd[2] - lower->tangent_north_east_tvd[2])),
  });
  auto normal = normalize(project_onto_normal_plane({
      lower->normal_north_east_tvd[0] + (ratio * (upper->normal_north_east_tvd[0] - lower->normal_north_east_tvd[0])),
      lower->normal_north_east_tvd[1] + (ratio * (upper->normal_north_east_tvd[1] - lower->normal_north_east_tvd[1])),
      lower->normal_north_east_tvd[2] + (ratio * (upper->normal_north_east_tvd[2] - lower->normal_north_east_tvd[2])),
  }, tangent));
  if (vector_norm(normal) <= kMinimumNorm) {
    normal = stable_normal_from_reference(tangent, lower->normal_north_east_tvd);
  }

  TrajectoryFrameNode frame_node;
  frame_node.measured_depth_m = measured_depth_m;
  frame_node.tangent_north_east_tvd = tangent;
  frame_node.normal_north_east_tvd = normal;
  frame_node.binormal_north_east_tvd = stable_binormal_from_tangent_normal(tangent, normal);
  frame_node.discrete_curvature_rad_per_m =
      lower->discrete_curvature_rad_per_m +
      (ratio * (upper->discrete_curvature_rad_per_m - lower->discrete_curvature_rad_per_m));
  frame_node.frame_rotation_change_rad =
      lower->frame_rotation_change_rad +
      (ratio * (upper->frame_rotation_change_rad - lower->frame_rotation_change_rad));
  return frame_node;
}

}  // namespace centraltd
