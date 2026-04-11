from __future__ import annotations

from dataclasses import dataclass
import math

from .models import LoadedCase

E_X = (1.0, 0.0, 0.0)
E_Y = (0.0, 1.0, 0.0)
E_Z = (0.0, 0.0, 1.0)
VERTICAL_TOLERANCE = 1.0e-12


def _dot(lhs: tuple[float, float, float], rhs: tuple[float, float, float]) -> float:
    return (lhs[0] * rhs[0]) + (lhs[1] * rhs[1]) + (lhs[2] * rhs[2])


def _cross(
    lhs: tuple[float, float, float],
    rhs: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        (lhs[1] * rhs[2]) - (lhs[2] * rhs[1]),
        (lhs[2] * rhs[0]) - (lhs[0] * rhs[2]),
        (lhs[0] * rhs[1]) - (lhs[1] * rhs[0]),
    )


def _norm(vector: tuple[float, float, float]) -> float:
    return math.sqrt(_dot(vector, vector))


def _normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    norm = _norm(vector)
    if norm <= VERTICAL_TOLERANCE:
        return (0.0, 0.0, 0.0)
    return (vector[0] / norm, vector[1] / norm, vector[2] / norm)


def _project_to_normal_plane(
    vector: tuple[float, float, float],
    tangent: tuple[float, float, float],
) -> tuple[float, float, float]:
    tangent_component = _dot(vector, tangent)
    return (
        vector[0] - (tangent_component * tangent[0]),
        vector[1] - (tangent_component * tangent[1]),
        vector[2] - (tangent_component * tangent[2]),
    )


def _angle_between(lhs: tuple[float, float, float], rhs: tuple[float, float, float]) -> float:
    lhs_n = _normalize(lhs)
    rhs_n = _normalize(rhs)
    dot = max(-1.0, min(1.0, _dot(lhs_n, rhs_n)))
    return math.acos(dot)


def _eq11_high_side_direction(
    tangent: tuple[float, float, float],
) -> tuple[float, float, float] | None:
    tangent_z = _dot(tangent, E_Z)
    denominator_squared = 1.0 - (tangent_z * tangent_z)
    if denominator_squared <= VERTICAL_TOLERANCE:
        return None

    denominator = math.sqrt(denominator_squared)
    return (
        ((tangent_z * tangent[0]) - E_Z[0]) / denominator,
        ((tangent_z * tangent[1]) - E_Z[1]) / denominator,
        ((tangent_z * tangent[2]) - E_Z[2]) / denominator,
    )


def _vertical_reference_direction(
    tangent: tuple[float, float, float],
    previous_normal: tuple[float, float, float] | None,
    previous_binormal: tuple[float, float, float] | None,
) -> tuple[float, float, float]:
    projected_ex = _normalize(_project_to_normal_plane(E_X, tangent))
    projected_ey = _normalize(_project_to_normal_plane(E_Y, tangent))
    projected_candidates = [candidate for candidate in (projected_ex, projected_ey) if _norm(candidate) > VERTICAL_TOLERANCE]

    if not projected_candidates:
        raise ValueError("Vertical-frame fallback could not construct a valid eX/eY-based reference direction.")

    if previous_normal is None and previous_binormal is None:
        return projected_candidates[0]

    best_direction = projected_candidates[0]
    best_score = float("-inf")
    for candidate in projected_candidates:
        for sign in (1.0, -1.0):
            signed_candidate = (
                sign * candidate[0],
                sign * candidate[1],
                sign * candidate[2],
            )
            signed_binormal = _normalize(_cross(tangent, signed_candidate))
            score = 0.0
            if previous_normal is not None:
                score += _dot(signed_candidate, previous_normal)
            if previous_binormal is not None:
                score += _dot(signed_binormal, previous_binormal)
            if score > best_score:
                best_direction = signed_candidate
                best_score = score

    return best_direction


def build_local_frame_from_tangent(
    tangent_north_east_tvd: tuple[float, float, float],
    previous_normal_north_east_tvd: tuple[float, float, float] | None = None,
    previous_binormal_north_east_tvd: tuple[float, float, float] | None = None,
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    """
    Build the local frame from Dao et al. 2023, Eq. 11.

    ew = t
    eu = (-eZ + tZ * t) / sqrt(1 - tZ^2)
    ev = t x eu

    The vertical-section fallback uses a continuity-preserving direction
    constructed from eX/eY candidates to avoid the Eq. 11 singularity
    when tZ^2 ~= 1.
    """
    tangent = _normalize(tangent_north_east_tvd)
    if _norm(tangent) <= VERTICAL_TOLERANCE:
        raise ValueError("Local frame construction requires a non-zero tangent vector.")

    high_side = _eq11_high_side_direction(tangent)
    if high_side is None:
        high_side = _vertical_reference_direction(
            tangent,
            previous_normal_north_east_tvd,
            previous_binormal_north_east_tvd,
        )

    eu = _normalize(high_side)
    ev = _normalize(_cross(tangent, eu))
    eu = _normalize(_cross(ev, tangent))

    return eu, ev, tangent


@dataclass(slots=True)
class FrameNodeModel:
    measured_depth_m: float
    tangent_north_east_tvd: tuple[float, float, float]
    normal_north_east_tvd: tuple[float, float, float]
    binormal_north_east_tvd: tuple[float, float, float]
    discrete_curvature_rad_per_m: float
    frame_rotation_change_rad: float


def build_frame_nodes(loaded_case: LoadedCase) -> list[FrameNodeModel]:
    geometry_nodes = loaded_case.trajectory_nodes()
    frame_nodes: list[FrameNodeModel] = []
    previous_normal: tuple[float, float, float] | None = None
    previous_binormal: tuple[float, float, float] | None = None

    for index, geometry_node in enumerate(geometry_nodes):
        normal, binormal, tangent = build_local_frame_from_tangent(
            geometry_node.tangent_north_east_tvd,
            previous_normal,
            previous_binormal,
        )
        frame_nodes.append(
            FrameNodeModel(
                measured_depth_m=geometry_node.measured_depth_m,
                tangent_north_east_tvd=tangent,
                normal_north_east_tvd=normal,
                binormal_north_east_tvd=binormal,
                discrete_curvature_rad_per_m=geometry_node.discrete_curvature_rad_per_m,
                frame_rotation_change_rad=(
                    0.0 if index == 0 or previous_normal is None else _angle_between(previous_normal, normal)
                ),
            )
        )
        previous_normal = normal
        previous_binormal = binormal

    return frame_nodes


def interpolate_frame(frame_nodes: list[FrameNodeModel], measured_depth_m: float) -> FrameNodeModel:
    if not frame_nodes:
        raise ValueError("Trajectory frame interpolation requires at least one frame node.")
    if measured_depth_m < frame_nodes[0].measured_depth_m or measured_depth_m > frame_nodes[-1].measured_depth_m:
        raise ValueError("Trajectory frame interpolation MD is outside the available range.")
    if measured_depth_m == frame_nodes[0].measured_depth_m:
        return frame_nodes[0]
    if measured_depth_m == frame_nodes[-1].measured_depth_m:
        return frame_nodes[-1]

    upper_index = 0
    for upper_index, frame_node in enumerate(frame_nodes):
        if frame_node.measured_depth_m >= measured_depth_m:
            break
    lower = frame_nodes[upper_index - 1]
    upper = frame_nodes[upper_index]
    interval_m = upper.measured_depth_m - lower.measured_depth_m
    ratio = 0.0 if interval_m <= 0.0 else (measured_depth_m - lower.measured_depth_m) / interval_m

    tangent = _normalize(
        (
            lower.tangent_north_east_tvd[0]
            + (ratio * (upper.tangent_north_east_tvd[0] - lower.tangent_north_east_tvd[0])),
            lower.tangent_north_east_tvd[1]
            + (ratio * (upper.tangent_north_east_tvd[1] - lower.tangent_north_east_tvd[1])),
            lower.tangent_north_east_tvd[2]
            + (ratio * (upper.tangent_north_east_tvd[2] - lower.tangent_north_east_tvd[2])),
        )
    )
    normal, binormal, tangent = build_local_frame_from_tangent(
        tangent,
        lower.normal_north_east_tvd,
        lower.binormal_north_east_tvd,
    )

    return FrameNodeModel(
        measured_depth_m=measured_depth_m,
        tangent_north_east_tvd=tangent,
        normal_north_east_tvd=normal,
        binormal_north_east_tvd=binormal,
        discrete_curvature_rad_per_m=lower.discrete_curvature_rad_per_m
        + (ratio * (upper.discrete_curvature_rad_per_m - lower.discrete_curvature_rad_per_m)),
        frame_rotation_change_rad=lower.frame_rotation_change_rad
        + (ratio * (upper.frame_rotation_change_rad - lower.frame_rotation_change_rad)),
    )
