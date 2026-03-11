from __future__ import annotations

from dataclasses import dataclass
import math

from .models import LoadedCase


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
    if norm <= 1.0e-12:
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


def _fallback_reference_axis(tangent: tuple[float, float, float]) -> tuple[float, float, float]:
    vertical_axis = (0.0, 0.0, 1.0)
    if abs(_dot(tangent, vertical_axis)) < 0.9:
        return vertical_axis
    return (1.0, 0.0, 0.0)


def _stable_normal(
    tangent: tuple[float, float, float],
    reference_axis: tuple[float, float, float],
) -> tuple[float, float, float]:
    normal = _normalize(_project_to_normal_plane(reference_axis, tangent))
    if _norm(normal) <= 1.0e-12:
        normal = _normalize(_project_to_normal_plane(_fallback_reference_axis(tangent), tangent))
    return normal


def _stable_binormal(
    tangent: tuple[float, float, float],
    normal: tuple[float, float, float],
) -> tuple[float, float, float]:
    binormal = _normalize(_cross(tangent, normal))
    if _norm(binormal) <= 1.0e-12:
        binormal = _normalize(_cross(tangent, _fallback_reference_axis(tangent)))
    return binormal


def _angle_between(lhs: tuple[float, float, float], rhs: tuple[float, float, float]) -> float:
    lhs_n = _normalize(lhs)
    rhs_n = _normalize(rhs)
    dot = max(-1.0, min(1.0, _dot(lhs_n, rhs_n)))
    return math.acos(dot)


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
    previous_normal = (1.0, 0.0, 0.0)
    previous_binormal = (0.0, 1.0, 0.0)

    for index, geometry_node in enumerate(geometry_nodes):
        tangent = _normalize(geometry_node.tangent_north_east_tvd)
        if index == 0:
            normal = _stable_normal(tangent, _fallback_reference_axis(tangent))
        else:
            normal = _normalize(_project_to_normal_plane(previous_normal, tangent))
            if _norm(normal) <= 1.0e-12:
                normal = _stable_normal(tangent, previous_binormal)
            if _dot(normal, previous_normal) < 0.0:
                normal = (-normal[0], -normal[1], -normal[2])

        binormal = _stable_binormal(tangent, normal)
        if index > 0 and _dot(binormal, previous_binormal) < 0.0:
            normal = (-normal[0], -normal[1], -normal[2])
            binormal = (-binormal[0], -binormal[1], -binormal[2])

        frame_nodes.append(
            FrameNodeModel(
                measured_depth_m=geometry_node.measured_depth_m,
                tangent_north_east_tvd=tangent,
                normal_north_east_tvd=normal,
                binormal_north_east_tvd=binormal,
                discrete_curvature_rad_per_m=geometry_node.discrete_curvature_rad_per_m,
                frame_rotation_change_rad=(
                    0.0 if index == 0 else _angle_between(previous_normal, normal)
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
    normal = _normalize(
        _project_to_normal_plane(
            (
                lower.normal_north_east_tvd[0]
                + (ratio * (upper.normal_north_east_tvd[0] - lower.normal_north_east_tvd[0])),
                lower.normal_north_east_tvd[1]
                + (ratio * (upper.normal_north_east_tvd[1] - lower.normal_north_east_tvd[1])),
                lower.normal_north_east_tvd[2]
                + (ratio * (upper.normal_north_east_tvd[2] - lower.normal_north_east_tvd[2])),
            ),
            tangent,
        )
    )
    if _norm(normal) <= 1.0e-12:
        normal = _stable_normal(tangent, lower.normal_north_east_tvd)
    binormal = _stable_binormal(tangent, normal)

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
