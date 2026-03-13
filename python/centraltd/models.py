from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any


class ConfigError(ValueError):
    """Raised when a YAML file is missing required fields or contains invalid values."""


PI = math.pi
TAU = 2.0 * math.pi
STANDARD_GRAVITY_M_PER_S2 = 9.80665
SUPPORTED_OPERATION_MODES = {"run_in", "pull_out", "rotate_in_place"}


def _expect_mapping(raw: Any, context: str) -> Mapping[str, Any]:
    if not isinstance(raw, Mapping):
        raise ConfigError(f"{context} must be a mapping.")
    return raw


def _expect_sequence(raw: Any, context: str) -> Sequence[Any]:
    if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes, bytearray)):
        raise ConfigError(f"{context} must be a sequence.")
    return raw


def _require_text(raw: Mapping[str, Any], key: str, context: str) -> str:
    value = raw.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ConfigError(f"{context}.{key} must be a non-empty string.")
    return value.strip()


def _optional_text(raw: Mapping[str, Any], key: str, default: str | None = None) -> str | None:
    value = raw.get(key, default)
    if value is None:
        return None
    if not isinstance(value, str):
        raise ConfigError(f"{key} must be a string when provided.")
    return value


def _require_float(raw: Mapping[str, Any], key: str, context: str) -> float:
    value = raw.get(key)
    if not isinstance(value, (int, float)):
        raise ConfigError(f"{context}.{key} must be numeric.")
    return float(value)


def _optional_float(raw: Mapping[str, Any], key: str, context: str) -> float | None:
    value = raw.get(key)
    if value is None:
        return None
    if not isinstance(value, (int, float)):
        raise ConfigError(f"{context}.{key} must be numeric when provided.")
    return float(value)


def _require_int(raw: Mapping[str, Any], key: str, context: str) -> int:
    value = raw.get(key)
    if not isinstance(value, int) or isinstance(value, bool):
        raise ConfigError(f"{context}.{key} must be an integer.")
    return value


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def _normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    norm = math.sqrt((vector[0] ** 2) + (vector[1] ** 2) + (vector[2] ** 2))
    if norm <= 0.0:
        return (0.0, 0.0, 1.0)
    return (vector[0] / norm, vector[1] / norm, vector[2] / norm)


def _tangent_from_angles(inclination_rad: float, azimuth_rad: float) -> tuple[float, float, float]:
    return (
        math.sin(inclination_rad) * math.cos(azimuth_rad),
        math.sin(inclination_rad) * math.sin(azimuth_rad),
        math.cos(inclination_rad),
    )


def _angles_from_tangent(tangent: tuple[float, float, float]) -> tuple[float, float]:
    normalized = _normalize(tangent)
    inclination_rad = math.acos(_clamp(normalized[2], -1.0, 1.0))
    azimuth_rad = math.atan2(normalized[1], normalized[0])
    if azimuth_rad < 0.0:
        azimuth_rad += TAU
    return inclination_rad, azimuth_rad


def _interpolate_tangent(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
    ratio: float,
) -> tuple[float, float, float]:
    return _normalize(
        (
            left[0] + (ratio * (right[0] - left[0])),
            left[1] + (ratio * (right[1] - left[1])),
            left[2] + (ratio * (right[2] - left[2])),
        )
    )


def _angle_between(left: tuple[float, float, float], right: tuple[float, float, float]) -> float:
    lhs = _normalize(left)
    rhs = _normalize(right)
    dot_product = (lhs[0] * rhs[0]) + (lhs[1] * rhs[1]) + (lhs[2] * rhs[2])
    return math.acos(_clamp(dot_product, -1.0, 1.0))


@dataclass(slots=True)
class WellTrajectoryPointModel:
    measured_depth_m: float
    inclination_rad: float
    azimuth_rad: float

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any], index: int) -> "WellTrajectoryPointModel":
        context = f"trajectory[{index}]"
        point = cls(
            measured_depth_m=_require_float(raw, "measured_depth_m", context),
            inclination_rad=_require_float(raw, "inclination_rad", context),
            azimuth_rad=_require_float(raw, "azimuth_rad", context),
        )
        point.validate(context)
        return point

    def validate(self, context: str) -> None:
        if self.measured_depth_m < 0.0:
            raise ConfigError(f"{context}.measured_depth_m must be non-negative.")
        if self.inclination_rad < 0.0 or self.inclination_rad > PI:
            raise ConfigError(f"{context}.inclination_rad must stay within [0, pi].")
        if self.azimuth_rad < 0.0 or self.azimuth_rad > TAU:
            raise ConfigError(f"{context}.azimuth_rad must stay within [0, 2*pi].")


@dataclass(slots=True)
class TrajectoryNodeModel:
    measured_depth_m: float
    inclination_rad: float
    azimuth_rad: float
    tvd_m: float
    northing_m: float
    easting_m: float
    tangent_north_east_tvd: tuple[float, float, float]
    discrete_curvature_rad_per_m: float


@dataclass(slots=True)
class TrajectorySummaryModel:
    point_count: int
    start_measured_depth_m: float
    final_measured_depth_m: float
    total_course_length_m: float
    vertical_depth_m: float
    lateral_displacement_m: float
    max_inclination_rad: float
    max_curvature_rad_per_m: float
    coordinates_are_approximate: bool = True


@dataclass(slots=True)
class WellModel:
    name: str
    trajectory: list[WellTrajectoryPointModel]
    hole_diameter_m: float | None = None
    fluid_density_kg_per_m3: float = 1000.0

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "WellModel":
        context = "well"
        trajectory_raw = _expect_sequence(raw.get("trajectory"), f"{context}.trajectory")
        trajectory = [
            WellTrajectoryPointModel.from_dict(
                _expect_mapping(item, f"{context}.trajectory[{index}]"),
                index,
            )
            for index, item in enumerate(trajectory_raw)
        ]
        hole_diameter_m = _optional_float(raw, "hole_diameter_m", context)
        fluid_density_kg_per_m3 = _optional_float(raw, "fluid_density_kg_per_m3", context)
        model = cls(
            name=_require_text(raw, "name", context),
            trajectory=trajectory,
            hole_diameter_m=hole_diameter_m,
            fluid_density_kg_per_m3=1000.0
            if fluid_density_kg_per_m3 is None
            else fluid_density_kg_per_m3,
        )
        model.validate()
        return model

    def validate(self) -> None:
        if not self.trajectory:
            raise ConfigError("well.trajectory must contain at least one point.")
        if self.hole_diameter_m is not None and self.hole_diameter_m <= 0.0:
            raise ConfigError("well.hole_diameter_m must be positive when provided.")
        if self.fluid_density_kg_per_m3 < 0.0:
            raise ConfigError("well.fluid_density_kg_per_m3 must be non-negative.")
        previous_md = -1.0
        for index, point in enumerate(self.trajectory):
            point.validate(f"well.trajectory[{index}]")
            if point.measured_depth_m <= previous_md:
                raise ConfigError(
                    f"well.trajectory[{index}].measured_depth_m must be strictly increasing."
                )
            previous_md = point.measured_depth_m

    @property
    def final_measured_depth_m(self) -> float:
        return self.trajectory[-1].measured_depth_m

    def derived_nodes(self) -> list[TrajectoryNodeModel]:
        self.validate()
        nodes: list[TrajectoryNodeModel] = []
        for index, point in enumerate(self.trajectory):
            tangent = _tangent_from_angles(point.inclination_rad, point.azimuth_rad)
            discrete_curvature = self.discrete_curvature_rad_per_m(index)
            tvd_m = 0.0
            northing_m = 0.0
            easting_m = 0.0
            if nodes:
                previous = nodes[-1]
                segment_length_m = point.measured_depth_m - previous.measured_depth_m
                northing_m = previous.northing_m + (
                    0.5
                    * segment_length_m
                    * (previous.tangent_north_east_tvd[0] + tangent[0])
                )
                easting_m = previous.easting_m + (
                    0.5
                    * segment_length_m
                    * (previous.tangent_north_east_tvd[1] + tangent[1])
                )
                tvd_m = previous.tvd_m + (
                    0.5
                    * segment_length_m
                    * (previous.tangent_north_east_tvd[2] + tangent[2])
                )

            nodes.append(
                TrajectoryNodeModel(
                    measured_depth_m=point.measured_depth_m,
                    inclination_rad=point.inclination_rad,
                    azimuth_rad=point.azimuth_rad,
                    tvd_m=tvd_m,
                    northing_m=northing_m,
                    easting_m=easting_m,
                    tangent_north_east_tvd=tangent,
                    discrete_curvature_rad_per_m=discrete_curvature,
                )
            )
        return nodes

    def interpolate(self, measured_depth_m: float) -> TrajectoryNodeModel:
        self.validate()
        if measured_depth_m < self.trajectory[0].measured_depth_m or measured_depth_m > self.final_measured_depth_m:
            raise ConfigError("Interpolation measured depth is outside the trajectory range.")

        nodes = self.derived_nodes()
        if measured_depth_m == nodes[0].measured_depth_m:
            return nodes[0]
        if measured_depth_m == nodes[-1].measured_depth_m:
            return nodes[-1]

        upper_index = 0
        for upper_index, node in enumerate(nodes):
            if node.measured_depth_m >= measured_depth_m:
                break

        lower = nodes[upper_index - 1]
        upper = nodes[upper_index]
        segment_length_m = upper.measured_depth_m - lower.measured_depth_m
        ratio = 0.0 if segment_length_m <= 0.0 else (measured_depth_m - lower.measured_depth_m) / segment_length_m
        tangent = _interpolate_tangent(lower.tangent_north_east_tvd, upper.tangent_north_east_tvd, ratio)
        inclination_rad, azimuth_rad = _angles_from_tangent(tangent)

        return TrajectoryNodeModel(
            measured_depth_m=measured_depth_m,
            inclination_rad=inclination_rad,
            azimuth_rad=azimuth_rad,
            tvd_m=lower.tvd_m + (ratio * (upper.tvd_m - lower.tvd_m)),
            northing_m=lower.northing_m + (ratio * (upper.northing_m - lower.northing_m)),
            easting_m=lower.easting_m + (ratio * (upper.easting_m - lower.easting_m)),
            tangent_north_east_tvd=tangent,
            discrete_curvature_rad_per_m=0.0 if segment_length_m <= 0.0 else _angle_between(lower.tangent_north_east_tvd, upper.tangent_north_east_tvd) / segment_length_m,
        )

    def discrete_curvature_rad_per_m(self, point_index: int) -> float:
        self.validate()
        if point_index < 0 or point_index >= len(self.trajectory):
            raise ConfigError("Trajectory curvature index is out of range.")
        if point_index == 0:
            return 0.0
        current = self.trajectory[point_index]
        previous = self.trajectory[point_index - 1]
        segment_length_m = current.measured_depth_m - previous.measured_depth_m
        if segment_length_m <= 0.0:
            return 0.0
        return _angle_between(
            _tangent_from_angles(previous.inclination_rad, previous.azimuth_rad),
            _tangent_from_angles(current.inclination_rad, current.azimuth_rad),
        ) / segment_length_m

    def summary(self) -> TrajectorySummaryModel:
        nodes = self.derived_nodes()
        last = nodes[-1]
        return TrajectorySummaryModel(
            point_count=len(nodes),
            start_measured_depth_m=nodes[0].measured_depth_m,
            final_measured_depth_m=last.measured_depth_m,
            total_course_length_m=last.measured_depth_m - nodes[0].measured_depth_m,
            vertical_depth_m=last.tvd_m,
            lateral_displacement_m=math.sqrt((last.northing_m ** 2) + (last.easting_m ** 2)),
            max_inclination_rad=max(node.inclination_rad for node in nodes),
            max_curvature_rad_per_m=max(node.discrete_curvature_rad_per_m for node in nodes),
            coordinates_are_approximate=True,
        )


@dataclass(slots=True)
class StringSectionModel:
    name: str
    md_start_m: float
    md_end_m: float
    outer_diameter_m: float
    inner_diameter_m: float
    linear_weight_n_per_m: float
    young_modulus_pa: float
    shear_modulus_pa: float
    density_kg_per_m3: float
    friction_coefficient: float

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any], index: int) -> "StringSectionModel":
        context = f"sections[{index}]"
        section = cls(
            name=_require_text(raw, "name", context),
            md_start_m=_require_float(raw, "md_start_m", context),
            md_end_m=_require_float(raw, "md_end_m", context),
            outer_diameter_m=_require_float(raw, "outer_diameter_m", context),
            inner_diameter_m=_require_float(raw, "inner_diameter_m", context),
            linear_weight_n_per_m=_require_float(raw, "linear_weight_n_per_m", context),
            young_modulus_pa=_require_float(raw, "young_modulus_pa", context),
            shear_modulus_pa=_require_float(raw, "shear_modulus_pa", context),
            density_kg_per_m3=_require_float(raw, "density_kg_per_m3", context),
            friction_coefficient=_require_float(raw, "friction_coefficient", context),
        )
        section.validate(context)
        return section

    @property
    def length_m(self) -> float:
        return self.md_end_m - self.md_start_m

    @property
    def outer_radius_m(self) -> float:
        return 0.5 * self.outer_diameter_m

    @property
    def cross_sectional_area_m2(self) -> float:
        return 0.25 * PI * ((self.outer_diameter_m**2) - (self.inner_diameter_m**2))

    @property
    def second_moment_of_area_m4(self) -> float:
        return (PI / 64.0) * ((self.outer_diameter_m**4) - (self.inner_diameter_m**4))

    @property
    def polar_moment_of_area_m4(self) -> float:
        return (PI / 32.0) * ((self.outer_diameter_m**4) - (self.inner_diameter_m**4))

    @property
    def bending_stiffness_n_m2(self) -> float:
        return self.young_modulus_pa * self.second_moment_of_area_m4

    @property
    def torsional_stiffness_n_m2(self) -> float:
        return self.shear_modulus_pa * self.polar_moment_of_area_m4

    @property
    def displaced_area_m2(self) -> float:
        return 0.25 * PI * (self.outer_diameter_m**2)

    def buoyancy_force_n_per_m(self, fluid_density_kg_per_m3: float) -> float:
        return fluid_density_kg_per_m3 * STANDARD_GRAVITY_M_PER_S2 * self.displaced_area_m2

    def effective_line_weight_n_per_m(self, fluid_density_kg_per_m3: float) -> float:
        return self.linear_weight_n_per_m - self.buoyancy_force_n_per_m(fluid_density_kg_per_m3)

    def contains_md(self, measured_depth_m: float) -> bool:
        return self.md_start_m <= measured_depth_m <= self.md_end_m

    def validate(self, context: str) -> None:
        if self.md_start_m < 0.0:
            raise ConfigError(f"{context}.md_start_m must be non-negative.")
        if self.md_end_m <= self.md_start_m:
            raise ConfigError(f"{context}.md_end_m must be greater than md_start_m.")
        if self.outer_diameter_m <= 0.0 or self.inner_diameter_m <= 0.0:
            raise ConfigError(f"{context}.diameters must be positive.")
        if self.inner_diameter_m >= self.outer_diameter_m:
            raise ConfigError(f"{context}.inner_diameter_m must be smaller than outer_diameter_m.")
        if self.linear_weight_n_per_m <= 0.0:
            raise ConfigError(f"{context}.linear_weight_n_per_m must be positive.")
        if self.young_modulus_pa <= 0.0:
            raise ConfigError(f"{context}.young_modulus_pa must be positive.")
        if self.shear_modulus_pa <= 0.0:
            raise ConfigError(f"{context}.shear_modulus_pa must be positive.")
        if self.density_kg_per_m3 <= 0.0:
            raise ConfigError(f"{context}.density_kg_per_m3 must be positive.")
        if self.friction_coefficient < 0.0 or self.friction_coefficient > 1.5:
            raise ConfigError(f"{context}.friction_coefficient must stay within [0, 1.5].")


@dataclass(slots=True)
class StringSectionSummaryModel:
    name: str
    md_start_m: float
    md_end_m: float
    length_m: float
    outer_diameter_m: float
    inner_diameter_m: float
    linear_weight_n_per_m: float
    friction_coefficient: float
    nominal_radial_clearance_m: float


@dataclass(slots=True)
class StringSummaryModel:
    section_count: int
    total_length_m: float
    total_weight_n: float
    total_effective_weight_n: float
    max_outer_diameter_m: float
    min_inner_diameter_m: float
    average_friction_coefficient: float
    average_density_kg_per_m3: float


@dataclass(slots=True)
class StringConfigModel:
    name: str
    sections: list[StringSectionModel]

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "StringConfigModel":
        context = "string"
        sections_raw = _expect_sequence(raw.get("sections"), f"{context}.sections")
        sections = [
            StringSectionModel.from_dict(
                _expect_mapping(item, f"{context}.sections[{index}]"),
                index,
            )
            for index, item in enumerate(sections_raw)
        ]
        model = cls(name=_require_text(raw, "name", context), sections=sections)
        model.validate()
        return model

    def validate(self) -> None:
        if not self.sections:
            raise ConfigError("string.sections must contain at least one item.")
        previous_end_m = -1.0
        for index, section in enumerate(self.sections):
            section.validate(f"string.sections[{index}]")
            if section.md_start_m < previous_end_m:
                raise ConfigError("string.sections must be ordered and non-overlapping.")
            previous_end_m = section.md_end_m

    def summary(self) -> StringSummaryModel:
        total_length_m = sum(section.length_m for section in self.sections)
        total_weight_n = sum(section.length_m * section.linear_weight_n_per_m for section in self.sections)
        total_effective_weight_n = sum(
            section.length_m * section.effective_line_weight_n_per_m(1000.0)
            for section in self.sections
        )
        max_outer_diameter_m = max(section.outer_diameter_m for section in self.sections)
        min_inner_diameter_m = min(section.inner_diameter_m for section in self.sections)
        average_friction_coefficient = (
            0.0
            if total_length_m <= 0.0
            else sum(section.friction_coefficient * section.length_m for section in self.sections) / total_length_m
        )
        average_density_kg_per_m3 = (
            0.0
            if total_length_m <= 0.0
            else sum(section.density_kg_per_m3 * section.length_m for section in self.sections) / total_length_m
        )
        return StringSummaryModel(
            section_count=len(self.sections),
            total_length_m=total_length_m,
            total_weight_n=total_weight_n,
            total_effective_weight_n=total_effective_weight_n,
            max_outer_diameter_m=max_outer_diameter_m,
            min_inner_diameter_m=min_inner_diameter_m,
            average_friction_coefficient=average_friction_coefficient,
            average_density_kg_per_m3=average_density_kg_per_m3,
        )

    def section_summaries(self, hole_diameter_m: float) -> list[StringSectionSummaryModel]:
        return [
            StringSectionSummaryModel(
                name=section.name,
                md_start_m=section.md_start_m,
                md_end_m=section.md_end_m,
                length_m=section.length_m,
                outer_diameter_m=section.outer_diameter_m,
                inner_diameter_m=section.inner_diameter_m,
                linear_weight_n_per_m=section.linear_weight_n_per_m,
                friction_coefficient=section.friction_coefficient,
                nominal_radial_clearance_m=0.0
                if hole_diameter_m <= 0.0
                else 0.5 * (hole_diameter_m - section.outer_diameter_m),
            )
            for section in self.sections
        ]


@dataclass(slots=True)
class CentralizerSpecModel:
    name: str
    type: str
    outer_diameter_m: float
    support_outer_diameter_m: float
    number_of_bows: int
    angular_orientation_reference_deg: float
    inner_clearance_to_pipe_m: float
    nominal_restoring_force_n: float
    nominal_running_force_n: float
    spacing_hint_m: float
    axial_force_ratio: float | None = None
    tangential_force_ratio: float | None = None
    blade_power_law_k: float | None = None
    blade_power_law_p: float = 1.0
    min_contact_diameter_m: float | None = None
    max_contact_diameter_m: float | None = None
    count_hint: int | None = None
    installation_md_m: list[float] | None = None

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any], index: int) -> "CentralizerSpecModel":
        context = f"centralizers[{index}]"
        installation_md_raw = raw.get("installation_md_m", [])
        if installation_md_raw is None:
            installation_md_m: list[float] = []
        else:
            installation_md_m = [
                float(value)
                for value in _expect_sequence(installation_md_raw, f"{context}.installation_md_m")
            ]
        count_hint = raw.get("count_hint")
        if count_hint is not None and (not isinstance(count_hint, int) or isinstance(count_hint, bool)):
            raise ConfigError(f"{context}.count_hint must be an integer when provided.")
        centralizer_type = (
            _optional_text(raw, "centralizer_type")
            or _optional_text(raw, "type", "bow-spring")
            or "bow-spring"
        )
        support_outer_diameter_m = _optional_float(raw, "support_outer_diameter_m", context)
        outer_diameter_m = _optional_float(raw, "outer_diameter_m", context)
        resolved_support_outer_diameter_m = (
            support_outer_diameter_m
            if support_outer_diameter_m is not None
            else outer_diameter_m
        )
        if resolved_support_outer_diameter_m is None:
            raise ConfigError(
                f"{context}.support_outer_diameter_m or {context}.outer_diameter_m must be provided."
            )
        spec = cls(
            name=_require_text(raw, "name", context),
            type=centralizer_type,
            outer_diameter_m=(
                outer_diameter_m
                if outer_diameter_m is not None
                else resolved_support_outer_diameter_m
            ),
            support_outer_diameter_m=resolved_support_outer_diameter_m,
            number_of_bows=(
                _require_int(raw, "number_of_bows", context)
                if raw.get("number_of_bows") is not None
                else 6
            ),
            angular_orientation_reference_deg=(
                _optional_float(raw, "angular_orientation_reference_deg", context) or 0.0
            ),
            inner_clearance_to_pipe_m=(
                _optional_float(raw, "inner_clearance_to_pipe_m", context) or 0.0
            ),
            nominal_restoring_force_n=_require_float(raw, "nominal_restoring_force_n", context),
            nominal_running_force_n=_require_float(raw, "nominal_running_force_n", context),
            axial_force_ratio=_optional_float(raw, "axial_force_ratio", context),
            tangential_force_ratio=_optional_float(raw, "tangential_force_ratio", context),
            blade_power_law_k=_optional_float(raw, "blade_power_law_k", context),
            blade_power_law_p=(
                _optional_float(raw, "blade_power_law_p", context) or 1.0
            ),
            min_contact_diameter_m=_optional_float(raw, "min_contact_diameter_m", context),
            max_contact_diameter_m=_optional_float(raw, "max_contact_diameter_m", context),
            spacing_hint_m=_require_float(raw, "spacing_hint_m", context),
            count_hint=count_hint,
            installation_md_m=installation_md_m,
        )
        spec.validate(context)
        return spec

    def validate(self, context: str) -> None:
        if not self.type:
            raise ConfigError(f"{context}.type must be non-empty.")
        if self.support_outer_diameter_m <= 0.0:
            raise ConfigError(f"{context}.support_outer_diameter_m must be positive.")
        if self.number_of_bows <= 0:
            raise ConfigError(f"{context}.number_of_bows must be at least one.")
        if self.inner_clearance_to_pipe_m < 0.0:
            raise ConfigError(f"{context}.inner_clearance_to_pipe_m must be non-negative.")
        if self.nominal_restoring_force_n <= 0.0:
            raise ConfigError(f"{context}.nominal_restoring_force_n must be positive.")
        if self.nominal_running_force_n <= 0.0:
            raise ConfigError(f"{context}.nominal_running_force_n must be positive.")
        if self.axial_force_ratio is not None and self.axial_force_ratio < 0.0:
            raise ConfigError(f"{context}.axial_force_ratio must be non-negative when provided.")
        if self.tangential_force_ratio is not None and self.tangential_force_ratio < 0.0:
            raise ConfigError(
                f"{context}.tangential_force_ratio must be non-negative when provided."
            )
        if self.blade_power_law_k is not None and self.blade_power_law_k <= 0.0:
            raise ConfigError(f"{context}.blade_power_law_k must be positive when provided.")
        if self.blade_power_law_p <= 0.0:
            raise ConfigError(f"{context}.blade_power_law_p must be positive.")
        if self.min_contact_diameter_m is not None and self.min_contact_diameter_m <= 0.0:
            raise ConfigError(f"{context}.min_contact_diameter_m must be positive when provided.")
        if self.max_contact_diameter_m is not None and self.max_contact_diameter_m <= 0.0:
            raise ConfigError(f"{context}.max_contact_diameter_m must be positive when provided.")
        if (
            self.min_contact_diameter_m is not None
            and self.max_contact_diameter_m is not None
            and self.min_contact_diameter_m > self.max_contact_diameter_m
        ):
            raise ConfigError(
                f"{context}.min_contact_diameter_m cannot exceed {context}.max_contact_diameter_m."
            )
        if self.spacing_hint_m <= 0.0:
            raise ConfigError(f"{context}.spacing_hint_m must be positive.")
        if self.count_hint is not None and self.count_hint <= 0:
            raise ConfigError(f"{context}.count_hint must be at least one when provided.")
        previous_md = -1.0
        for installation_index, installation_md in enumerate(self.installation_md_m or []):
            if installation_md < 0.0:
                raise ConfigError(f"{context}.installation_md_m[{installation_index}] must be non-negative.")
            if installation_md <= previous_md:
                raise ConfigError(f"{context}.installation_md_m must be strictly increasing.")
            previous_md = installation_md

    def explicit_installation_count(self) -> int:
        return len(self.installation_md_m or [])

    @property
    def centralizer_type(self) -> str:
        return self.type

    def resolved_support_outer_diameter_m(self) -> float:
        return self.support_outer_diameter_m if self.support_outer_diameter_m > 0.0 else self.outer_diameter_m

    def effective_contact_diameter_m(self) -> float:
        diameter_m = self.resolved_support_outer_diameter_m()
        if self.min_contact_diameter_m is not None:
            diameter_m = max(diameter_m, self.min_contact_diameter_m)
        if self.max_contact_diameter_m is not None:
            diameter_m = min(diameter_m, self.max_contact_diameter_m)
        return diameter_m

    def spacing_based_estimate(self, coverage_length_m: float) -> int:
        if self.count_hint is not None:
            return self.count_hint
        if self.installation_md_m:
            return len(self.installation_md_m)
        if coverage_length_m <= 0.0 or self.spacing_hint_m <= 0.0:
            return 0
        return max(1, math.floor(coverage_length_m / self.spacing_hint_m) + 1)


@dataclass(slots=True)
class CentralizerSummaryModel:
    spec_count: int
    explicit_installation_count: int
    count_hint_total: int
    spacing_based_installation_estimate: int
    expanded_installation_count: int
    max_outer_diameter_m: float
    min_nominal_radial_clearance_m: float


@dataclass(slots=True)
class CentralizerConfigModel:
    name: str
    centralizers: list[CentralizerSpecModel]

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "CentralizerConfigModel":
        context = "centralizers"
        centralizers_raw = _expect_sequence(raw.get("centralizers"), f"{context}.centralizers")
        centralizers = [
            CentralizerSpecModel.from_dict(
                _expect_mapping(item, f"{context}.centralizers[{index}]"),
                index,
            )
            for index, item in enumerate(centralizers_raw)
        ]
        model = cls(name=_require_text(raw, "name", context), centralizers=centralizers)
        model.validate()
        return model

    def validate(self) -> None:
        for index, spec in enumerate(self.centralizers):
            spec.validate(f"centralizers.centralizers[{index}]")

    def summary(self, coverage_length_m: float, hole_diameter_m: float) -> CentralizerSummaryModel:
        explicit_installation_count = sum(spec.explicit_installation_count() for spec in self.centralizers)
        count_hint_total = sum(spec.count_hint or 0 for spec in self.centralizers)
        spacing_based_installation_estimate = sum(
            spec.spacing_based_estimate(coverage_length_m) for spec in self.centralizers
        )
        max_outer_diameter_m = max(
            (spec.resolved_support_outer_diameter_m() for spec in self.centralizers),
            default=0.0,
        )
        clearances = [
            0.5 * (hole_diameter_m - spec.effective_contact_diameter_m())
            for spec in self.centralizers
            if hole_diameter_m > 0.0
        ]
        return CentralizerSummaryModel(
            spec_count=len(self.centralizers),
            explicit_installation_count=explicit_installation_count,
            count_hint_total=count_hint_total,
            spacing_based_installation_estimate=spacing_based_installation_estimate,
            expanded_installation_count=spacing_based_installation_estimate,
            max_outer_diameter_m=max_outer_diameter_m,
            min_nominal_radial_clearance_m=min(clearances) if clearances else 0.0,
        )


@dataclass(slots=True)
class CaseDefinition:
    name: str
    well: str
    string: str
    centralizers: str
    output_json: str | None = None
    operation_mode: str = "run_in"
    discretization_step_m: float | None = None
    global_solver_max_iterations: int | None = None
    contact_penalty_scale: float | None = None
    coupling_max_iterations: int | None = None
    coupling_tolerance_n: float | None = None
    coupling_torque_tolerance_n_m: float | None = None
    relaxation_factor: float | None = None
    frame_method: str = "parallel-transport"

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "CaseDefinition":
        context = "case"
        return cls(
            name=_require_text(raw, "name", context),
            well=_require_text(raw, "well", context),
            string=_require_text(raw, "string", context),
            centralizers=_require_text(raw, "centralizers", context),
            output_json=_optional_text(raw, "output_json"),
            operation_mode=_optional_text(raw, "operation_mode", "run_in") or "run_in",
            discretization_step_m=_optional_float(raw, "discretization_step_m", context),
            global_solver_max_iterations=(
                None
                if raw.get("global_solver_max_iterations") is None
                else _require_int(raw, "global_solver_max_iterations", context)
            ),
            contact_penalty_scale=_optional_float(raw, "contact_penalty_scale", context),
            coupling_max_iterations=(
                None
                if raw.get("coupling_max_iterations") is None
                else _require_int(raw, "coupling_max_iterations", context)
            ),
            coupling_tolerance_n=_optional_float(raw, "coupling_tolerance_n", context),
            coupling_torque_tolerance_n_m=_optional_float(
                raw,
                "coupling_torque_tolerance_n_m",
                context,
            ),
            relaxation_factor=_optional_float(raw, "relaxation_factor", context),
            frame_method=_optional_text(raw, "frame_method", "parallel-transport")
            or "parallel-transport",
        )

    def validate(self) -> None:
        if self.operation_mode not in SUPPORTED_OPERATION_MODES:
            raise ConfigError(
                "case.operation_mode must be one of: run_in, pull_out, rotate_in_place."
            )
        if self.discretization_step_m is not None and self.discretization_step_m <= 0.0:
            raise ConfigError("case.discretization_step_m must be positive when provided.")
        if self.global_solver_max_iterations is not None and self.global_solver_max_iterations <= 0:
            raise ConfigError("case.global_solver_max_iterations must be at least one when provided.")
        if self.contact_penalty_scale is not None and self.contact_penalty_scale <= 0.0:
            raise ConfigError("case.contact_penalty_scale must be positive when provided.")
        if self.coupling_max_iterations is not None and self.coupling_max_iterations <= 0:
            raise ConfigError("case.coupling_max_iterations must be at least one when provided.")
        if self.coupling_tolerance_n is not None and self.coupling_tolerance_n <= 0.0:
            raise ConfigError("case.coupling_tolerance_n must be positive when provided.")
        if (
            self.coupling_torque_tolerance_n_m is not None
            and self.coupling_torque_tolerance_n_m <= 0.0
        ):
            raise ConfigError(
                "case.coupling_torque_tolerance_n_m must be positive when provided."
            )
        if self.relaxation_factor is not None and (
            self.relaxation_factor <= 0.0 or self.relaxation_factor > 1.0
        ):
            raise ConfigError("case.relaxation_factor must stay within (0, 1].")
        if self.frame_method != "parallel-transport":
            raise ConfigError("case.frame_method must currently be 'parallel-transport'.")


@dataclass(slots=True)
class LoadedCase:
    case_path: Path
    definition: CaseDefinition
    well_path: Path
    string_path: Path
    centralizers_path: Path
    well: WellModel
    string: StringConfigModel
    centralizers: CentralizerConfigModel

    def validate(self) -> None:
        self.definition.validate()
        self.well.validate()
        self.string.validate()
        self.centralizers.validate()
        final_well_md_m = self.well.final_measured_depth_m
        hole_diameter_m = self.reference_hole_diameter_m

        for index, section in enumerate(self.string.sections):
            if section.md_end_m > final_well_md_m:
                raise ConfigError(
                    f"string.sections[{index}].md_end_m cannot exceed the final well MD."
                )
            if hole_diameter_m > 0.0 and section.outer_diameter_m > hole_diameter_m:
                raise ConfigError(
                    f"string.sections[{index}].outer_diameter_m cannot exceed well.hole_diameter_m."
                )

        for index, spec in enumerate(self.centralizers.centralizers):
            if hole_diameter_m > 0.0 and spec.effective_contact_diameter_m() > hole_diameter_m:
                raise ConfigError(
                    f"centralizers.centralizers[{index}].support_outer_diameter_m cannot exceed well.hole_diameter_m."
                )
            for installation_index, installation_md in enumerate(spec.installation_md_m or []):
                if installation_md > final_well_md_m:
                    raise ConfigError(
                        "centralizers.centralizers"
                        f"[{index}].installation_md_m[{installation_index}] cannot exceed the final well MD."
                    )

    @property
    def reference_hole_diameter_m(self) -> float:
        return 0.0 if self.well.hole_diameter_m is None else self.well.hole_diameter_m

    @property
    def fluid_density_kg_per_m3(self) -> float:
        return self.well.fluid_density_kg_per_m3

    @property
    def discretization_step_m(self) -> float:
        return 30.0 if self.definition.discretization_step_m is None else self.definition.discretization_step_m

    @property
    def operation_mode(self) -> str:
        return self.definition.operation_mode

    @property
    def global_solver_max_iterations(self) -> int:
        return (
            8
            if self.definition.global_solver_max_iterations is None
            else self.definition.global_solver_max_iterations
        )

    @property
    def contact_penalty_scale(self) -> float:
        return (
            25.0
            if self.definition.contact_penalty_scale is None
            else self.definition.contact_penalty_scale
        )

    @property
    def coupling_max_iterations(self) -> int:
        return (
            6
            if self.definition.coupling_max_iterations is None
            else self.definition.coupling_max_iterations
        )

    @property
    def coupling_tolerance_n(self) -> float:
        return (
            25.0
            if self.definition.coupling_tolerance_n is None
            else self.definition.coupling_tolerance_n
        )

    @property
    def coupling_torque_tolerance_n_m(self) -> float:
        return (
            5.0
            if self.definition.coupling_torque_tolerance_n_m is None
            else self.definition.coupling_torque_tolerance_n_m
        )

    @property
    def relaxation_factor(self) -> float:
        return (
            0.5
            if self.definition.relaxation_factor is None
            else self.definition.relaxation_factor
        )

    @property
    def frame_method(self) -> str:
        return self.definition.frame_method

    def trajectory_summary(self) -> TrajectorySummaryModel:
        return self.well.summary()

    def trajectory_nodes(self) -> list[TrajectoryNodeModel]:
        return self.well.derived_nodes()

    def string_summary(self) -> StringSummaryModel:
        total_length_m = sum(section.length_m for section in self.string.sections)
        total_weight_n = sum(
            section.length_m * section.linear_weight_n_per_m for section in self.string.sections
        )
        total_effective_weight_n = sum(
            section.length_m * section.effective_line_weight_n_per_m(self.fluid_density_kg_per_m3)
            for section in self.string.sections
        )
        max_outer_diameter_m = max(section.outer_diameter_m for section in self.string.sections)
        min_inner_diameter_m = min(section.inner_diameter_m for section in self.string.sections)
        average_friction_coefficient = (
            0.0
            if total_length_m <= 0.0
            else sum(
                section.friction_coefficient * section.length_m
                for section in self.string.sections
            )
            / total_length_m
        )
        average_density_kg_per_m3 = (
            0.0
            if total_length_m <= 0.0
            else sum(section.density_kg_per_m3 * section.length_m for section in self.string.sections)
            / total_length_m
        )
        return StringSummaryModel(
            section_count=len(self.string.sections),
            total_length_m=total_length_m,
            total_weight_n=total_weight_n,
            total_effective_weight_n=total_effective_weight_n,
            max_outer_diameter_m=max_outer_diameter_m,
            min_inner_diameter_m=min_inner_diameter_m,
            average_friction_coefficient=average_friction_coefficient,
            average_density_kg_per_m3=average_density_kg_per_m3,
        )

    def centralizer_summary(self) -> CentralizerSummaryModel:
        return self.centralizers.summary(self.string_summary().total_length_m, self.reference_hole_diameter_m)

    def section_summaries(self) -> list[StringSectionSummaryModel]:
        return self.string.section_summaries(self.reference_hole_diameter_m)

    def total_string_length_m(self) -> float:
        return self.string_summary().total_length_m

    def total_string_weight_n(self) -> float:
        return self.string_summary().total_weight_n

    def total_centralizer_count(self) -> int:
        return self.centralizer_summary().expanded_installation_count

    def minimum_nominal_radial_clearance_m(self) -> float:
        section_clearances = [summary.nominal_radial_clearance_m for summary in self.section_summaries()]
        centralizer_summary = self.centralizer_summary()
        candidates = [clearance for clearance in section_clearances if clearance or self.reference_hole_diameter_m > 0.0]
        if centralizer_summary.spec_count > 0:
            candidates.append(centralizer_summary.min_nominal_radial_clearance_m)
        return min(candidates) if candidates else 0.0
