from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any


class ConfigError(ValueError):
    """Raised when a YAML file is missing required fields or contains invalid values."""


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


def _require_int(raw: Mapping[str, Any], key: str, context: str) -> int:
    value = raw.get(key)
    if not isinstance(value, int) or isinstance(value, bool):
        raise ConfigError(f"{context}.{key} must be an integer.")
    return value


@dataclass(slots=True)
class WellTrajectoryPointModel:
    measured_depth_m: float
    inclination_rad: float
    azimuth_rad: float
    tvd_m: float
    northing_m: float = 0.0
    easting_m: float = 0.0

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any], index: int) -> "WellTrajectoryPointModel":
        context = f"trajectory[{index}]"
        point = cls(
            measured_depth_m=_require_float(raw, "measured_depth_m", context),
            inclination_rad=_require_float(raw, "inclination_rad", context),
            azimuth_rad=_require_float(raw, "azimuth_rad", context),
            tvd_m=_require_float(raw, "tvd_m", context),
            northing_m=float(raw.get("northing_m", 0.0)),
            easting_m=float(raw.get("easting_m", 0.0)),
        )
        if point.measured_depth_m < 0.0:
            raise ConfigError(f"{context}.measured_depth_m must be non-negative.")
        if point.tvd_m < 0.0:
            raise ConfigError(f"{context}.tvd_m must be non-negative.")
        return point


@dataclass(slots=True)
class WellModel:
    name: str
    trajectory: list[WellTrajectoryPointModel]

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "WellModel":
        context = "well"
        trajectory_raw = _expect_sequence(raw.get("trajectory"), f"{context}.trajectory")
        trajectory = [
            WellTrajectoryPointModel.from_dict(_expect_mapping(item, f"{context}.trajectory[{index}]"), index)
            for index, item in enumerate(trajectory_raw)
        ]
        if not trajectory:
            raise ConfigError("well.trajectory must contain at least one point.")
        previous_md = -1.0
        for index, point in enumerate(trajectory):
            if point.measured_depth_m <= previous_md:
                raise ConfigError(
                    f"well.trajectory[{index}].measured_depth_m must be strictly increasing."
                )
            previous_md = point.measured_depth_m
        return cls(name=_require_text(raw, "name", context), trajectory=trajectory)

    @property
    def final_measured_depth_m(self) -> float:
        return self.trajectory[-1].measured_depth_m


@dataclass(slots=True)
class StringSectionModel:
    name: str
    length_m: float
    outer_diameter_m: float
    inner_diameter_m: float
    unit_weight_n_per_m: float
    grade: str = "unspecified"

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any], index: int) -> "StringSectionModel":
        context = f"sections[{index}]"
        section = cls(
            name=_require_text(raw, "name", context),
            length_m=_require_float(raw, "length_m", context),
            outer_diameter_m=_require_float(raw, "outer_diameter_m", context),
            inner_diameter_m=_require_float(raw, "inner_diameter_m", context),
            unit_weight_n_per_m=_require_float(raw, "unit_weight_n_per_m", context),
            grade=_optional_text(raw, "grade", "unspecified") or "unspecified",
        )
        if section.length_m <= 0.0:
            raise ConfigError(f"{context}.length_m must be positive.")
        if section.outer_diameter_m <= 0.0 or section.inner_diameter_m <= 0.0:
            raise ConfigError(f"{context}.diameters must be positive.")
        if section.inner_diameter_m >= section.outer_diameter_m:
            raise ConfigError(f"{context}.inner_diameter_m must be smaller than outer_diameter_m.")
        if section.unit_weight_n_per_m <= 0.0:
            raise ConfigError(f"{context}.unit_weight_n_per_m must be positive.")
        return section


@dataclass(slots=True)
class StringConfigModel:
    name: str
    sections: list[StringSectionModel]

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "StringConfigModel":
        context = "string"
        sections_raw = _expect_sequence(raw.get("sections"), f"{context}.sections")
        sections = [
            StringSectionModel.from_dict(_expect_mapping(item, f"{context}.sections[{index}]"), index)
            for index, item in enumerate(sections_raw)
        ]
        if not sections:
            raise ConfigError("string.sections must contain at least one item.")
        return cls(name=_require_text(raw, "name", context), sections=sections)


@dataclass(slots=True)
class CentralizerSpecModel:
    name: str
    outer_diameter_m: float
    start_md_m: float
    spacing_m: float
    count: int
    type: str = "bow-spring"

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any], index: int) -> "CentralizerSpecModel":
        context = f"centralizers[{index}]"
        spec = cls(
            name=_require_text(raw, "name", context),
            outer_diameter_m=_require_float(raw, "outer_diameter_m", context),
            start_md_m=_require_float(raw, "start_md_m", context),
            spacing_m=_require_float(raw, "spacing_m", context),
            count=_require_int(raw, "count", context),
            type=_optional_text(raw, "type", "bow-spring") or "bow-spring",
        )
        if spec.outer_diameter_m <= 0.0:
            raise ConfigError(f"{context}.outer_diameter_m must be positive.")
        if spec.start_md_m < 0.0:
            raise ConfigError(f"{context}.start_md_m must be non-negative.")
        if spec.spacing_m < 0.0:
            raise ConfigError(f"{context}.spacing_m must be non-negative.")
        if spec.count <= 0:
            raise ConfigError(f"{context}.count must be at least one.")
        return spec


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
                _expect_mapping(item, f"{context}.centralizers[{index}]"), index
            )
            for index, item in enumerate(centralizers_raw)
        ]
        return cls(name=_require_text(raw, "name", context), centralizers=centralizers)


@dataclass(slots=True)
class CaseDefinition:
    name: str
    well: str
    string: str
    centralizers: str
    output_json: str | None = None

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "CaseDefinition":
        context = "case"
        return cls(
            name=_require_text(raw, "name", context),
            well=_require_text(raw, "well", context),
            string=_require_text(raw, "string", context),
            centralizers=_require_text(raw, "centralizers", context),
            output_json=_optional_text(raw, "output_json"),
        )


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

    def total_string_length_m(self) -> float:
        return sum(section.length_m for section in self.string.sections)

    def total_string_weight_n(self) -> float:
        return sum(section.length_m * section.unit_weight_n_per_m for section in self.string.sections)

    def total_centralizer_count(self) -> int:
        return sum(spec.count for spec in self.centralizers.centralizers)

