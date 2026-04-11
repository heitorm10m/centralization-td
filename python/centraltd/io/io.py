from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import yaml

from ..models import (
    CaseDefinition,
    CentralizerConfigModel,
    ConfigError,
    LoadedCase,
    StringConfigModel,
    WellModel,
    _expect_mapping,
)


def read_yaml(path: str | Path) -> Any:
    file_path = Path(path).resolve()
    with file_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    return {} if data is None else data


def write_json(path: str | Path, payload: dict[str, Any]) -> Path:
    file_path = Path(path).resolve()
    file_path.parent.mkdir(parents=True, exist_ok=True)
    with file_path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
    return file_path


def resolve_case_reference(case_path: Path, reference: str) -> Path:
    return (case_path.parent / reference).resolve()


def load_case_bundle(case_path: str | Path) -> LoadedCase:
    resolved_case_path = Path(case_path).resolve()
    raw_case = _expect_mapping(read_yaml(resolved_case_path), "case file")
    definition = CaseDefinition.from_dict(raw_case)

    well_path = resolve_case_reference(resolved_case_path, definition.well)
    string_path = resolve_case_reference(resolved_case_path, definition.string)
    centralizers_path = resolve_case_reference(resolved_case_path, definition.centralizers)

    for dependency_path in (well_path, string_path, centralizers_path):
        if not dependency_path.exists():
            raise ConfigError(f"Referenced config file does not exist: {dependency_path}")

    raw_well = _expect_mapping(read_yaml(well_path), "well file")
    raw_string = _expect_mapping(read_yaml(string_path), "string file")
    raw_centralizers = _expect_mapping(read_yaml(centralizers_path), "centralizers file")

    loaded_case = LoadedCase(
        case_path=resolved_case_path,
        definition=definition,
        well_path=well_path,
        string_path=string_path,
        centralizers_path=centralizers_path,
        well=WellModel.from_dict(raw_well),
        string=StringConfigModel.from_dict(raw_string),
        centralizers=CentralizerConfigModel.from_dict(raw_centralizers),
    )
    loaded_case.validate()
    return loaded_case
