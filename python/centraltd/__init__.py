from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

__all__ = [
    "CORE_AVAILABLE",
    "__version__",
    "format_case_summary",
    "load_case",
    "run_stub_case",
]

__version__ = "0.1.0"


def _load_workspace_built_core() -> ModuleType | None:
    package_dir = Path(__file__).resolve().parent
    repo_root = package_dir.parents[1]
    extension_patterns = ("_core*.pyd", "_core*.so", "_core*.dylib")

    candidates: list[Path] = []
    for build_dir in sorted(repo_root.glob("build*")):
        if not build_dir.is_dir():
            continue
        build_package_dir = build_dir / "python" / "centraltd"
        if not build_package_dir.is_dir():
            continue
        for pattern in extension_patterns:
            candidates.extend(build_package_dir.glob(pattern))

    if not candidates:
        return None

    candidate = max(candidates, key=lambda path: path.stat().st_mtime)
    spec = importlib.util.spec_from_file_location("centraltd._core", candidate)
    if spec is None or spec.loader is None:
        return None

    module = importlib.util.module_from_spec(spec)
    sys.modules["centraltd._core"] = module
    spec.loader.exec_module(module)
    return module


try:
    _core = _load_workspace_built_core()
    if _core is None:
        from . import _core as _core  # type: ignore[attr-defined]

    CORE_AVAILABLE = True
except ImportError:
    _core = None
    CORE_AVAILABLE = False

from .runner import format_case_summary, load_case, run_stub_case

