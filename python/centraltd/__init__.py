from __future__ import annotations

from .runner import format_case_summary, load_case, run_stub_case

__all__ = [
    "CORE_AVAILABLE",
    "__version__",
    "format_case_summary",
    "load_case",
    "run_stub_case",
]

__version__ = "0.1.0"

try:
    from . import _core as _core  # type: ignore[attr-defined]

    CORE_AVAILABLE = True
except ImportError:
    _core = None
    CORE_AVAILABLE = False

