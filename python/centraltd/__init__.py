from __future__ import annotations

import sys

__all__ = [
    "__version__",
    "format_benchmark_summary",
    "format_bow_spring_calibration_summary",
    "format_case_summary",
    "load_case",
    "run_benchmark_suite",
    "run_bow_spring_calibration",
    "run_stub_case",
]

__version__ = "0.1.0"

from .benchmarking import format_benchmark_summary, run_benchmark_suite
from .io import io as _io_module
from .physics import bow_spring as _bow_spring_module
from .physics import friction as _friction_module
from .runner import format_case_summary, load_case, run_stub_case
from .solver import calibration as _calibration_module
from .solver import coupling as _coupling_module
from .solver import mechanics as _mechanics_module
from .solver.calibration import format_bow_spring_calibration_summary, run_bow_spring_calibration

io = _io_module
bow_spring = _bow_spring_module
local_tangential_model = _friction_module
torsional_reduced_model = _friction_module
calibration = _calibration_module
coupling = _coupling_module
mechanics = _mechanics_module

sys.modules.setdefault(__name__ + ".io", _io_module)
sys.modules.setdefault(__name__ + ".bow_spring", _bow_spring_module)
sys.modules.setdefault(__name__ + ".local_tangential_model", _friction_module)
sys.modules.setdefault(__name__ + ".torsional_reduced_model", _friction_module)
sys.modules.setdefault(__name__ + ".calibration", _calibration_module)
sys.modules.setdefault(__name__ + ".coupling", _coupling_module)
sys.modules.setdefault(__name__ + ".mechanics", _mechanics_module)
