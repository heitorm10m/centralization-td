from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from centraltd.frames import build_local_frame_from_tangent  # noqa: E402
from centraltd.physics.beam_element import stiffness_matrix_local  # noqa: E402


def _beam_parameters() -> dict[str, float]:
    return {
        "length_m": 12.0,
        "young_modulus_pa": 2.07e11,
        "poisson_ratio": 0.29,
        "outer_radius_m": 0.0889,
        "inner_radius_m": 0.0762,
    }


def _second_moment_of_area_m4(outer_radius_m: float, inner_radius_m: float) -> float:
    return (math.pi / 4.0) * ((outer_radius_m**4) - (inner_radius_m**4))


def _polar_moment_of_area_m4(outer_radius_m: float, inner_radius_m: float) -> float:
    return (math.pi / 2.0) * ((outer_radius_m**4) - (inner_radius_m**4))


def test_beam_stiffness_matrix_has_expected_shape() -> None:
    matrix = stiffness_matrix_local(**_beam_parameters())

    assert matrix.shape == (12, 12)


def test_beam_stiffness_matrix_is_symmetric() -> None:
    matrix = stiffness_matrix_local(**_beam_parameters())

    assert np.allclose(matrix, matrix.T)


def test_axial_diagonal_term_equals_ea_over_l() -> None:
    parameters = _beam_parameters()
    matrix = stiffness_matrix_local(**parameters)
    area_m2 = math.pi * ((parameters["outer_radius_m"] ** 2) - (parameters["inner_radius_m"] ** 2))
    expected = parameters["young_modulus_pa"] * area_m2 / parameters["length_m"]

    assert matrix[2, 2] == pytest.approx(expected)


def test_torsional_diagonal_term_equals_gj_over_l() -> None:
    parameters = _beam_parameters()
    matrix = stiffness_matrix_local(**parameters)
    shear_modulus_pa = parameters["young_modulus_pa"] / (2.0 * (1.0 + parameters["poisson_ratio"]))
    polar_moment_m4 = _polar_moment_of_area_m4(
        parameters["outer_radius_m"],
        parameters["inner_radius_m"],
    )
    expected = shear_modulus_pa * polar_moment_m4 / parameters["length_m"]

    assert matrix[5, 5] == pytest.approx(expected)


def test_transverse_bending_term_equals_12_ei_over_l_cubed() -> None:
    parameters = _beam_parameters()
    matrix = stiffness_matrix_local(**parameters)
    second_moment_m4 = _second_moment_of_area_m4(
        parameters["outer_radius_m"],
        parameters["inner_radius_m"],
    )
    expected = 12.0 * parameters["young_modulus_pa"] * second_moment_m4 / (parameters["length_m"] ** 3)

    assert matrix[0, 0] == pytest.approx(expected)
    assert matrix[1, 1] == pytest.approx(expected)


def test_generic_deviated_frame_is_orthonormal() -> None:
    eu, ev, ew = build_local_frame_from_tangent((0.3, 0.4, 0.8660254037844386))

    assert np.linalg.norm(eu) == pytest.approx(1.0)
    assert np.linalg.norm(ev) == pytest.approx(1.0)
    assert np.linalg.norm(ew) == pytest.approx(1.0)
    assert float(np.dot(eu, ev)) == pytest.approx(0.0, abs=1.0e-12)
    assert float(np.dot(eu, ew)) == pytest.approx(0.0, abs=1.0e-12)
    assert float(np.dot(ev, ew)) == pytest.approx(0.0, abs=1.0e-12)


def test_nearly_vertical_frame_has_no_nan_or_inf() -> None:
    eu, ev, ew = build_local_frame_from_tangent((1.0e-10, -2.0e-10, 1.0))
    frame_matrix = np.asarray((eu, ev, ew), dtype=float)

    assert np.all(np.isfinite(frame_matrix))
    assert np.linalg.norm(eu) == pytest.approx(1.0)
    assert np.linalg.norm(ev) == pytest.approx(1.0)
    assert np.linalg.norm(ew) == pytest.approx(1.0)
    assert float(np.dot(eu, ev)) == pytest.approx(0.0, abs=1.0e-12)
    assert float(np.dot(eu, ew)) == pytest.approx(0.0, abs=1.0e-12)
    assert float(np.dot(ev, ew)) == pytest.approx(0.0, abs=1.0e-12)


def test_exactly_vertical_frame_is_continuous_and_finite() -> None:
    previous_eu, previous_ev, previous_ew = build_local_frame_from_tangent((1.0e-6, 0.0, 1.0))
    eu, ev, ew = build_local_frame_from_tangent(
        (0.0, 0.0, 1.0),
        previous_eu,
        previous_ev,
    )
    frame_matrix = np.asarray((eu, ev, ew), dtype=float)

    assert np.all(np.isfinite(frame_matrix))
    assert np.linalg.norm(eu) == pytest.approx(1.0)
    assert np.linalg.norm(ev) == pytest.approx(1.0)
    assert np.linalg.norm(ew) == pytest.approx(1.0)
    assert float(np.dot(eu, ev)) == pytest.approx(0.0, abs=1.0e-12)
    assert float(np.dot(eu, ew)) == pytest.approx(0.0, abs=1.0e-12)
    assert float(np.dot(ev, ew)) == pytest.approx(0.0, abs=1.0e-12)
    assert float(np.dot(eu, previous_eu)) > 0.999
    assert float(np.dot(ev, previous_ev)) > 0.999
    assert float(np.dot(ew, previous_ew)) > 0.999
