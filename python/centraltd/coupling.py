from __future__ import annotations

from dataclasses import dataclass

from .mechanics import (
    CentralizerPlacementModel,
    MechanicalSegmentResultModel,
    MechanicalSummaryModel,
    NormalReactionPointModel,
    compute_buoyant_axial_load_profile,
    discretize_case,
    run_mechanical_with_axial_profile,
)
from .models import LoadedCase
from .torque_drag import (
    AxialForcePointModel,
    CentralizerFrictionPointModel,
    TorqueDragResultModel,
    TorquePointModel,
    run_torque_drag_baseline,
)


@dataclass(slots=True)
class CouplingResultModel:
    status: str
    iteration_count: int
    converged: bool
    converged_axial_profile: list[AxialForcePointModel]
    converged_normal_reaction_profile: list[NormalReactionPointModel]
    converged_torque_profile: list[TorquePointModel]
    mechanical_summary: MechanicalSummaryModel
    mechanical_profile: list[MechanicalSegmentResultModel]
    torque_drag_result: TorqueDragResultModel
    placements: list[CentralizerPlacementModel]


def _select_axial_profile(
    loaded_case: LoadedCase,
    buoyant_axial_profile: list[float],
    torque_drag_result: TorqueDragResultModel,
) -> tuple[list[float], float]:
    if loaded_case.operation_mode == "pull_out":
        return (
            [point.axial_force_n for point in torque_drag_result.axial_force_pull_out_profile],
            torque_drag_result.hookload_pull_out_n,
        )
    if loaded_case.operation_mode == "rotate_in_place":
        return buoyant_axial_profile, 0.0
    return (
        [point.axial_force_n for point in torque_drag_result.axial_force_run_in_profile],
        torque_drag_result.hookload_run_in_n,
    )


def _axial_points(measured_depths_m: list[float], axial_loads_n: list[float]) -> list[AxialForcePointModel]:
    return [
        AxialForcePointModel(measured_depth_m=measured_depth_m, axial_force_n=axial_force_n)
        for measured_depth_m, axial_force_n in zip(measured_depths_m, axial_loads_n, strict=True)
    ]


def _normal_reaction_points(
    mechanical_profile: list[MechanicalSegmentResultModel],
) -> list[NormalReactionPointModel]:
    return [
        NormalReactionPointModel(
            measured_depth_m=segment.measured_depth_center_m,
            support_normal_reaction_estimate_n=segment.support_normal_reaction_estimate_n,
            body_normal_reaction_estimate_n=segment.body_normal_reaction_estimate_n,
            normal_reaction_estimate_n=segment.normal_reaction_estimate_n,
        )
        for segment in mechanical_profile
    ]


def run_coupled_global_baseline(loaded_case: LoadedCase) -> CouplingResultModel:
    segments, placements = discretize_case(loaded_case)
    buoyant_axial_profile_n, buoyant_top_hookload_n = compute_buoyant_axial_load_profile(segments)
    measured_depths_m = [segment.measured_depth_center_m for segment in segments]
    current_axial_profile_n = list(buoyant_axial_profile_n)
    current_top_hookload_n = buoyant_top_hookload_n

    mechanical_summary = None
    mechanical_profile = []
    torque_drag_result = TorqueDragResultModel(
        status="phase9-reduced-bow-spring-td-baseline",
        operation_mode=loaded_case.operation_mode,
        axial_force_run_in_profile=[],
        axial_force_pull_out_profile=[],
        torque_profile=[],
        centralizer_axial_friction_profile=[],
        centralizer_tangential_friction_profile=[],
        centralizer_torque_profile=[],
        hookload_run_in_n=0.0,
        hookload_pull_out_n=0.0,
        drag_run_in_n=0.0,
        drag_pull_out_n=0.0,
        estimated_surface_torque_n_m=0.0,
    )
    iteration_count = 0
    converged = False

    for iteration_index in range(loaded_case.coupling_max_iterations):
        mechanical_summary, mechanical_profile, _ = run_mechanical_with_axial_profile(
            loaded_case,
            current_axial_profile_n,
            current_top_hookload_n,
        )
        torque_drag_result = run_torque_drag_baseline(
            loaded_case,
            mechanical_profile,
            reference_buoyant_hookload_n=buoyant_top_hookload_n,
        )
        target_axial_profile_n, target_top_hookload_n = _select_axial_profile(
            loaded_case,
            list(buoyant_axial_profile_n),
            torque_drag_result,
        )
        if loaded_case.operation_mode == "rotate_in_place":
            target_top_hookload_n = buoyant_top_hookload_n

        maximum_profile_update_n = 0.0
        next_axial_profile_n: list[float] = []
        for current_load_n, target_load_n in zip(
            current_axial_profile_n,
            target_axial_profile_n,
            strict=True,
        ):
            relaxed_load_n = (
                ((1.0 - loaded_case.relaxation_factor) * current_load_n)
                + (loaded_case.relaxation_factor * target_load_n)
            )
            relaxed_load_n = max(0.0, relaxed_load_n)
            next_axial_profile_n.append(relaxed_load_n)
            maximum_profile_update_n = max(
                maximum_profile_update_n,
                abs(relaxed_load_n - current_load_n),
            )

        current_axial_profile_n = next_axial_profile_n
        current_top_hookload_n = max(
            0.0,
            ((1.0 - loaded_case.relaxation_factor) * current_top_hookload_n)
            + (loaded_case.relaxation_factor * target_top_hookload_n),
        )
        iteration_count = iteration_index + 1

        if maximum_profile_update_n <= loaded_case.coupling_tolerance_n:
            converged = True
            break

    mechanical_summary, mechanical_profile, _ = run_mechanical_with_axial_profile(
        loaded_case,
        current_axial_profile_n,
        current_top_hookload_n,
    )
    torque_drag_result = run_torque_drag_baseline(
        loaded_case,
        mechanical_profile,
        reference_buoyant_hookload_n=buoyant_top_hookload_n,
    )

    return CouplingResultModel(
        status="converged" if converged else "max_iterations_reached",
        iteration_count=iteration_count,
        converged=converged,
        converged_axial_profile=_axial_points(measured_depths_m, current_axial_profile_n),
        converged_normal_reaction_profile=_normal_reaction_points(mechanical_profile),
        converged_torque_profile=list(torque_drag_result.torque_profile),
        mechanical_summary=mechanical_summary,
        mechanical_profile=mechanical_profile,
        torque_drag_result=torque_drag_result,
        placements=placements,
    )
