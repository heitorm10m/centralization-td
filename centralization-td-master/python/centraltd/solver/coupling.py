from __future__ import annotations

from dataclasses import dataclass

from .mechanics import (
    CentralizerPlacementModel,
    MechanicalSegmentResultModel,
    MechanicalSummaryModel,
    NormalReactionPointModel,
    compute_buoyant_axial_load_profile,
    discretize_case,
    run_mechanical_with_axial_profile as run_mechanical_with_axial_profile_6dof_active,
)
from ..models import LoadedCase
from ..torque_drag import (
    AxialForcePointModel,
    CentralizerFrictionPointModel,
    TorquePartitionSummaryModel,
    TorqueDragResultModel,
    TorquePointModel,
    run_torque_drag_baseline,
)
from ..physics.friction import (
    ReducedTorqueAccumulationPointModel,
    TorsionalStatePointModel,
    run_reduced_torsional_model,
)


@dataclass(slots=True)
class CouplingResultModel:
    status: str
    iteration_count: int
    maximum_profile_update_n: float
    maximum_torque_update_n_m: float
    maximum_torsional_load_update_n_m: float
    converged: bool
    torque_feedback_mode: str
    torsional_feedback_status: str
    converged_axial_profile: list[AxialForcePointModel]
    converged_normal_reaction_profile: list[NormalReactionPointModel]
    converged_torque_profile: list[TorquePointModel]
    reduced_torque_accumulation_profile: list[ReducedTorqueAccumulationPointModel]
    torsional_state_profile: list[TorsionalStatePointModel]
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
    current_torsional_load_profile_n_m = [0.0 for _ in segments]

    mechanical_summary = None
    mechanical_profile = []
    torque_drag_result = TorqueDragResultModel(
        status="phase14-reduced-unified-local-tangential-torque-baseline",
        operation_mode=loaded_case.operation_mode,
        axial_force_run_in_profile=[],
        axial_force_pull_out_profile=[],
        torque_profile=[],
        body_axial_friction_profile=[],
        body_torque_profile=[],
        centralizer_axial_friction_profile=[],
        centralizer_tangential_friction_profile=[],
        centralizer_tangential_direction_profile=[],
        centralizer_tangential_friction_vector_profile=[],
        centralizer_torque_profile=[],
        centralizer_torque_breakdown_profile=[],
        local_tangential_interaction_state=[],
        torque_partition_summary=TorquePartitionSummaryModel(
            body_surface_torque_n_m=0.0,
            centralizer_surface_torque_n_m=0.0,
            total_surface_torque_n_m=0.0,
            body_axial_friction_sum_n=0.0,
            centralizer_axial_friction_sum_n=0.0,
            body_tangential_friction_sum_n=0.0,
            centralizer_tangential_friction_sum_n=0.0,
            status="phase14-reduced-body-centralizer-torque-partition",
        ),
        hookload_run_in_n=0.0,
        hookload_pull_out_n=0.0,
        drag_run_in_n=0.0,
        drag_pull_out_n=0.0,
        estimated_surface_torque_n_m=0.0,
    )
    iteration_count = 0
    maximum_profile_update_n = 0.0
    maximum_torque_update_n_m = 0.0
    maximum_torsional_load_update_n_m = 0.0
    converged = False
    previous_torque_profile: list[TorquePointModel] = []
    reduced_torque_accumulation_profile: list[ReducedTorqueAccumulationPointModel] = []
    torsional_state_profile: list[TorsionalStatePointModel] = []
    torsional_feedback_status = "inactive"

    for iteration_index in range(loaded_case.coupling_max_iterations):
        # Active mechanics path: the public mechanics entrypoint now carries the
        # 6-DOF nodal state while the legacy reduced branch remains preserved
        # internally for compatibility during the transition.
        mechanical_summary, mechanical_profile, _ = run_mechanical_with_axial_profile_6dof_active(
            loaded_case,
            current_axial_profile_n,
            current_top_hookload_n,
        )
        current_torsional_state_result = run_reduced_torsional_model(
            loaded_case,
            mechanical_profile,
            [
                TorquePointModel(
                    measured_depth_m=segment.measured_depth_center_m,
                    effective_contact_radius_m=0.0,
                    body_torque_increment_n_m=0.0,
                    centralizer_torque_increment_n_m=0.0,
                    local_torque_increment_n_m=0.0,
                    cumulative_torque_n_m=0.0,
                )
                for segment in mechanical_profile
            ],
            carried_torsional_load_profile_n_m=current_torsional_load_profile_n_m,
        )
        torque_drag_result = run_torque_drag_baseline(
            loaded_case,
            mechanical_profile,
            reference_buoyant_hookload_n=buoyant_top_hookload_n,
            reduced_torsional_load_profile_n_m=list(current_torsional_load_profile_n_m),
            reduced_twist_rate_profile_rad_per_m=[
                point.reduced_twist_rate_rad_per_m
                for point in current_torsional_state_result.torsional_state_profile
            ],
        )
        if previous_torque_profile and len(previous_torque_profile) == len(torque_drag_result.torque_profile):
            maximum_torque_update_n_m = max(
                abs(current_point.local_torque_increment_n_m - previous_point.local_torque_increment_n_m)
                for current_point, previous_point in zip(
                    torque_drag_result.torque_profile,
                    previous_torque_profile,
                    strict=True,
                )
            )
        else:
            maximum_torque_update_n_m = 0.0
        target_axial_profile_n, target_top_hookload_n = _select_axial_profile(
            loaded_case,
            list(buoyant_axial_profile_n),
            torque_drag_result,
        )
        target_torsional_result = run_reduced_torsional_model(
            loaded_case,
            mechanical_profile,
            torque_drag_result.torque_profile,
        )
        target_torsional_load_profile_n_m = [
            point.carried_torsional_load_n_m
            for point in target_torsional_result.reduced_torque_accumulation_profile
        ]
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
        next_torsional_load_profile_n_m: list[float] = []
        maximum_torsional_load_update_n_m = 0.0
        if iteration_index == 0:
            next_torsional_load_profile_n_m = list(target_torsional_load_profile_n_m)
        else:
            for current_load_n_m, target_load_n_m in zip(
                current_torsional_load_profile_n_m,
                target_torsional_load_profile_n_m,
                strict=True,
            ):
                relaxed_load_n_m = (
                    ((1.0 - loaded_case.relaxation_factor) * current_load_n_m)
                    + (loaded_case.relaxation_factor * target_load_n_m)
                )
                relaxed_load_n_m = max(0.0, relaxed_load_n_m)
                next_torsional_load_profile_n_m.append(relaxed_load_n_m)
                maximum_torsional_load_update_n_m = max(
                    maximum_torsional_load_update_n_m,
                    abs(relaxed_load_n_m - current_load_n_m),
                )
        carried_torsional_result = run_reduced_torsional_model(
            loaded_case,
            mechanical_profile,
            torque_drag_result.torque_profile,
            carried_torsional_load_profile_n_m=next_torsional_load_profile_n_m,
        )
        reduced_torque_accumulation_profile = list(
            carried_torsional_result.reduced_torque_accumulation_profile
        )
        torsional_state_profile = list(carried_torsional_result.torsional_state_profile)
        torsional_feedback_status = (
            "phase14-reduced-torsional-state-fed-into-unified-local-tangential-state"
            if any(
                point.body_feedback_applied or point.centralizer_feedback_applied
                for point in torque_drag_result.local_tangential_interaction_state
            )
            else current_torsional_state_result.status
        )

        current_axial_profile_n = next_axial_profile_n
        current_torsional_load_profile_n_m = next_torsional_load_profile_n_m
        current_top_hookload_n = max(
            0.0,
            ((1.0 - loaded_case.relaxation_factor) * current_top_hookload_n)
            + (loaded_case.relaxation_factor * target_top_hookload_n),
        )
        previous_torque_profile = list(torque_drag_result.torque_profile)
        iteration_count = iteration_index + 1

        if (
            maximum_profile_update_n <= loaded_case.coupling_tolerance_n
            and maximum_torque_update_n_m <= loaded_case.coupling_torque_tolerance_n_m
            and maximum_torsional_load_update_n_m <= loaded_case.coupling_torque_tolerance_n_m
        ):
            converged = True
            break

    mechanical_summary, mechanical_profile, _ = run_mechanical_with_axial_profile_6dof_active(
        loaded_case,
        current_axial_profile_n,
        current_top_hookload_n,
    )
    current_torsional_state_result = run_reduced_torsional_model(
        loaded_case,
        mechanical_profile,
        [
            TorquePointModel(
                measured_depth_m=segment.measured_depth_center_m,
                effective_contact_radius_m=0.0,
                body_torque_increment_n_m=0.0,
                centralizer_torque_increment_n_m=0.0,
                local_torque_increment_n_m=0.0,
                cumulative_torque_n_m=0.0,
            )
            for segment in mechanical_profile
        ],
        carried_torsional_load_profile_n_m=current_torsional_load_profile_n_m,
    )
    torque_drag_result = run_torque_drag_baseline(
        loaded_case,
        mechanical_profile,
        reference_buoyant_hookload_n=buoyant_top_hookload_n,
        reduced_torsional_load_profile_n_m=list(current_torsional_load_profile_n_m),
        reduced_twist_rate_profile_rad_per_m=[
            point.reduced_twist_rate_rad_per_m
            for point in current_torsional_state_result.torsional_state_profile
        ],
    )
    target_torsional_result = run_reduced_torsional_model(
        loaded_case,
        mechanical_profile,
        torque_drag_result.torque_profile,
    )
    target_torsional_load_profile_n_m = [
        point.carried_torsional_load_n_m
        for point in target_torsional_result.reduced_torque_accumulation_profile
    ]
    if previous_torque_profile and len(previous_torque_profile) == len(torque_drag_result.torque_profile):
        maximum_torque_update_n_m = max(
            abs(current_point.local_torque_increment_n_m - previous_point.local_torque_increment_n_m)
            for current_point, previous_point in zip(
                torque_drag_result.torque_profile,
                previous_torque_profile,
                strict=True,
            )
        )
    else:
        maximum_torque_update_n_m = 0.0
    if len(current_torsional_load_profile_n_m) == len(target_torsional_load_profile_n_m):
        maximum_torsional_load_update_n_m = max(
            abs(current_load_n_m - target_load_n_m)
            for current_load_n_m, target_load_n_m in zip(
                current_torsional_load_profile_n_m,
                target_torsional_load_profile_n_m,
                strict=True,
            )
        )
    else:
        current_torsional_load_profile_n_m = target_torsional_load_profile_n_m
        maximum_torsional_load_update_n_m = 0.0
    carried_torsional_result = run_reduced_torsional_model(
        loaded_case,
        mechanical_profile,
        torque_drag_result.torque_profile,
        carried_torsional_load_profile_n_m=current_torsional_load_profile_n_m,
    )
    reduced_torque_accumulation_profile = list(
        carried_torsional_result.reduced_torque_accumulation_profile
    )
    torsional_state_profile = list(carried_torsional_result.torsional_state_profile)
    torsional_feedback_status = (
        "phase14-reduced-torsional-state-fed-into-unified-local-tangential-state"
        if any(
            point.body_feedback_applied or point.centralizer_feedback_applied
            for point in torque_drag_result.local_tangential_interaction_state
        )
        else carried_torsional_result.status
    )

    return CouplingResultModel(
        status="converged" if converged else "max_iterations_reached",
        iteration_count=iteration_count,
        maximum_profile_update_n=maximum_profile_update_n,
        maximum_torque_update_n_m=maximum_torque_update_n_m,
        maximum_torsional_load_update_n_m=maximum_torsional_load_update_n_m,
        converged=converged,
        torque_feedback_mode=(
            "reduced-unified-local-tangential-state-fed-by-carried-torsional-state-plus-centralizer-axial-tangential-budget-and-convergence"
        ),
        torsional_feedback_status=torsional_feedback_status,
        converged_axial_profile=_axial_points(measured_depths_m, current_axial_profile_n),
        converged_normal_reaction_profile=_normal_reaction_points(mechanical_profile),
        converged_torque_profile=list(torque_drag_result.torque_profile),
        reduced_torque_accumulation_profile=reduced_torque_accumulation_profile,
        torsional_state_profile=torsional_state_profile,
        mechanical_summary=mechanical_summary,
        mechanical_profile=mechanical_profile,
        torque_drag_result=torque_drag_result,
        placements=placements,
    )
