from __future__ import annotations

from pathlib import Path

import typer

from .models import ConfigError
from .runner import format_case_summary, load_case, run_stub_case

app = typer.Typer(
    add_completion=False,
    help="CLI for the staged centralization and torque & drag project baselines.",
)


@app.command("summary")
def summary(
    case_path: Path = typer.Argument(..., exists=True, dir_okay=False, readable=True, resolve_path=True)
) -> None:
    """Load YAML files, validate minimum fields, and print a case summary."""

    try:
        loaded_case = load_case(case_path)
    except ConfigError as exc:
        typer.echo(str(exc), err=True)
        raise typer.Exit(code=1) from exc

    typer.echo(format_case_summary(loaded_case))


@app.command("run-stub")
def run_stub(
    case_path: Path = typer.Argument(..., exists=True, dir_okay=False, readable=True, resolve_path=True),
    output: Path | None = typer.Option(
        None,
        "--output",
        "-o",
        dir_okay=False,
        help="Optional JSON output path. Defaults to the path defined by the case manifest.",
    ),
) -> None:
    """Execute the current baseline solver and write a JSON result file."""

    try:
        loaded_case, payload, output_path = run_stub_case(case_path, output)
    except ConfigError as exc:
        typer.echo(str(exc), err=True)
        raise typer.Exit(code=1) from exc

    typer.echo(format_case_summary(loaded_case))
    typer.echo("")
    typer.echo(f"Backend: {payload['backend']}")
    typer.echo(f"Status: {payload['status']}")
    typer.echo(f"Operation mode: {payload['operation_mode']}")
    typer.echo(f"Message: {payload['message']}")
    typer.echo(
        "Approx vertical depth [m]: "
        f"{payload['trajectory_summary']['vertical_depth_m']:.2f}"
    )
    typer.echo(
        "Approx lateral displacement [m]: "
        f"{payload['trajectory_summary']['lateral_displacement_m']:.2f}"
    )
    typer.echo(
        "Max discrete curvature [rad/m]: "
        f"{payload['trajectory_summary']['max_curvature_rad_per_m']:.6f}"
    )
    typer.echo(
        "Global solver iterations [-]: "
        f"{payload['mechanical_summary']['global_solver_iteration_count']}"
    )
    typer.echo(
        "Global solver final update norm [m]: "
        f"{payload['mechanical_summary']['global_solver_final_update_norm_m']:.6e}"
    )
    typer.echo(f"Coupling status: {payload['coupling_status']}")
    typer.echo(f"Coupling iterations [-]: {payload['coupling_iterations']}")
    typer.echo(f"Coupling converged: {payload['coupling_converged']}")
    typer.echo(
        "Top effective axial load [N]: "
        f"{payload['mechanical_summary']['top_effective_axial_load_n']:.2f}"
    )
    typer.echo(f"Hookload run in [N]: {payload['hookload_run_in_n']:.2f}")
    typer.echo(f"Hookload pull out [N]: {payload['hookload_pull_out_n']:.2f}")
    typer.echo(f"Drag run in [N]: {payload['drag_run_in_n']:.2f}")
    typer.echo(f"Drag pull out [N]: {payload['drag_pull_out_n']:.2f}")
    typer.echo(
        "Max bending moment [N.m]: "
        f"{payload['mechanical_summary']['maximum_bending_moment_n_m']:.2f}"
    )
    typer.echo(
        "Max bending stress [Pa]: "
        f"{payload['mechanical_summary']['maximum_bending_stress_pa']:.2f}"
    )
    typer.echo(
        "Max bending strain estimate [-]: "
        f"{payload['mechanical_summary']['maximum_bending_strain_estimate']:.6f}"
    )
    typer.echo(
        "Max lateral load [N/m]: "
        f"{payload['mechanical_summary']['maximum_equivalent_lateral_load_n_per_m']:.2f}"
    )
    typer.echo(
        "Max normal reaction estimate [N]: "
        f"{payload['mechanical_summary']['maximum_normal_reaction_estimate_n']:.2f}"
    )
    typer.echo(
        "Max eccentricity estimate [m]: "
        f"{payload['mechanical_summary']['maximum_eccentricity_estimate_m']:.6f}"
    )
    typer.echo(
        "Minimum standoff estimate [-]: "
        f"{payload['minimum_standoff_estimate']:.3f}"
    )
    typer.echo(
        "Minimum nominal radial clearance [m]: "
        f"{payload['minimum_nominal_radial_clearance_m']:.4f}"
    )
    typer.echo(f"Centralizer model status: {payload['centralizer_model_status']}")
    typer.echo(f"Torque drag status: {payload['torque_drag_status']}")
    if payload["estimated_surface_torque_n_m"] is None:
        typer.echo(f"Surface torque [N.m]: unavailable ({payload['torque_drag_status']})")
    else:
        typer.echo(f"Surface torque [N.m]: {payload['estimated_surface_torque_n_m']:.2f}")
    if payload["updated_estimated_surface_torque_n_m"] is not None:
        typer.echo(
            "Updated surface torque with detailed centralizers [N.m]: "
            f"{payload['updated_estimated_surface_torque_n_m']:.2f}"
        )
    typer.echo(
        "Contact segments: "
        f"{payload['mechanical_summary']['contact_segment_count']}"
    )
    typer.echo(
        "Pipe-body contact segments: "
        f"{payload['mechanical_summary']['pipe_body_contact_segment_count']}"
    )
    typer.echo(f"Saved JSON: {output_path}")


def main() -> None:
    app()


if __name__ == "__main__":
    main()
