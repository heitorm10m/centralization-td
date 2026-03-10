from __future__ import annotations

from pathlib import Path

import typer

from .models import ConfigError
from .runner import format_case_summary, load_case, run_stub_case

app = typer.Typer(
    add_completion=False,
    help="CLI for Phase 1 centralization and torque & drag scaffolding.",
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
    """Execute the deterministic solver stub and write a JSON result file."""

    try:
        loaded_case, payload, output_path = run_stub_case(case_path, output)
    except ConfigError as exc:
        typer.echo(str(exc), err=True)
        raise typer.Exit(code=1) from exc

    typer.echo(format_case_summary(loaded_case))
    typer.echo("")
    typer.echo(f"Backend: {payload['backend']}")
    typer.echo(f"Status: {payload['status']}")
    typer.echo(f"Hookload placeholder [N]: {payload['estimated_hookload_n']:.2f}")
    typer.echo(f"Surface torque placeholder [N.m]: {payload['estimated_surface_torque_n_m']:.2f}")
    typer.echo(f"Minimum standoff placeholder [-]: {payload['minimum_standoff_ratio']:.3f}")
    typer.echo(f"Contact nodes placeholder: {payload['contact_nodes']}")
    typer.echo(f"Saved JSON: {output_path}")


def main() -> None:
    app()


if __name__ == "__main__":
    main()
