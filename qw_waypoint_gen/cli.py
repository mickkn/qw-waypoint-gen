"""
:File: cli.py
:Details: Command line interface for generating KTX .bot route files (or
          Frogbot map_<name>.qc files) from compiled QuakeWorld BSP29 maps.

:Date: 2026-07-06

"""

from __future__ import annotations

import enum
import json
from pathlib import Path

import typer

from qw_waypoint_gen.exporters.frogbot_qc import write_frogbot_qc
from qw_waypoint_gen.exporters.ktx_bot import write_ktx_bot
from qw_waypoint_gen.generator import GenerationResult
from qw_waypoint_gen.generator import GenerationSettings
from qw_waypoint_gen.generator import generate_waypoints
from qw_waypoint_gen.nav.markers import FROGBOT_MAX_MARKERS
from qw_waypoint_gen.nav.markers import KTX_MAX_MARKERS
from qw_waypoint_gen.nav.zones import ZoneAssignmentError
from qw_waypoint_gen.parsers.bsp_parser import BspParseError
from qw_waypoint_gen.sim.movement import SAFE_FALL_HEIGHT
from qw_waypoint_gen.validation.checks import summarize

app = typer.Typer(no_args_is_help=True)


class OutputFormat(str, enum.Enum):
    """Supported waypoint output formats."""

    KTX = "ktx"
    FROGBOT = "frogbot"


_MAX_MARKERS = {
    OutputFormat.KTX: KTX_MAX_MARKERS,
    OutputFormat.FROGBOT: FROGBOT_MAX_MARKERS,
}


def default_output_name(map_name: str, fmt: OutputFormat) -> str:
    """Return the conventional output file name for a map.

    Args:
        map_name: Lowercase BSP stem.
        fmt: Output format.

    Returns:
        str: "<map>.bot" for KTX, "map_<map>.qc" for Frogbot.
    """
    if OutputFormat.KTX == fmt:
        return f"{map_name}.bot"
    return f"map_{map_name}.qc"


def _write(out_path: Path, result: GenerationResult, fmt: OutputFormat) -> None:
    """Write the generation result in the requested format."""
    if OutputFormat.KTX == fmt:
        write_ktx_bot(out_path, result.map_name, result.markers)
    else:
        write_frogbot_qc(out_path, result.map_name, result.markers)


def _settings(grid: float, link_distance: float, max_fall: float, fmt: OutputFormat) -> GenerationSettings:
    """Build generation settings for the target format."""
    return GenerationSettings(
        grid=grid,
        link_distance=link_distance,
        max_fall_height=max_fall,
        max_markers=_MAX_MARKERS[fmt],
    )


def _generate_one(bsp_file: Path, out_path: Path | None, settings: GenerationSettings, fmt: OutputFormat) -> dict:
    """Generate and write waypoints for one BSP, returning its summary."""
    result = generate_waypoints(bsp_file, settings)
    target = out_path if out_path is not None else Path(default_output_name(result.map_name, fmt))
    _write(target, result, fmt)
    summary = summarize(result.markers, result.zone_count, result.goal_count, settings.max_markers)
    summary["output"] = str(target)
    return summary


@app.command("generate")
def generate(
    bsp_file: Path = typer.Argument(..., exists=True, readable=True, dir_okay=False),
    out: Path | None = typer.Option(None, "--out", "-o", help="Output path; defaults to <map>.bot (or map_<map>.qc)."),
    fmt: OutputFormat = typer.Option(OutputFormat.KTX, "--format", "-f", help="Output format."),
    grid: float = typer.Option(128.0, help="Floor sampling grid spacing (map units)."),
    link_distance: float = typer.Option(400.0, help="Maximum path link distance."),
    max_fall: float = typer.Option(SAFE_FALL_HEIGHT, help="Maximum fall height allowed on paths."),
) -> None:
    """Generate a KTX <map>.bot (default) or Frogbot map_<map>.qc file from a compiled BSP."""
    summary = _generate_one(bsp_file, out, _settings(grid, link_distance, max_fall, fmt), fmt)
    typer.echo(json.dumps(summary, indent=2))
    if summary["problems"]:
        raise typer.Exit(code=1)


@app.command("batch")
def batch(
    maps_dir: Path = typer.Argument(..., exists=True, file_okay=False, help="Directory containing .bsp files."),
    out_dir: Path = typer.Option(..., "--out-dir", "-o", help="Output directory, e.g. <ktx>/bots/maps."),
    fmt: OutputFormat = typer.Option(OutputFormat.KTX, "--format", "-f", help="Output format."),
    overwrite: bool = typer.Option(False, help="Regenerate files that already exist in the output directory."),
    grid: float = typer.Option(128.0, help="Floor sampling grid spacing (map units)."),
    link_distance: float = typer.Option(400.0, help="Maximum path link distance."),
    max_fall: float = typer.Option(SAFE_FALL_HEIGHT, help="Maximum fall height allowed on paths."),
) -> None:
    """Generate waypoint files for every .bsp in a directory.

    Existing files (e.g. hand-made .bot files shipped with KTX) are kept unless
    --overwrite is given.
    """
    settings = _settings(grid, link_distance, max_fall, fmt)
    report: dict[str, object] = {}
    for bsp_file in sorted(maps_dir.glob("*.bsp")):
        map_name = bsp_file.stem.lower()
        target = out_dir / default_output_name(map_name, fmt)
        if target.exists() and not overwrite:
            report[map_name] = "skipped (exists)"
            continue
        try:
            summary = _generate_one(bsp_file, target, settings, fmt)
        except (BspParseError, ZoneAssignmentError, OSError) as exc:
            report[map_name] = f"failed: {exc}"
            typer.echo(f"{map_name}: failed: {exc}", err=True)
            continue
        status = "ok" if not summary["problems"] else f"problems: {summary['problems']}"
        report[map_name] = status
        typer.echo(f"{map_name}: {summary['markers_total']} markers -> {target} ({status})", err=True)
    typer.echo(json.dumps(report, indent=2))


@app.command("version")
def version() -> None:
    """Print the tool version."""
    typer.echo("qw-waypoint-gen 1.1.0")


if __name__ == "__main__":
    app()
