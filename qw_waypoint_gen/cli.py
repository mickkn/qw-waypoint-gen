"""
:File:      cli.py

:Details:   Command line interface for generating KTX .bot route files (or
            Frogbot map_<name>.qc files) from compiled QuakeWorld BSP29 maps.

:Date:      2026-11-07
:Author:    Mick K
"""

from __future__ import annotations

import enum
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
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


def _generate_one(
    bsp_file: Path, out_path: Path | None, settings: GenerationSettings, fmt: OutputFormat
) -> dict[str, object]:
    """Generate and write waypoints for one BSP, returning its summary."""
    result = generate_waypoints(bsp_file, settings)
    target = out_path if out_path is not None else Path(default_output_name(result.map_name, fmt))
    _write(target, result, fmt)
    summary = summarize(result.markers, result.zone_count, result.goal_count, settings.max_markers)
    summary["output"] = str(target)
    return summary


def _batch_generate_one(
    bsp_file: Path, target: Path, settings: GenerationSettings, fmt: OutputFormat
) -> dict[str, object] | str:
    """Return a map summary or an expected, reportable generation failure."""
    try:
        return _generate_one(bsp_file, target, settings, fmt)
    except (BspParseError, ZoneAssignmentError, OSError) as exc:
        return f"failed: {exc}"


def _report_batch_result(map_name: str, target: Path, result: dict[str, object] | str) -> str:
    """Print one completed map's status from the main process."""
    if isinstance(result, str):
        typer.echo(f"{map_name}: {result}", err=True)
        return result
    status = "ok" if not result["problems"] else f"problems: {result['problems']}"
    typer.echo(f"{map_name}: {result['markers_total']} markers -> {target} ({status})", err=True)
    return status


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
    workers: int = typer.Option(0, min=0, help="Worker processes; 0 uses CPU count minus one, 1 runs sequentially."),
) -> None:
    """Generate waypoint files for every .bsp in a directory.

    Existing files (e.g. hand-made .bot files shipped with KTX) are kept unless
    --overwrite is given.
    """
    settings = _settings(grid, link_distance, max_fall, fmt)
    report: dict[str, object] = {}
    jobs: list[tuple[Path, Path]] = []
    for bsp_file in sorted(maps_dir.glob("*.bsp")):
        map_name = bsp_file.stem.lower()
        target = out_dir / default_output_name(map_name, fmt)
        if map_name in report:
            raise typer.BadParameter(f"Multiple BSP files share the output name {target.name}", param_hint="maps_dir")
        if target.exists() and not overwrite:
            report[map_name] = "skipped (exists)"
            continue
        report[map_name] = "pending"
        jobs.append((bsp_file, target))

    worker_count = workers or max(1, (os.cpu_count() or 1) - 1)
    if sys.platform == "win32":
        worker_count = min(worker_count, 61)
    worker_count = min(worker_count, len(jobs))
    if worker_count == 1:
        for bsp_file, target in jobs:
            map_name = bsp_file.stem.lower()
            result = _batch_generate_one(bsp_file, target, settings, fmt)
            report[map_name] = _report_batch_result(map_name, target, result)
    elif worker_count > 1:
        with ProcessPoolExecutor(max_workers=worker_count) as executor:
            futures = {
                executor.submit(_batch_generate_one, bsp_file, target, settings, fmt): (bsp_file.stem.lower(), target)
                for bsp_file, target in jobs
            }
            for future in as_completed(futures):
                map_name, target = futures[future]
                report[map_name] = _report_batch_result(map_name, target, future.result())
    typer.echo(json.dumps(report, indent=2, sort_keys=True))


@app.command("version")
def version() -> None:
    """Print the tool version."""
    typer.echo("qw-waypoint-gen 1.1.0")


if __name__ == "__main__":
    app()
