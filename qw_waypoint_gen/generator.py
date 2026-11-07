"""
:File:      generator.py

:Details:   Top-level waypoint generation pipeline: BSP -> markers -> paths ->
            zones -> goals, ready for the KTX .bot or Frogbot QC exporter.

:Date:      2026-11-07
:Author:    Mick K
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from qw_waypoint_gen.nav.goals import assign_goals
from qw_waypoint_gen.nav.markers import KTX_MAX_MARKERS
from qw_waypoint_gen.nav.markers import Marker
from qw_waypoint_gen.nav.markers import build_custom_markers
from qw_waypoint_gen.nav.markers import build_entity_markers
from qw_waypoint_gen.nav.paths import build_paths
from qw_waypoint_gen.nav.zones import assign_zones
from qw_waypoint_gen.parsers.bsp_parser import QuakeBsp
from qw_waypoint_gen.sim.hull_trace import HULL_PLAYER, HULL_POINT, HullTracer
from qw_waypoint_gen.sim.movement import SAFE_FALL_HEIGHT, MovementSimulator


@dataclass(frozen=True)
class GenerationSettings:
    """Tunable knobs for waypoint generation.

    Attributes:
        grid: Floor sampling grid spacing in map units.
        link_distance: Maximum straight-line path link distance.
        max_fall_height: Falls beyond this are not linked.
        max_markers: Total marker cap (KTX 300, Frogbot QuakeC 350).
    """

    grid: float = 128.0
    link_distance: float = 400.0
    max_fall_height: float = SAFE_FALL_HEIGHT
    max_markers: int = KTX_MAX_MARKERS


@dataclass
class GenerationResult:
    """Output of one generation run.

    Attributes:
        map_name: Lowercase BSP stem.
        markers: Final markers in index order.
        zone_count: Number of zones created.
        goal_count: Number of goal slots used.
    """

    map_name: str
    markers: list[Marker]
    zone_count: int
    goal_count: int


def generate_waypoints(bsp_path: str | Path, settings: GenerationSettings | None = None) -> GenerationResult:
    """Generate a full bot waypoint graph from a compiled BSP.

    Args:
        bsp_path: Path to the Quake BSP29 file.
        settings: Optional generation settings; defaults are sensible for
            standard deathmatch maps.

    Returns:
        GenerationResult: Markers with paths, zones, and goals assigned.
    """
    if settings is None:
        settings = GenerationSettings()

    bsp_path = Path(bsp_path)
    bsp = QuakeBsp.load(bsp_path)
    sim = MovementSimulator(
        player_tracer=HullTracer.for_world_hull(bsp, HULL_PLAYER),
        point_tracer=HullTracer.for_world_hull(bsp, HULL_POINT),
        max_fall_height=settings.max_fall_height,
    )

    entity_markers = build_entity_markers(bsp, sim)
    custom_markers = build_custom_markers(
        bsp, sim, entity_markers, grid=settings.grid, max_total=settings.max_markers
    )
    markers = build_paths(entity_markers + custom_markers, sim, link_distance=settings.link_distance)
    zone_count = assign_zones(markers)
    goal_count = assign_goals(markers)
    return GenerationResult(
        map_name=bsp_path.stem.lower(),
        markers=markers,
        zone_count=zone_count,
        goal_count=goal_count,
    )
