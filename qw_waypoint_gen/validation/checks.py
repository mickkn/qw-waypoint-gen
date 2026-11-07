"""
:File:      validation/checks.py

:Details:   Validation of generated waypoint graphs against Frogbot engine
            limits, plus a human-readable summary for CLI output.

:Date:      2026-11-07
:Author:    Mick K
"""

from __future__ import annotations

from collections import Counter

from qw_waypoint_gen.nav.markers import MAX_TOTAL_MARKERS, Marker
from qw_waypoint_gen.nav.paths import MAX_PATHS_PER_MARKER
from qw_waypoint_gen.nav.zones import MAX_MARKERS_PER_ZONE, MAX_ZONES


def validate(markers: list[Marker], max_markers: int = MAX_TOTAL_MARKERS) -> list[str]:
    """Check hard bot engine limits and structural invariants.

    Args:
        markers: Final marker list.
        max_markers: Marker queue limit of the target engine.

    Returns:
        list[str]: Human-readable problem descriptions; empty when valid.
    """
    problems: list[str] = []
    if len(markers) > max_markers:
        problems.append(f"{len(markers)} markers exceed the engine queue limit of {max_markers}")

    indices = [m.index for m in markers]
    if sorted(indices) != list(range(1, len(markers) + 1)):
        problems.append("Marker indices are not contiguous starting at 1")

    zone_sizes = Counter(m.zone for m in markers if m.zone > 0)
    if len(zone_sizes) > MAX_ZONES:
        problems.append(f"{len(zone_sizes)} zones exceed the limit of {MAX_ZONES}")
    for zone, size in zone_sizes.items():
        if size > MAX_MARKERS_PER_ZONE:
            problems.append(f"Zone {zone} has {size} markers (limit {MAX_MARKERS_PER_ZONE})")

    problems.extend(_path_problems(markers, set(indices)))
    return problems


def _path_problems(markers: list[Marker], index_set: set[int]) -> list[str]:
    """Check path slot counts, link targets, and zoning of path endpoints.

    Args:
        markers: Final marker list.
        index_set: All valid marker indices.

    Returns:
        list[str]: Human-readable problem descriptions.
    """
    problems: list[str] = []
    zoned = {m.index for m in markers if m.zone > 0}
    for marker in markers:
        if len(marker.paths) > MAX_PATHS_PER_MARKER:
            problems.append(f"m{marker.index} has {len(marker.paths)} paths (limit {MAX_PATHS_PER_MARKER})")
        for dst in marker.paths:
            if dst not in index_set:
                problems.append(f"m{marker.index} links to missing marker m{dst}")
            elif dst not in zoned:
                # KTX route_calc indexes zones[Z_ - 1] on path targets.
                problems.append(f"m{marker.index} links to unzoned marker m{dst}")
        if marker.paths and 0 == marker.zone:
            problems.append(f"m{marker.index} is routable but has no zone")
    return problems


def summarize(
    markers: list[Marker], zone_count: int, goal_count: int, max_markers: int = MAX_TOTAL_MARKERS
) -> dict[str, object]:
    """Build a summary dictionary for CLI reporting.

    Args:
        markers: Final marker list.
        zone_count: Zones created by the generator.
        goal_count: Goal numbers assigned.
        max_markers: Marker queue limit of the target engine.

    Returns:
        dict[str, object]: Aggregate statistics about the generated graph.
    """
    customs = [m for m in markers if m.is_custom]
    entity_markers = [m for m in markers if not m.is_custom]
    routable = [m for m in markers if m.paths]
    path_count = sum(len(m.paths) for m in markers)
    unreachable_items = [
        f"m{m.index} {m.classname}"
        for m in entity_markers
        if not m.paths and m.classname.startswith(("item_", "weapon_"))
    ]
    return {
        "markers_total": len(markers),
        "markers_entity": len(entity_markers),
        "markers_custom": len(customs),
        "markers_routable": len(routable),
        "paths": path_count,
        "zones": zone_count,
        "goals": goal_count,
        "unreachable_items": unreachable_items,
        "problems": validate(markers, max_markers),
    }
