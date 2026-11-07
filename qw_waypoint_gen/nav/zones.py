"""
:File:      nav/zones.py

:Details:   Automatic Frogbot zone assignment. Grows connected regions over
            the path graph honoring the engine limits (24 zones, 32 markers
            per zone) so every routable marker gets a zone.

:Date:      2026-11-07
:Author:    Mick K
"""

from __future__ import annotations

from collections import deque

from qw_waypoint_gen.nav.markers import Marker

MAX_ZONES = 24
MAX_MARKERS_PER_ZONE = 32


class ZoneAssignmentError(RuntimeError):
    """Raised when markers cannot fit within the Frogbot zone limits."""


def assign_zones(markers: list[Marker]) -> int:
    """Assign zone numbers to all path-connected markers.

    Regions are grown breadth-first over the undirected path graph, capped at
    a balanced size so the total stays within 24 zones. Markers without any
    paths keep zone 0 (unroutable but touchable, matching Frogbot behavior).

    Args:
        markers: Final marker list with paths filled in.

    Returns:
        int: Number of zones created.

    Raises:
        ZoneAssignmentError: If the routable marker count exceeds the
            24 * 32 engine capacity.
    """
    by_index = {m.index: m for m in markers}
    adjacency: dict[int, set[int]] = {m.index: set() for m in markers}
    for marker in markers:
        for dst_index in marker.paths:
            if dst_index in adjacency:
                adjacency[marker.index].add(dst_index)
                adjacency[dst_index].add(marker.index)

    routable = [m for m in markers if adjacency[m.index]]
    if len(routable) > MAX_ZONES * MAX_MARKERS_PER_ZONE:
        raise ZoneAssignmentError(
            f"{len(routable)} routable markers exceed the " f"{MAX_ZONES * MAX_MARKERS_PER_ZONE} marker zone capacity"
        )

    min_zones_needed = (len(routable) + MAX_MARKERS_PER_ZONE - 1) // MAX_MARKERS_PER_ZONE
    target_size = min(
        MAX_MARKERS_PER_ZONE,
        max(8, (len(routable) + min_zones_needed - 1) // max(1, min_zones_needed)),
    )

    zone = 0
    unassigned = {m.index for m in routable}
    while unassigned:
        zone += 1
        if zone > MAX_ZONES:
            zone = _rebalance_overflow(by_index, unassigned)
            break
        seed = _pick_seed(unassigned, by_index, adjacency)
        size = 0
        queue: deque[int] = deque([seed])
        while queue and size < target_size:
            current = queue.popleft()
            if current not in unassigned:
                continue
            unassigned.discard(current)
            by_index[current].zone = zone
            size += 1
            neighbors = sorted(
                (n for n in adjacency[current] if n in unassigned),
                key=lambda n: _distance(by_index[current], by_index[n]),
            )
            queue.extend(neighbors)
    return zone


def _pick_seed(
    unassigned: set[int],
    by_index: dict[int, Marker],
    adjacency: dict[int, set[int]],
) -> int:
    """Pick the next region seed: prefer frontier markers next to assigned zones."""
    frontier = [idx for idx in unassigned if any(n not in unassigned for n in adjacency[idx])]
    pool = frontier if frontier else list(unassigned)
    return min(pool)


def _rebalance_overflow(by_index: dict[int, Marker], unassigned: set[int]) -> int:
    """Assign leftover markers to the nearest existing zone (limit reached).

    Args:
        by_index: Marker lookup by index.
        unassigned: Marker indices that did not fit in 24 fresh zones.

    Returns:
        int: The zone count (MAX_ZONES).
    """
    zoned = [m for m in by_index.values() if m.zone > 0]
    counts: dict[int, int] = {}
    for marker in zoned:
        counts[marker.zone] = counts.get(marker.zone, 0) + 1

    for idx in sorted(unassigned):
        marker = by_index[idx]
        best_zone = 0
        best_distance = float("inf")
        for other in zoned:
            if counts.get(other.zone, 0) >= MAX_MARKERS_PER_ZONE:
                continue
            distance = _distance(marker, other)
            if distance < best_distance:
                best_distance = distance
                best_zone = other.zone
        if best_zone > 0:
            marker.zone = best_zone
            counts[best_zone] = counts.get(best_zone, 0) + 1
    return MAX_ZONES


def _distance(a: Marker, b: Marker) -> float:
    """Distance between two markers' navigation (or info) positions."""
    pa = a.nav_pos if a.nav_pos is not None else a.info_pos
    pb = b.nav_pos if b.nav_pos is not None else b.info_pos
    return pa.distance_to(pb)
