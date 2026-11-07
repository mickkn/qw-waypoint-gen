"""
:File:      nav/paths.py

:Details:   Builds the Frogbot path graph: classifies marker-to-marker moves
            with the movement simulator, wires teleporters, prunes isolated
            custom markers, and fills the 8 P-slots per marker with D flags.

:Date:      2026-11-07
:Author:    Mick K
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from qw_waypoint_gen.geom.vector import Vec3, vec3
from qw_waypoint_gen.models import Entity
from qw_waypoint_gen.nav.markers import Marker
from qw_waypoint_gen.parsers.bsp_parser import CONTENTS_LAVA, CONTENTS_SLIME
from qw_waypoint_gen.sim.movement import STEP_HEIGHT, MovementSimulator, MoveResult

# Frogbot D path flags (defs.qc), only the ones this generator emits.
D_JUMP_LEDGE = 1024

MAX_PATHS_PER_MARKER = 8

_DEFAULT_LINK_DISTANCE = 400.0
_NEIGHBOR_LIMIT = 14
_SECTOR_COUNT = 8


@dataclass
class NavEdge:
    """Directed traversal candidate between two markers.

    Attributes:
        src: Source marker.
        dst: Destination marker.
        result: Movement classification.
        distance: Straight-line distance between nav positions.
    """

    src: Marker
    dst: Marker
    result: MoveResult
    distance: float


def build_paths(
    markers: list[Marker],
    sim: MovementSimulator,
    link_distance: float = _DEFAULT_LINK_DISTANCE,
) -> list[Marker]:
    """Build the full path graph and fill each marker's P/D slots.

    Steps: classify neighbor moves, wire teleporters, drop custom markers
    that ended up isolated, renumber, then select up to 8 outgoing paths per
    marker.

    Args:
        markers: Entity markers followed by custom markers, index order.
        sim: Movement simulator.
        link_distance: Maximum straight-line distance for ground links.

    Returns:
        list[Marker]: Final marker list (isolated customs removed, indices
        contiguous) with paths filled in.
    """
    navigable = [m for m in markers if m.nav_pos is not None and not _in_hazard(m)]
    edges = _classify_edges(navigable, sim, link_distance)
    forced_edges = _teleport_edges(markers, sim) + _push_edges(navigable, sim)
    edges.extend(forced_edges)

    markers = _prune_isolated_customs(markers, edges)
    kept_ids = {id(m) for m in markers}
    edges = [e for e in edges if id(e.src) in kept_ids and id(e.dst) in kept_ids]
    _fill_path_slots(edges, forced_edges)
    return markers


def _in_hazard(marker: Marker) -> bool:
    """Return True for markers standing in slime or lava."""
    return marker.contents in (CONTENTS_SLIME, CONTENTS_LAVA)


def _classify_edges(navigable: list[Marker], sim: MovementSimulator, link_distance: float) -> list[NavEdge]:
    """Classify moves between each marker and its nearest neighbors.

    Teleport trigger markers get no ground exits (entering one teleports you)
    and teleport destinations get no ground entrances (you appear there).

    Args:
        navigable: Markers with a valid nav position.
        sim: Movement simulator.
        link_distance: Maximum link distance.

    Returns:
        list[NavEdge]: Traversable directed edges.
    """
    edges: list[NavEdge] = []
    for src in navigable:
        if src.classname in ("trigger_teleport", "trigger_push"):
            continue
        neighbors = _nearest_neighbors(src, navigable, link_distance)
        for dst, distance in neighbors:
            result = sim.classify_move(src.nav_pos, dst.nav_pos)
            if MoveResult.BLOCKED == result:
                continue
            edges.append(NavEdge(src=src, dst=dst, result=result, distance=distance))
    return edges


def _nearest_neighbors(src: Marker, navigable: list[Marker], link_distance: float) -> list[tuple[Marker, float]]:
    """Return the closest candidate destinations for one source marker."""
    candidates: list[tuple[Marker, float]] = []
    for dst in navigable:
        if dst is src:
            continue
        distance = src.nav_pos.distance_to(dst.nav_pos)
        if distance <= link_distance:
            candidates.append((dst, distance))
    candidates.sort(key=lambda pair: pair[1])
    return candidates[:_NEIGHBOR_LIMIT]


def _teleport_edges(markers: list[Marker], sim: MovementSimulator) -> list[NavEdge]:
    """Create mandatory one-way edges from teleport triggers to destinations.

    Args:
        markers: All markers.
        sim: Movement simulator (unused, kept for interface symmetry).

    Returns:
        list[NavEdge]: One edge per resolvable teleporter.
    """
    destinations_by_targetname: dict[str, Marker] = {}
    for marker in markers:
        if "info_teleport_destination" == marker.classname and marker.entity is not None:
            targetname = marker.entity.properties.get("targetname", "")
            if targetname and targetname not in destinations_by_targetname:
                destinations_by_targetname[targetname] = marker

    edges: list[NavEdge] = []
    for marker in markers:
        if "trigger_teleport" != marker.classname or marker.entity is None:
            continue
        target = marker.entity.properties.get("target", "")
        destination = destinations_by_targetname.get(target)
        if destination is None or destination.nav_pos is None:
            continue
        if marker.nav_pos is None:
            continue
        distance = marker.nav_pos.distance_to(destination.nav_pos)
        edges.append(NavEdge(src=marker, dst=destination, result=MoveResult.WALK, distance=distance))
    return edges


_PUSH_SPEED_SCALE = 10.0
_PUSH_DEFAULT_SPEED = 1000.0
_VERTICAL_PUSH_RADIUS = 256.0
_LANDING_MATCH_RADIUS = 192.0


def _push_edges(navigable: list[Marker], sim: MovementSimulator) -> list[NavEdge]:
    """Create one-way exit edges for trigger_push jump pads.

    The pad's push velocity (movedir * speed * 10) is flown ballistically.
    Vertical pads connect to the highest marker near the flight column (the
    ledge the pad is meant to reach, allowing for air control); angled pads
    connect to the marker nearest the simulated landing point.

    Args:
        navigable: Markers with nav positions.
        sim: Movement simulator for ballistic flight.

    Returns:
        list[NavEdge]: One edge per resolvable jump pad.
    """
    edges: list[NavEdge] = []
    for marker in navigable:
        if "trigger_push" != marker.classname or marker.entity is None:
            continue
        velocity = _push_velocity(marker.entity)
        if velocity is None:
            continue
        start = vec3(marker.info_pos.x, marker.info_pos.y, marker.info_pos.z)
        if velocity.z > 0.0 and abs(velocity.x) < 1.0 and abs(velocity.y) < 1.0:
            target = _best_vertical_push_target(marker, start, velocity.z, navigable)
        else:
            landing = sim.simulate_launch(start, velocity)
            target = _nearest_marker(landing, navigable, marker) if landing is not None else None
        if target is None:
            continue
        edges.append(
            NavEdge(
                src=marker,
                dst=target,
                result=MoveResult.WALK,
                distance=marker.nav_pos.distance_to(target.nav_pos),
            )
        )
    return edges


def _push_velocity(entity: Entity) -> Vec3 | None:
    """Compute the trigger_push launch velocity from its angle and speed."""
    try:
        speed = float(entity.properties.get("speed", _PUSH_DEFAULT_SPEED))
    except ValueError:
        speed = _PUSH_DEFAULT_SPEED
    try:
        angle = float(entity.properties.get("angle", "0"))
    except ValueError:
        angle = 0.0
    magnitude = speed * _PUSH_SPEED_SCALE
    if -1.0 == angle:
        return vec3(0.0, 0.0, magnitude)
    if -2.0 == angle:
        return vec3(0.0, 0.0, -magnitude)
    radians = math.radians(angle)
    return vec3(math.cos(radians) * magnitude, math.sin(radians) * magnitude, 0.0)


def _best_vertical_push_target(pad: Marker, start: Vec3, vz: float, navigable: list[Marker]) -> Marker | None:
    """Pick the ledge marker a vertical jump pad is meant to reach."""
    apex = start.z + vz * vz / (2.0 * 800.0)
    best: Marker | None = None
    best_key = (-1e9, -1e9)
    for candidate in navigable:
        if candidate is pad or candidate.nav_pos is None:
            continue
        if "trigger_push" == candidate.classname:
            continue
        dx = candidate.nav_pos.x - start.x
        dy = candidate.nav_pos.y - start.y
        horizontal = (dx * dx + dy * dy) ** 0.5
        if horizontal > _VERTICAL_PUSH_RADIUS:
            continue
        z = candidate.nav_pos.z
        if z <= start.z + 45.0 or z > apex + 45.0:
            continue
        key = (z, -horizontal)
        if key > best_key:
            best_key = key
            best = candidate
    return best


def _nearest_marker(position: Vec3, navigable: list[Marker], exclude: Marker) -> Marker | None:
    """Return the marker closest to a position within the landing radius."""
    best: Marker | None = None
    best_distance = _LANDING_MATCH_RADIUS
    for candidate in navigable:
        if candidate is exclude or candidate.nav_pos is None:
            continue
        if candidate.classname in ("trigger_teleport", "trigger_push"):
            continue
        distance = candidate.nav_pos.distance_to(position)
        if distance < best_distance:
            best_distance = distance
            best = candidate
    return best


def _prune_isolated_customs(markers: list[Marker], edges: list[NavEdge]) -> list[Marker]:
    """Remove custom markers with no edges and renumber the survivors.

    Entity markers always keep their index (numbering is fixed by spawn
    order); custom markers are renumbered contiguously after them.

    Args:
        markers: Full marker list.
        edges: Candidate edge list.

    Returns:
        list[Marker]: Marker list without isolated custom markers.
    """
    connected: set[int] = set()
    for edge in edges:
        connected.add(id(edge.src))
        connected.add(id(edge.dst))

    kept = [m for m in markers if not m.is_custom or id(m) in connected]
    next_custom = sum(1 for m in kept if not m.is_custom) + 1
    for marker in kept:
        if marker.is_custom:
            marker.index = next_custom
            next_custom += 1
    return kept


def _fill_path_slots(edges: list[NavEdge], forced_edges: list[NavEdge]) -> None:
    """Select up to 8 outgoing paths per marker and set D flags.

    Forced edges (teleporters, jump pads) are always kept first. Remaining
    slots are filled by preference (walk before jump, near before far) with
    directional diversity so all 8 slots do not point the same way.

    Args:
        edges: All candidate edges (including forced edges).
        forced_edges: The forced teleporter/jump-pad edges.
    """
    teleport_ids = {(id(e.src), id(e.dst)) for e in forced_edges}
    outgoing: dict[int, list[NavEdge]] = {}
    marker_by_id: dict[int, Marker] = {}
    for edge in edges:
        outgoing.setdefault(id(edge.src), []).append(edge)
        marker_by_id[id(edge.src)] = edge.src

    for src_id, candidates in outgoing.items():
        src = marker_by_id[src_id]
        src.paths = []
        src.path_flags = {}

        forced = [e for e in candidates if (id(e.src), id(e.dst)) in teleport_ids]
        optional = [e for e in candidates if (id(e.src), id(e.dst)) not in teleport_ids]
        optional.sort(key=_edge_rank)

        chosen: list[NavEdge] = list(forced[:MAX_PATHS_PER_MARKER])
        used_sectors: set[int] = set()
        deferred: list[NavEdge] = []
        seen_dst: set[int] = {id(e.dst) for e in chosen}

        for edge in optional:
            if len(chosen) >= MAX_PATHS_PER_MARKER:
                break
            if id(edge.dst) in seen_dst:
                continue
            sector = _direction_sector(edge)
            if sector in used_sectors:
                deferred.append(edge)
                continue
            used_sectors.add(sector)
            seen_dst.add(id(edge.dst))
            chosen.append(edge)

        for edge in deferred:
            if len(chosen) >= MAX_PATHS_PER_MARKER:
                break
            if id(edge.dst) in seen_dst:
                continue
            seen_dst.add(id(edge.dst))
            chosen.append(edge)

        for slot, edge in enumerate(chosen):
            src.paths.append(edge.dst.index)
            flags = _path_flags(edge)
            if flags:
                src.path_flags[slot] = flags


def _edge_rank(edge: NavEdge) -> tuple[int, float]:
    """Sort key: walking beats falling beats jumping, then distance."""
    priority = {
        MoveResult.WALK: 0,
        MoveResult.SWIM: 0,
        MoveResult.WALK_WITH_FALL: 1,
        MoveResult.JUMP: 2,
    }[edge.result]
    return priority, edge.distance


def _direction_sector(edge: NavEdge) -> int:
    """Bucket an edge's horizontal direction into one of 8 sectors."""
    dx = edge.dst.nav_pos.x - edge.src.nav_pos.x
    dy = edge.dst.nav_pos.y - edge.src.nav_pos.y
    angle = math.atan2(dy, dx)
    sector = int(((angle + math.pi) / (2.0 * math.pi)) * _SECTOR_COUNT)
    return min(sector, _SECTOR_COUNT - 1)


def _path_flags(edge: NavEdge) -> int:
    """Return the D flag value for an edge (0 when no flag is needed).

    Upward jumps get JUMP_LEDGE so the bot commits to the jump. Ordinary
    walking, swimming, and small drops need no flags; the bot handles gap
    jumps automatically.
    """
    if MoveResult.JUMP == edge.result:
        dz = edge.dst.nav_pos.z - edge.src.nav_pos.z
        if dz > STEP_HEIGHT:
            return D_JUMP_LEDGE
    return 0
