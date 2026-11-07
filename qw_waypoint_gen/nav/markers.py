"""
:File: markers.py
:Details: Frogbot marker placement. Assigns entity marker indices exactly like
          the Frogbot QuakeC spawn functions (AddToQue order), then generates
          ground-validated custom markers from BSP floor geometry.

:Date: 2026-07-06

"""

from __future__ import annotations

from dataclasses import dataclass, field

from qw_waypoint_gen.geom.vector import Vec3, vec3
from qw_waypoint_gen.models import Entity
from qw_waypoint_gen.parsers.bsp_parser import CONTENTS_LAVA, CONTENTS_SLIME, QuakeBsp
from qw_waypoint_gen.sim.movement import ORIGIN_ABOVE_FLOOR, GroundInfo, MovementSimulator

# Every classname whose Frogbot spawn function calls AddToQue(), in the same
# set the waypoint_map_inject.py tool uses for index counting. Order of marker
# indices is entity lump order filtered to these classnames.
MARKERABLE_CLASSNAMES = (
    "func_button",
    "trigger_changelevel",
    "info_player_deathmatch",
    "func_door",
    "func_door_secret",
    "item_health",
    "item_armor1",
    "item_armor2",
    "item_armorInv",
    "weapon_supershotgun",
    "weapon_nailgun",
    "weapon_supernailgun",
    "weapon_grenadelauncher",
    "weapon_rocketlauncher",
    "weapon_lightning",
    "item_shells",
    "item_spikes",
    "item_rockets",
    "item_cells",
    "item_weapon",
    "item_artifact_invulnerability",
    "item_artifact_envirosuit",
    "item_artifact_invisibility",
    "item_artifact_super_damage",
    "func_plat",
    "func_train",
    "trigger_multiple",
    "trigger_once",
    "trigger_secret",
    "trigger_counter",
    "info_teleport_destination",
    "trigger_teleport",
    "trigger_setskill",
    "trigger_onlyregistered",
    "trigger_hurt",
    "trigger_push",
)

# Runtime classname renames done by Frogbot spawn functions; the MarkerInfo
# trailer records runtime classnames.
RUNTIME_CLASSNAME = {
    "func_door": "door",
    "func_plat": "door",
}

# Brush-entity classnames whose MarkerInfo position is the bbox center.
BRUSH_CENTER_CLASSNAMES = {
    "trigger_teleport",
    "door",
    "func_button",
    "trigger_multiple",
    "trigger_push",
}

# Classnames that participate in the navigation graph. The others still
# consume marker indices but are not useful path nodes for generation.
NAVIGABLE_CLASSNAMES = (
    "info_player_deathmatch",
    "info_teleport_destination",
    "trigger_teleport",
    "trigger_push",
    "item_health",
    "item_armor1",
    "item_armor2",
    "item_armorInv",
    "weapon_supershotgun",
    "weapon_nailgun",
    "weapon_supernailgun",
    "weapon_grenadelauncher",
    "weapon_rocketlauncher",
    "weapon_lightning",
    "item_shells",
    "item_spikes",
    "item_rockets",
    "item_cells",
    "item_artifact_invulnerability",
    "item_artifact_envirosuit",
    "item_artifact_invisibility",
    "item_artifact_super_damage",
)

NOT_IN_DEATHMATCH = 2048

_MIN_MARKER_SPACING = 100.0
_ENTITY_CLEARANCE = 64.0
# Spacing growth per retry when custom markers exceed the marker budget. The
# attempt cap keeps the widest spacing (~231 units) well inside the default
# 400-unit link distance so the sparser markers still connect.
_SPACING_GROWTH = 1.15
_MAX_SPACING_ATTEMPTS = 7
# Engine marker queue limits: KTX NUMBER_MARKERS (progs.h) and the Frogbot
# QuakeC m1..m350 globals.
KTX_MAX_MARKERS = 300
FROGBOT_MAX_MARKERS = 350
MAX_TOTAL_MARKERS = KTX_MAX_MARKERS


@dataclass
class Marker:
    """One Frogbot marker (entity-derived or custom).

    Attributes:
        index: 1-based Frogbot marker index (m<index>).
        classname: Runtime classname; "marker" for custom markers.
        info_pos: Position reported in the MarkerInfo trailer.
        nav_pos: Ground-snapped standing origin used for reachability, or
            None if the marker is not navigable (e.g. doors, triggers).
        entity: Source entity for entity markers.
        is_custom: True for generated (N()) markers.
        in_liquid: True if nav_pos is inside a liquid volume.
        contents: CONTENTS_* value at nav_pos (0 when unknown).
        zone: Assigned zone number (0 = unassigned).
        goal: Assigned goal number (0 = none).
        marker_type: T flag bits to emit.
        paths: Outgoing path destination marker indices (max 8).
        path_flags: D flag value per path slot.
    """

    index: int
    classname: str
    info_pos: Vec3
    nav_pos: Vec3 | None = None
    entity: Entity | None = None
    is_custom: bool = False
    in_liquid: bool = False
    contents: int = 0
    zone: int = 0
    goal: int = 0
    marker_type: int = 0
    paths: list[int] = field(default_factory=list)
    path_flags: dict[int, int] = field(default_factory=dict)


def is_deathmatch_entity(entity: Entity) -> bool:
    """Return True if the entity spawns in deathmatch (spawnflags bit 2048 unset).

    Args:
        entity: Parsed BSP entity.

    Returns:
        bool: True when the entity spawns in deathmatch mode.
    """
    raw = entity.properties.get("spawnflags", "0")
    try:
        spawnflags = int(float(raw))
    except ValueError:
        return True
    return 0 == (spawnflags & NOT_IN_DEATHMATCH)


def entity_bbox_center(bsp: QuakeBsp, entity: Entity) -> Vec3 | None:
    """Return the bounding-box center of a brush entity via its *N model.

    Args:
        bsp: Parsed BSP (for the models lump).
        entity: Entity carrying a "model" "*N" property.

    Returns:
        Vec3 | None: Center of the submodel bounds, or None if unresolvable.
    """
    model_ref = entity.properties.get("model", "")
    if not model_ref.startswith("*"):
        return None
    try:
        model_index = int(model_ref[1:])
    except ValueError:
        return None
    models = bsp.models()
    if model_index <= 0 or model_index >= len(models):
        return None
    return models[model_index].center()


def build_entity_markers(bsp: QuakeBsp, sim: MovementSimulator) -> list[Marker]:
    """Create entity markers with Frogbot-compatible index numbering.

    Iterates entities in lump order; every MARKERABLE classname that spawns in
    deathmatch consumes the next marker index, exactly as the QuakeC spawn
    functions call AddToQue().

    Args:
        bsp: Parsed BSP file.
        sim: Movement simulator for ground snapping.

    Returns:
        list[Marker]: Entity markers in index order (index starts at 1).
    """
    markers: list[Marker] = []
    index = 0
    for entity in bsp.entities():
        if entity.classname not in MARKERABLE_CLASSNAMES:
            continue
        if not is_deathmatch_entity(entity):
            continue
        index += 1
        runtime_class = RUNTIME_CLASSNAME.get(entity.classname, entity.classname)

        if runtime_class in BRUSH_CENTER_CLASSNAMES and entity.origin is None:
            info_pos = entity_bbox_center(bsp, entity)
            if info_pos is None:
                info_pos = entity.origin if entity.origin is not None else vec3(0.0, 0.0, 0.0)
        else:
            info_pos = (
                entity.origin if entity.origin is not None else (entity_bbox_center(bsp, entity) or vec3(0.0, 0.0, 0.0))
            )

        marker = Marker(index=index, classname=runtime_class, info_pos=info_pos, entity=entity)
        if entity.classname in NAVIGABLE_CLASSNAMES:
            ground = _snap_entity(sim, info_pos)
            if ground is not None:
                marker.nav_pos = ground.origin
                marker.in_liquid = ground.in_liquid
                marker.contents = ground.contents
        markers.append(marker)
    return markers


_SNAP_OFFSETS = (24.0, 40.0, 56.0)
_SNAP_DIRECTIONS = (
    (1.0, 0.0),
    (-1.0, 0.0),
    (0.0, 1.0),
    (0.0, -1.0),
    (0.7071, 0.7071),
    (-0.7071, 0.7071),
    (0.7071, -0.7071),
    (-0.7071, -0.7071),
)


def _snap_entity(sim: MovementSimulator, info_pos: Vec3):
    """Snap an entity marker position to standable ground.

    Items in wall alcoves or on shelves cannot fit the player hull at their
    own origin; bots collect them by touch radius while standing nearby. When
    the direct snap fails, nearby horizontally-offset positions are tried so
    the marker still joins the navigation graph.

    Args:
        sim: Movement simulator.
        info_pos: Entity origin (or bbox center).

    Returns:
        GroundInfo | None: Standing info, or None if no standable spot exists
        near the entity.
    """
    direct = sim.snap_to_ground(vec3(info_pos.x, info_pos.y, info_pos.z + ORIGIN_ABOVE_FLOOR))
    if direct is not None:
        return direct
    for offset in _SNAP_OFFSETS:
        for dx, dy in _SNAP_DIRECTIONS:
            probe = vec3(
                info_pos.x + dx * offset,
                info_pos.y + dy * offset,
                info_pos.z + ORIGIN_ABOVE_FLOOR,
            )
            ground = sim.snap_to_ground(probe)
            if ground is not None and abs(ground.origin.z - info_pos.z) <= 64.0:
                return ground
    return None


def build_custom_markers(
    bsp: QuakeBsp,
    sim: MovementSimulator,
    entity_markers: list[Marker],
    grid: float = 128.0,
    max_total: int = MAX_TOTAL_MARKERS,
) -> list[Marker]:
    """Generate custom floor markers from world floor faces.

    Samples upward-facing world surfaces on a grid, snaps each sample to the
    ground with the player hull, and keeps samples that are far enough from
    all existing markers. When the samples do not fit the marker budget the
    spacing is widened and placement retried, so large maps keep full
    coverage at lower density instead of losing whole regions.

    Args:
        bsp: Parsed BSP file.
        sim: Movement simulator for validation.
        entity_markers: Already-numbered entity markers.
        grid: Sampling grid spacing in map units.
        max_total: Hard cap on total marker count (engine queue limit).

    Returns:
        list[Marker]: Custom markers numbered after the entity markers.
    """
    grounds: list[GroundInfo] = []
    for sample in _floor_samples(bsp, grid):
        ground = sim.snap_to_ground(sample)
        if ground is None or ground.contents in (CONTENTS_SLIME, CONTENTS_LAVA):
            continue
        grounds.append(ground)

    entity_positions = [m.nav_pos for m in entity_markers if m.nav_pos is not None]
    budget = max(0, max_total - len(entity_markers))
    scale = 1.0
    for _ in range(_MAX_SPACING_ATTEMPTS):
        placed = _place_custom(grounds, entity_positions, scale, budget + 1)
        if len(placed) <= budget:
            break
        scale *= _SPACING_GROWTH
    placed = placed[:budget]

    first_index = len(entity_markers) + 1
    return [
        Marker(
            index=first_index + offset,
            classname="marker",
            info_pos=vec3(round(ground.origin.x), round(ground.origin.y), round(ground.origin.z)),
            nav_pos=ground.origin,
            is_custom=True,
            in_liquid=ground.in_liquid,
            contents=ground.contents,
        )
        for offset, ground in enumerate(placed)
    ]


def _place_custom(
    grounds: list[GroundInfo], entity_positions: list[Vec3], scale: float, limit: int
) -> list[GroundInfo]:
    """Greedily keep ground samples that respect the scaled marker spacing.

    Args:
        grounds: Snapped ground results, in sampling order.
        entity_positions: Navigable entity marker positions.
        scale: Multiplier applied to the base spacing distances.
        limit: Stop after keeping this many samples.

    Returns:
        list[GroundInfo]: The kept ground results.
    """
    kept_positions = list(entity_positions)
    placed: list[GroundInfo] = []
    for ground in grounds:
        if len(placed) >= limit:
            break
        spacing = _ENTITY_CLEARANCE if not placed else _MIN_MARKER_SPACING
        if _too_close(ground.origin, kept_positions, _MIN_MARKER_SPACING * scale, spacing * scale):
            continue
        placed.append(ground)
        kept_positions.append(ground.origin)
    return placed


def _floor_samples(bsp: QuakeBsp, grid: float) -> list[Vec3]:
    """Sample points slightly above upward-facing world surfaces.

    Args:
        bsp: Parsed BSP file.
        grid: Grid spacing for interior face sampling.

    Returns:
        list[Vec3]: Candidate positions at player-origin height.
    """
    samples: list[Vec3] = []
    for surface in bsp.surfaces(model_index=0):
        if surface.normal.z < 0.7:
            continue
        texture = surface.texture.lower()
        if texture.startswith(("sky", "*lava", "*slime")):
            continue
        xs = [v.x for v in surface.vertices]
        ys = [v.y for v in surface.vertices]
        z = max(v.z for v in surface.vertices)
        centroid = vec3(sum(xs) / len(xs), sum(ys) / len(ys), z + ORIGIN_ABOVE_FLOOR + 1.0)
        samples.append(centroid)

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        x = _ceil_to_grid(min_x + grid * 0.25, grid)
        while x <= max_x:
            y = _ceil_to_grid(min_y + grid * 0.25, grid)
            while y <= max_y:
                if _point_in_poly_xy(x, y, surface.vertices):
                    samples.append(vec3(x, y, z + ORIGIN_ABOVE_FLOOR + 1.0))
                y += grid
            x += grid
    return samples


def _too_close(pos: Vec3, existing: list[Vec3], custom_spacing: float, entity_spacing: float) -> bool:
    """Return True if pos violates the minimum spacing to existing markers."""
    spacing = min(custom_spacing, entity_spacing)
    spacing_sq = spacing * spacing
    for other in existing:
        dx = pos.x - other.x
        dy = pos.y - other.y
        dz = pos.z - other.z
        if dx * dx + dy * dy + dz * dz < spacing_sq:
            return True
    return False


def _ceil_to_grid(value: float, grid: float) -> float:
    """Round value up to the next multiple of grid."""
    if grid <= 0.0:
        return value
    quotient = value / grid
    rounded = float(int(quotient))
    if rounded < quotient:
        rounded += 1.0
    return rounded * grid


def _point_in_poly_xy(x: float, y: float, poly: list[Vec3]) -> bool:
    """Return True if (x, y) lies inside the polygon projected to XY."""
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i].x, poly[i].y
        xj, yj = poly[j].x, poly[j].y
        crosses = (yi > y) != (yj > y)
        if crosses:
            denom = yj - yi
            if abs(denom) < 1e-9:
                denom = 1e-9
            if x < (xj - xi) * (y - yi) / denom + xi:
                inside = not inside
        j = i
    return inside
