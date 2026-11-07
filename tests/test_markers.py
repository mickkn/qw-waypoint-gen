"""
:File:      tests/test_markers.py
:Details:   Unit tests for Frogbot entity marker numbering (AddToQue order),
            deathmatch spawnflag filtering, and zone/goal assignment.

:Date:      2026-11-07
:Author:    Mick K
"""

from __future__ import annotations

from qw_waypoint_gen.geom.vector import vec3
from qw_waypoint_gen.models import Entity
from qw_waypoint_gen.nav.goals import assign_goals
from qw_waypoint_gen.nav.markers import Marker, build_entity_markers, is_deathmatch_entity
from qw_waypoint_gen.nav.zones import MAX_MARKERS_PER_ZONE, assign_zones


class _FakeBsp:
    """Minimal QuakeBsp stand-in exposing entities() and models()."""

    def __init__(self, entities: list[Entity]) -> None:
        self._entities = entities

    def entities(self) -> list[Entity]:
        return self._entities

    def models(self) -> list:
        return []


class _NoGroundSim:
    """Movement simulator stub whose ground snapping always fails."""

    def snap_to_ground(self, position, max_drop: float = 2048.0):
        return None


def test_is_deathmatch_entity() -> None:
    assert is_deathmatch_entity(Entity(classname="item_health", properties={}))
    assert is_deathmatch_entity(Entity(classname="item_health", properties={"spawnflags": "1"}))
    assert not is_deathmatch_entity(Entity(classname="item_health", properties={"spawnflags": "2048"}))
    assert not is_deathmatch_entity(Entity(classname="item_health", properties={"spawnflags": "2049"}))


def test_entity_marker_numbering_matches_addtoque_order() -> None:
    entities = [
        Entity(classname="worldspawn"),
        Entity(classname="info_player_start", origin=vec3(0, 0, 0)),
        Entity(classname="item_health", origin=vec3(10, 0, 0)),
        Entity(classname="light"),
        Entity(classname="weapon_rocketlauncher", origin=vec3(20, 0, 0)),
        Entity(
            classname="item_cells",
            origin=vec3(30, 0, 0),
            properties={"spawnflags": "2048"},
        ),
        Entity(classname="info_player_deathmatch", origin=vec3(40, 0, 0)),
        Entity(classname="func_door", properties={"model": "*1"}),
    ]
    markers = build_entity_markers(_FakeBsp(entities), _NoGroundSim())

    assert 4 == len(markers)
    assert ["item_health", "weapon_rocketlauncher", "info_player_deathmatch", "door"] == [m.classname for m in markers]
    assert [1, 2, 3, 4] == [m.index for m in markers]


def _chain_markers(count: int) -> list[Marker]:
    markers = [
        Marker(
            index=i,
            classname="marker",
            info_pos=vec3(float(i * 64), 0.0, 24.0),
            nav_pos=vec3(float(i * 64), 0.0, 48.0),
            is_custom=True,
        )
        for i in range(1, count + 1)
    ]
    for i, marker in enumerate(markers):
        if i > 0:
            marker.paths.append(markers[i - 1].index)
        if i + 1 < len(markers):
            marker.paths.append(markers[i + 1].index)
    return markers


def test_zone_sizes_within_limit() -> None:
    markers = _chain_markers(90)
    zone_count = assign_zones(markers)
    assert zone_count >= 3
    sizes: dict[int, int] = {}
    for marker in markers:
        assert marker.zone > 0
        sizes[marker.zone] = sizes.get(marker.zone, 0) + 1
    assert max(sizes.values()) <= MAX_MARKERS_PER_ZONE


def test_goal_priority_ordering() -> None:
    quad = Marker(
        index=1,
        classname="item_artifact_super_damage",
        info_pos=vec3(0, 0, 0),
        nav_pos=vec3(0, 0, 24),
        entity=Entity(classname="item_artifact_super_damage"),
        zone=1,
    )
    rl = Marker(
        index=2,
        classname="weapon_rocketlauncher",
        info_pos=vec3(100, 0, 0),
        nav_pos=vec3(100, 0, 24),
        entity=Entity(classname="weapon_rocketlauncher"),
        zone=2,
    )
    mega = Marker(
        index=3,
        classname="item_health",
        info_pos=vec3(200, 0, 0),
        nav_pos=vec3(200, 0, 24),
        entity=Entity(classname="item_health", properties={"spawnflags": "2"}),
        zone=1,
    )
    small_health = Marker(
        index=4,
        classname="item_health",
        info_pos=vec3(300, 0, 0),
        nav_pos=vec3(300, 0, 24),
        entity=Entity(classname="item_health"),
        zone=2,
    )
    used = assign_goals([small_health, mega, rl, quad])

    assert 3 == used
    assert 1 == quad.goal
    assert 2 == mega.goal
    assert 3 == rl.goal
    assert 0 == small_health.goal
