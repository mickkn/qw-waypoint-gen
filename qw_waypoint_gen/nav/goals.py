"""
:File:      nav/goals.py

:Details:   Automatic Frogbot goal numbering. Ranks important items (powerups,
            armor, big weapons, megahealth) and assigns low goal numbers to the
            most desirable ones, leaving the built-in defaults (G19-G24) to the
            mod.

:Date:      2026-11-07
:Author:    Mick K
"""

from __future__ import annotations

from qw_waypoint_gen.nav.markers import Marker

# Item desirability ranking, most desirable first. Classnames not listed get
# no explicit goal: the mod auto-assigns G19-G24 defaults for basic ammo and
# weak weapons, and everything else is grabbed opportunistically.
GOAL_RANKING = (
    "item_artifact_super_damage",
    "item_artifact_invulnerability",
    "item_armorInv",
    "megahealth",
    "weapon_rocketlauncher",
    "weapon_lightning",
    "item_armor2",
    "weapon_grenadelauncher",
    "item_armor1",
    "item_artifact_invisibility",
    "item_artifact_envirosuit",
)

# Goals 19-24 are reserved for the mod's built-in defaults.
MAX_ASSIGNED_GOAL = 18

_HEALTH_MEGA_SPAWNFLAG = 2


def assign_goals(markers: list[Marker]) -> int:
    """Assign goal numbers to important item markers.

    Lower goal numbers mean more desirable. Same-class items share a goal
    number only when they sit in different zones; different items in the
    same zone never share a number.

    Args:
        markers: Final marker list with zones assigned.

    Returns:
        int: Number of distinct goal numbers used.
    """
    ranked: list[tuple[int, Marker]] = []
    for marker in markers:
        key = _goal_class(marker)
        if key is None:
            continue
        ranked.append((GOAL_RANKING.index(key), marker))
    ranked.sort(key=lambda pair: (pair[0], pair[1].index))

    next_goal = 0
    goal_by_class: dict[str, int] = {}
    zones_by_goal: dict[int, set[int]] = {}
    for rank, marker in ranked:
        key = GOAL_RANKING[rank]
        shared = goal_by_class.get(key)
        if shared is not None and marker.zone not in zones_by_goal.get(shared, set()):
            marker.goal = shared
            zones_by_goal[shared].add(marker.zone)
            continue
        if next_goal >= MAX_ASSIGNED_GOAL:
            continue
        next_goal += 1
        marker.goal = next_goal
        goal_by_class.setdefault(key, next_goal)
        zones_by_goal[next_goal] = {marker.zone}
    return next_goal


def _goal_class(marker: Marker) -> str | None:
    """Return the ranking key for a marker, or None if it gets no goal."""
    if marker.entity is None or marker.nav_pos is None:
        return None
    classname = marker.classname
    if "item_health" == classname:
        spawnflags = _spawnflags(marker)
        if spawnflags & _HEALTH_MEGA_SPAWNFLAG:
            return "megahealth"
        return None
    if classname in GOAL_RANKING:
        return classname
    return None


def _spawnflags(marker: Marker) -> int:
    """Return the marker entity's spawnflags as an int (0 on parse failure)."""
    if marker.entity is None:
        return 0
    raw = marker.entity.properties.get("spawnflags", "0")
    try:
        return int(float(raw))
    except ValueError:
        return 0
