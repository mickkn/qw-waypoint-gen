from __future__ import annotations

import re

from qw_waypoint_gen.geom.vector import vec3
from qw_waypoint_gen.models import Entity

_KEYVAL_RE = re.compile(r'"([^"]+)"\s+"([^"]*)"')


def parse_entity_lump(text: str) -> list[Entity]:
    entities: list[Entity] = []
    for block in _top_level_blocks(text):
        properties = dict(_KEYVAL_RE.findall(block))
        classname = properties.get("classname", "unknown")
        entities.append(
            Entity(
                classname=classname,
                origin=parse_origin(properties.get("origin")),
                properties=properties,
            )
        )
    return entities


def parse_origin(value: str | None):
    if not value:
        return None
    parts = value.split()
    if len(parts) != 3:
        return None
    try:
        return vec3(float(parts[0]), float(parts[1]), float(parts[2]))
    except ValueError:
        return None


def _top_level_blocks(text: str) -> list[str]:
    blocks: list[str] = []
    depth = 0
    start: int | None = None
    in_quote = False
    escape = False
    for i, ch in enumerate(text):
        if ch == '"' and not escape:
            in_quote = not in_quote
        escape = ch == "\\" and not escape
        if in_quote:
            continue
        if ch == "{":
            if depth == 0:
                start = i + 1
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0 and start is not None:
                blocks.append(text[start:i])
    return blocks
