"""
:File:      tests/test_bsp_parser.py

:Details:   Unit tests for parsing Quake BSP29 entity and surface data.

:Date:      2026-11-07
:Author:    Mick K
"""

from __future__ import annotations

import struct
from pathlib import Path

import pytest

from qw_waypoint_gen.geom.vector import vec3
from qw_waypoint_gen.parsers.bsp_parser import BspParseError, QuakeBsp, parse_bsp
from qw_waypoint_gen.sim.hull_trace import HULL_PLAYER, HULL_POINT, HullTracer


def test_parse_minimal_bsp_entities_and_surface(tmp_path: Path) -> None:
    bsp = tmp_path / "mini.bsp"
    bsp.write_bytes(_minimal_bsp())

    doc = parse_bsp(bsp)

    assert doc.source_format == "bsp29"
    assert [e.classname for e in doc.entities] == ["worldspawn", "weapon_rocketlauncher"]
    assert doc.entities[1].origin is not None
    assert len(doc.surfaces) == 1
    assert doc.surfaces[0].normal.z == 1.0
    assert len(doc.surfaces[0].vertices) == 4


@pytest.mark.parametrize("hull", [HULL_POINT, HULL_PLAYER])
def test_valid_collision_hulls(tmp_path: Path, collision_bsp: bytes, hull: int) -> None:
    path = tmp_path / "valid.bsp"
    path.write_bytes(collision_bsp)
    tracer = HullTracer.for_world_hull(QuakeBsp.load(path), hull)
    assert not tracer.is_solid(vec3(32.0, 32.0, 32.0))
    assert tracer.is_solid(vec3(32.0, 32.0, -32.0))


@pytest.mark.parametrize(
    ("lump", "offset", "encoding", "value", "message"),
    [
        (9, 0, "<i", 2, "player hull node 0 references plane 2"),
        (9, 0, "<i", -1, "player hull node 0 references plane -1"),
        (9, 4, "<h", 1, "player hull node 0 has invalid child 1"),
        (9, 4, "<h", -7, "player hull node 0 has invalid child -7"),
        (9, 4, "<h", 0, "player hull contains a cycle"),
        (5, 0, "<i", 2, "point hull node 0 references plane 2"),
        (5, 4, "<h", 1, "point hull node 0 has invalid child 1"),
        (5, 4, "<h", -3, "render node references missing leaf 2"),
        (5, 4, "<h", 0, "point hull contains a cycle"),
        (14, 36, "<i", 1, "point hull has invalid root 1"),
        (14, 40, "<i", 1, "player hull has invalid root 1"),
        (14, 40, "<i", -7, "player hull has invalid root -7"),
        (10, 0, "<i", 0, "leaf 0 has invalid contents 0"),
        (1, 16, "<i", -1, "invalid plane 0"),
        (1, 16, "<i", 6, "invalid plane 0"),
        (1, 12, "<f", float("nan"), "invalid plane 0"),
    ],
)
def test_invalid_collision_hulls_raise_parse_error(
    tmp_path: Path, collision_bsp: bytes, lump: int, offset: int,
    encoding: str, value: float, message: str,
) -> None:
    data = bytearray(collision_bsp)
    lump_offset = struct.unpack_from("<i", data, 4 + lump * 8)[0]
    struct.pack_into(encoding, data, lump_offset + offset, value)
    path = tmp_path / "broken.bsp"
    path.write_bytes(data)
    with pytest.raises(BspParseError, match=message) as exc:
        HullTracer.for_world_hull(QuakeBsp.load(path))
    assert str(path) in str(exc.value)


def test_missing_world_model_raises_parse_error(tmp_path: Path, collision_bsp: bytes) -> None:
    data = bytearray(collision_bsp)
    struct.pack_into("<i", data, 4 + 14 * 8 + 4, 0)
    path = tmp_path / "missing-world.bsp"
    path.write_bytes(data)
    with pytest.raises(BspParseError, match="missing world model"):
        HullTracer.for_world_hull(QuakeBsp.load(path))


def _minimal_bsp() -> bytes:
    entity_text = b'{"classname" "worldspawn"}\n{"classname" "weapon_rocketlauncher" "origin" "32 32 24"}\0'
    planes = struct.pack("<ffffi", 0.0, 0.0, 1.0, 0.0, 2)
    vertices = b"".join(
        struct.pack("<fff", *v) for v in [(0.0, 0.0, 0.0), (128.0, 0.0, 0.0), (128.0, 128.0, 0.0), (0.0, 128.0, 0.0)]
    )
    edges = b"".join(struct.pack("<HH", *e) for e in [(0, 1), (1, 2), (2, 3), (3, 0)])
    surfedges = b"".join(struct.pack("<i", e) for e in [0, 1, 2, 3])
    face = struct.pack("<HHIHH4Bi", 0, 0, 0, 4, 0, 0, 0, 0, 0, -1)

    lumps_data = {0: entity_text, 1: planes, 3: vertices, 7: face, 12: edges, 13: surfedges}
    header_size = 4 + 15 * 8
    offset = header_size
    lumps = []
    body = bytearray()
    for i in range(15):
        blob = lumps_data.get(i, b"")
        if blob:
            lumps.append((offset, len(blob)))
            body.extend(blob)
            offset += len(blob)
        else:
            lumps.append((offset, 0))
    header = bytearray(struct.pack("<i", 29))
    for lump_offset, lump_len in lumps:
        header.extend(struct.pack("<ii", lump_offset, lump_len))
    return bytes(header + body)
