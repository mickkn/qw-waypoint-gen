"""
:File:      tests/test_bsp_parser.py

:Details:   Unit tests for parsing Quake BSP29 entity and surface data.

:Date:      2026-11-07
:Author:    Mick K
"""

from __future__ import annotations

import struct
from pathlib import Path

from qw_waypoint_gen.parsers.bsp_parser import parse_bsp


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
