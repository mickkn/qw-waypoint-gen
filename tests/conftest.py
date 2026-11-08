"""
:File:      tests/conftest.py

:Details:   Shared synthetic BSP fixtures for parser and batch generation tests.

:Date:      2026-11-08
:Author:    Mick K
"""

from __future__ import annotations

import struct

import pytest


@pytest.fixture
def collision_bsp() -> bytes:
    """Build a small BSP with a floor, two markers, and valid collision hulls."""
    blobs = {
        0: (
            b'{"classname" "worldspawn"}\n'
            b'{"classname" "info_player_deathmatch" "origin" "32 32 24"}\n'
            b'{"classname" "weapon_rocketlauncher" "origin" "96 96 24"}\0'
        ),
        1: struct.pack("<ffffi", 0.0, 0.0, 1.0, 0.0, 2)
        + struct.pack("<ffffi", 0.0, 0.0, 1.0, 24.0, 2),
        5: struct.pack("<ihh", 0, -1, -2) + bytes(16),
        9: struct.pack("<ihh", 1, -1, -2),
        10: struct.pack("<i", -1) + bytes(24) + struct.pack("<i", -2) + bytes(24),
        14: struct.pack(
            "<9f7i",
            0.0, 0.0, 0.0, 128.0, 128.0, 128.0, 0.0, 0.0, 0.0,
            0, 0, 0, 0, 1, 0, 0,
        ),
    }
    offset = 4 + 15 * 8
    header = bytearray(struct.pack("<i", 29))
    body = bytearray()
    for index in range(15):
        blob = blobs.get(index, b"")
        header.extend(struct.pack("<ii", offset, len(blob)))
        body.extend(blob)
        offset += len(blob)
    return bytes(header + body)
