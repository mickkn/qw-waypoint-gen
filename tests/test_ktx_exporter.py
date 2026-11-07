"""
:File:      tests/test_ktx_exporter.py
:Details:   Unit tests for the KTX .bot exporter, checked against the command
            grammar of KTX 1.47 LoadBotRoutingFromFile() (marker_load.c).

:Date:      2026-11-07
:Author:    Mick K
"""

from __future__ import annotations

from qw_waypoint_gen.exporters.ktx_bot import encode_marker_flags
from qw_waypoint_gen.exporters.ktx_bot import encode_path_flags
from qw_waypoint_gen.exporters.ktx_bot import render_ktx_bot
from qw_waypoint_gen.geom.vector import vec3
from qw_waypoint_gen.nav.markers import Marker

# Command name -> argc (including the command) as enforced by the KTX loader.
_KTX_ARGC = {
    "CreateMarker": 4,
    "SetGoal": 3,
    "SetZone": 3,
    "SetMarkerPath": 4,
    "SetMarkerPathFlags": 4,
    "SetMarkerFlag": 3,
}
# KTX reads the file through a 128-byte line buffer.
_KTX_LINE_BUFFER = 128


def _sample_markers() -> list[Marker]:
    entity = Marker(
        index=1,
        classname="weapon_rocketlauncher",
        info_pos=vec3(100.0, -50.0, 24.0),
        nav_pos=vec3(100.0, -50.0, 48.25),
        zone=1,
        goal=3,
        paths=[2],
    )
    door = Marker(index=2, classname="door", info_pos=vec3(0.0, 0.0, 0.0), marker_type=64)
    customs = [
        Marker(
            index=i,
            classname="marker",
            info_pos=vec3(float(i * 10) + 0.5, float(-i * 10) - 0.5, 24.0),
            nav_pos=vec3(float(i * 10), float(-i * 10), 48.25),
            is_custom=True,
            zone=2,
            paths=[1] if 3 == i else [i - 1],
            path_flags={0: 1024} if 4 == i else {},
        )
        for i in range(3, 6)
    ]
    return [entity, door] + customs


def test_encode_path_flags() -> None:
    assert "j" == encode_path_flags(1024)
    assert "rj" == encode_path_flags(512 | 1024)
    assert "" == encode_path_flags(0)
    assert "" == encode_path_flags(4)


def test_encode_marker_flags() -> None:
    assert "u" == encode_marker_flags(1)
    assert "t" == encode_marker_flags(64)
    assert "" == encode_marker_flags(256 | 512)


def test_create_markers_only_for_customs_in_order() -> None:
    lines = render_ktx_bot("testmap", _sample_markers()).splitlines()
    creates = [ln for ln in lines if ln.startswith("CreateMarker")]
    assert ["CreateMarker 31 -31 24", "CreateMarker 41 -41 24", "CreateMarker 51 -51 24"] == creates
    first_other = next(i for i, ln in enumerate(lines) if ln.startswith("Set"))
    assert all(not ln.startswith("CreateMarker") for ln in lines[first_other:])


def test_per_marker_commands() -> None:
    text = render_ktx_bot("testmap", _sample_markers())
    assert "SetGoal 1 3\nSetZone 1 1\nSetMarkerPath 1 0 2\n" in text
    assert "SetMarkerPath 4 0 3\nSetMarkerPathFlags 4 0 j\n" in text
    assert "SetMarkerFlag 2 t\n" in text
    assert "SetMarkerPathFlags 3" not in text
    assert "SetZone 2" not in text


def test_every_line_parses_with_ktx_grammar() -> None:
    text = render_ktx_bot("testmap", _sample_markers())
    assert "\r" not in text
    for line in text.splitlines():
        assert len(line.encode("ascii")) + 1 < _KTX_LINE_BUFFER
        tokens = line.split()
        if not tokens or tokens[0].startswith("//"):
            continue
        assert _KTX_ARGC[tokens[0]] == len(tokens), line
        for token in tokens[1:]:
            if "SetMarkerPathFlags" == tokens[0] or "SetMarkerFlag" == tokens[0]:
                continue
            int(token)
