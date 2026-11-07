"""
:File: test_exporter.py
:Details: Unit tests for the Frogbot QC exporter, including compatibility
          with the exact regexes used by waypoint_map_inject.py.

:Date: 2026-07-06

"""

from __future__ import annotations

import re

from qw_waypoint_gen.exporters.frogbot_qc import quakecify_map_name, render_frogbot_qc
from qw_waypoint_gen.geom.vector import vec3
from qw_waypoint_gen.nav.markers import Marker

# The statement regexes waypoint_map_inject.py parse_statement() accepts.
_INJECT_PATTERNS = (
    re.compile(r"N\((-?\d+),(-?\d+),(-?\d+)\)$"),
    re.compile(r"LSQ\(\)$"),
    re.compile(r"Z(\d+)\(m(\d+)\)$"),
    re.compile(r"G(\d+)\(m(\d+)\)$"),
    re.compile(r"m(\d+)\.T=(\d+)$"),
    re.compile(r"m(\d+)\.P([0-7])=m(\d+)$"),
    re.compile(r"m(\d+)\.D([0-7])=(\d+)$"),
    re.compile(r"custom_marker_start=(\d+)$"),
)
_INFO_PATTERN = re.compile(r"m(\d+) (\S+) (-?\d+ -?\d+ -?\d+);$", re.MULTILINE)


def _sample_markers() -> list[Marker]:
    entity = Marker(
        index=1,
        classname="weapon_rocketlauncher",
        info_pos=vec3(100.0, -50.0, 24.0),
        nav_pos=vec3(100.0, -50.0, 48.25),
        zone=1,
        goal=1,
        paths=[2],
    )
    customs = [
        Marker(
            index=i,
            classname="marker",
            info_pos=vec3(float(i * 10), float(-i * 10), 24.0),
            nav_pos=vec3(float(i * 10), float(-i * 10), 48.25),
            is_custom=True,
            zone=1,
            paths=[1] if i == 2 else [i - 1],
            path_flags={0: 1024} if i == 3 else {},
        )
        for i in range(2, 8)
    ]
    return [entity] + customs


def test_quakecify_map_name() -> None:
    assert "dmz1PLUSPLUS" == quakecify_map_name("dmz1++")
    assert "someDASHmap" == quakecify_map_name("some-map")
    assert "dm4" == quakecify_map_name("dm4")


def test_render_structure() -> None:
    text = render_frogbot_qc("testmap", _sample_markers())
    lines = text.splitlines()

    assert "void() map_testmap =" in lines
    assert "{" in lines
    assert "custom_marker_start=2;" in lines
    assert "LSQ();" in lines
    assert "};" in lines
    assert "/* MarkerInfo" in lines
    assert "*/" in lines

    n_lines = [ln for ln in lines if ln.startswith("N(")]
    assert 2 == len(n_lines)
    assert 4 == n_lines[0].count("N(")
    assert 2 == n_lines[1].count("N(")


def test_flags_only_for_flagged_slots() -> None:
    text = render_frogbot_qc("testmap", _sample_markers())
    assert "m3.D0=1024;" in text
    assert "m2.D0=" not in text


def test_every_statement_matches_injector_regexes() -> None:
    text = render_frogbot_qc("testmap", _sample_markers())
    in_body = False
    in_info = False
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("//"):
            continue
        if "/* MarkerInfo" in line:
            in_info = True
            continue
        if in_info:
            if "*/" in line:
                in_info = False
                continue
            assert _INFO_PATTERN.match(line), f"MarkerInfo line rejected: {line}"
            continue
        if "{" == line:
            in_body = True
            continue
        if "};" == line:
            in_body = False
            continue
        if not in_body:
            continue
        for statement in filter(None, (s.strip() for s in line.split(";"))):
            matched = any(p.match(statement) for p in _INJECT_PATTERNS)
            assert matched, f"Statement rejected by injector regexes: {statement}"


def test_marker_info_lists_all_markers_in_order() -> None:
    text = render_frogbot_qc("testmap", _sample_markers())
    info = text.split("/* MarkerInfo", 1)[1]
    indices = [int(m.group(1)) for m in _INFO_PATTERN.finditer(info)]
    assert list(range(1, 8)) == indices
    assert "m1 weapon_rocketlauncher 100 -50 24;" in info
