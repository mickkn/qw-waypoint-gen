"""
:File: test_hull_trace.py
:Details: Unit tests for the Quake hull tracer using a synthetic box room
          built directly from clipnodes.

:Date: 2026-07-06

"""

from __future__ import annotations

from qw_waypoint_gen.geom.vector import vec3
from qw_waypoint_gen.parsers.bsp_parser import CONTENTS_EMPTY, CONTENTS_SOLID, ClipNode, Plane
from qw_waypoint_gen.sim.hull_trace import HullTracer


def _box_room_tracer() -> HullTracer:
    """Build a tracer for an empty 256^3 box room surrounded by solid."""
    planes = [
        Plane(vec3(1.0, 0.0, 0.0), 0.0, 0),
        Plane(vec3(1.0, 0.0, 0.0), 256.0, 0),
        Plane(vec3(0.0, 1.0, 0.0), 0.0, 1),
        Plane(vec3(0.0, 1.0, 0.0), 256.0, 1),
        Plane(vec3(0.0, 0.0, 1.0), 0.0, 2),
        Plane(vec3(0.0, 0.0, 1.0), 256.0, 2),
    ]
    clipnodes = [
        ClipNode(0, (1, CONTENTS_SOLID)),
        ClipNode(1, (CONTENTS_SOLID, 2)),
        ClipNode(2, (3, CONTENTS_SOLID)),
        ClipNode(3, (CONTENTS_SOLID, 4)),
        ClipNode(4, (5, CONTENTS_SOLID)),
        ClipNode(5, (CONTENTS_SOLID, CONTENTS_EMPTY)),
    ]
    return HullTracer(clipnodes, planes, 0)


def test_point_contents_inside_and_outside() -> None:
    tracer = _box_room_tracer()
    assert CONTENTS_EMPTY == tracer.point_contents(vec3(128.0, 128.0, 128.0))
    assert CONTENTS_SOLID == tracer.point_contents(vec3(-16.0, 128.0, 128.0))
    assert CONTENTS_SOLID == tracer.point_contents(vec3(128.0, 128.0, 300.0))
    assert tracer.is_solid(vec3(128.0, 128.0, -1.0))
    assert not tracer.is_solid(vec3(1.0, 1.0, 1.0))


def test_trace_hits_floor() -> None:
    tracer = _box_room_tracer()
    trace = tracer.trace(vec3(128.0, 128.0, 128.0), vec3(128.0, 128.0, -64.0))
    assert trace.hit_something()
    assert not trace.start_solid
    assert abs(trace.end_pos.z) < 1.0
    assert trace.plane_normal.z == 1.0


def test_trace_clear_line() -> None:
    tracer = _box_room_tracer()
    trace = tracer.trace(vec3(16.0, 128.0, 128.0), vec3(240.0, 128.0, 128.0))
    assert not trace.hit_something()
    assert 1.0 == trace.fraction


def test_trace_hits_wall_normal_points_back() -> None:
    tracer = _box_room_tracer()
    trace = tracer.trace(vec3(128.0, 128.0, 128.0), vec3(400.0, 128.0, 128.0))
    assert trace.hit_something()
    assert trace.plane_normal.x == -1.0
    assert abs(trace.end_pos.x - 256.0) < 1.0
