"""
:File:      sim/hull_trace.py

:Details:   Quake collision hull tracer. Ports SV_HullPointContents and
            SV_RecursiveHullCheck so waypoint generation can do real player-hull
            collision queries against BSP clipnodes.

:Date:      2026-11-07
:Author:    Mick K
"""

from __future__ import annotations

from dataclasses import dataclass, field

from qw_waypoint_gen.geom.vector import Vec3, vec3
from qw_waypoint_gen.parsers.bsp_parser import (CONTENTS_EMPTY, CONTENTS_LAVA, CONTENTS_SLIME, CONTENTS_SOLID,
                                                CONTENTS_WATER, ClipNode, Plane, QuakeBsp)

DIST_EPSILON = 0.03125

HULL_POINT = 0
HULL_PLAYER = 1
HULL_BIG = 2

# Player hull extents used by qbsp when building hull 1.
PLAYER_MINS = vec3(-16.0, -16.0, -24.0)
PLAYER_MAXS = vec3(16.0, 16.0, 32.0)


@dataclass
class Trace:
    """Result of a hull trace, mirroring the engine trace_t.

    Attributes:
        all_solid: Trace never left solid space.
        start_solid: Trace started inside a solid.
        fraction: Fraction of the move completed before hitting something.
        end_pos: Point reached by the trace.
        plane_normal: Normal of the plane that was hit (if fraction < 1).
        plane_dist: Distance of the plane that was hit.
        in_open: Trace passed through empty space.
        in_water: Trace passed through a liquid.
    """

    all_solid: bool = True
    start_solid: bool = False
    fraction: float = 1.0
    end_pos: Vec3 = field(default_factory=lambda: vec3(0.0, 0.0, 0.0))
    plane_normal: Vec3 = field(default_factory=lambda: vec3(0.0, 0.0, 0.0))
    plane_dist: float = 0.0
    in_open: bool = False
    in_water: bool = False

    def hit_something(self) -> bool:
        """Return True if the trace was blocked before reaching its end."""
        return self.fraction < 1.0


class HullTracer:
    """Collision queries against one BSP hull tree.

    A tracer wraps the world model's clipnodes for a fixed hull. Hull 1 bakes
    the player box into its planes, so all queries are point queries.
    """

    def __init__(self, clipnodes: list[ClipNode], planes: list[Plane], head_node: int) -> None:
        """Initialize the tracer.

        Args:
            clipnodes: Clipnode array for the chosen hull.
            planes: Shared BSP plane table.
            head_node: Root clipnode index for the traced model.
        """
        self._clipnodes = clipnodes
        self._planes = planes
        self._head_node = head_node

    @classmethod
    def for_world_hull(cls, bsp: QuakeBsp, hull: int = HULL_PLAYER) -> "HullTracer":
        """Create a tracer for the world model (submodel 0).

        Args:
            bsp: Parsed BSP file.
            hull: HULL_POINT, HULL_PLAYER, or HULL_BIG.

        Returns:
            HullTracer: Tracer bound to the requested world hull.
        """
        bsp.validate_collision_hulls()
        world = bsp.models()[0]
        if HULL_POINT == hull:
            return cls(bsp.hull0_clipnodes(), bsp.planes(), world.head_nodes[0])
        return cls(bsp.clipnodes(), bsp.planes(), world.head_nodes[hull])

    def point_contents(self, point: Vec3) -> int:
        """Return the CONTENTS_* value at a point (SV_HullPointContents).

        Args:
            point: Query position.

        Returns:
            int: CONTENTS_EMPTY, CONTENTS_SOLID, CONTENTS_WATER, etc.
        """
        num = self._head_node
        while num >= 0:
            node = self._clipnodes[num]
            plane = self._planes[node.plane_num]
            if plane.type_id < 3:
                dist = point.as_tuple()[plane.type_id] - plane.dist
            else:
                dist = plane.normal.dot(point) - plane.dist
            num = node.children[0] if dist >= 0.0 else node.children[1]
        return num

    def is_solid(self, point: Vec3) -> bool:
        """Return True if the point is inside solid space."""
        return CONTENTS_SOLID == self.point_contents(point)

    def is_liquid(self, point: Vec3) -> bool:
        """Return True if the point is in water, slime, or lava."""
        return self.point_contents(point) in (CONTENTS_WATER, CONTENTS_SLIME, CONTENTS_LAVA)

    def trace(self, start: Vec3, end: Vec3) -> Trace:
        """Trace a line through the hull (SV_RecursiveHullCheck).

        Args:
            start: Trace start position.
            end: Trace end position.

        Returns:
            Trace: Filled trace result; end_pos is the reached point.
        """
        trace = Trace(end_pos=end)
        self._recursive_check(self._head_node, 0.0, 1.0, start, end, trace)
        if trace.fraction >= 1.0 and not trace.start_solid:
            trace.end_pos = end
        return trace

    def _recursive_check(
        self,
        num: int,
        p1f: float,
        p2f: float,
        p1: Vec3,
        p2: Vec3,
        trace: Trace,
    ) -> bool:
        """Recursive clipnode walk; returns False once a hit is recorded."""
        if num < 0:
            if CONTENTS_SOLID != num:
                trace.all_solid = False
                if CONTENTS_EMPTY == num:
                    trace.in_open = True
                else:
                    trace.in_water = True
            else:
                trace.start_solid = True
            return True

        node = self._clipnodes[num]
        plane = self._planes[node.plane_num]

        if plane.type_id < 3:
            t1 = p1.as_tuple()[plane.type_id] - plane.dist
            t2 = p2.as_tuple()[plane.type_id] - plane.dist
        else:
            t1 = plane.normal.dot(p1) - plane.dist
            t2 = plane.normal.dot(p2) - plane.dist

        if t1 >= 0.0 and t2 >= 0.0:
            return self._recursive_check(node.children[0], p1f, p2f, p1, p2, trace)
        if t1 < 0.0 and t2 < 0.0:
            return self._recursive_check(node.children[1], p1f, p2f, p1, p2, trace)

        if t1 < 0.0:
            frac = (t1 + DIST_EPSILON) / (t1 - t2)
        else:
            frac = (t1 - DIST_EPSILON) / (t1 - t2)
        frac = min(1.0, max(0.0, frac))

        midf = p1f + (p2f - p1f) * frac
        mid = p1 + (p2 - p1) * frac
        side = 0 if t1 >= 0.0 else 1

        if not self._recursive_check(node.children[side], p1f, midf, p1, mid, trace):
            return False

        if CONTENTS_SOLID != self._contents_at(node.children[1 - side], mid):
            return self._recursive_check(node.children[1 - side], midf, p2f, mid, p2, trace)

        if trace.all_solid:
            return False

        if 0 == side:
            trace.plane_normal = plane.normal
            trace.plane_dist = plane.dist
        else:
            trace.plane_normal = plane.normal * -1.0
            trace.plane_dist = -plane.dist

        while CONTENTS_SOLID == self._contents_at(self._head_node, mid):
            frac -= 0.1
            if frac < 0.0:
                trace.fraction = midf
                trace.end_pos = mid
                return False
            midf = p1f + (p2f - p1f) * frac
            mid = p1 + (p2 - p1) * frac

        trace.fraction = midf
        trace.end_pos = mid
        return False

    def _contents_at(self, num: int, point: Vec3) -> int:
        """Return contents for a point starting from an arbitrary node."""
        while num >= 0:
            node = self._clipnodes[num]
            plane = self._planes[node.plane_num]
            if plane.type_id < 3:
                dist = point.as_tuple()[plane.type_id] - plane.dist
            else:
                dist = plane.normal.dot(point) - plane.dist
            num = node.children[0] if dist >= 0.0 else node.children[1]
        return num
