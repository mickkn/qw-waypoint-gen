"""
:File: movement.py
:Details: QuakeWorld player movement simulation built on the hull tracer.
          Provides ground snapping, step-aware walk tests, ballistic jump
          tests, and fall classification for waypoint reachability.

:Date: 2026-07-06

"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from qw_waypoint_gen.geom.vector import Vec3, vec3
from qw_waypoint_gen.parsers.bsp_parser import CONTENTS_LAVA, CONTENTS_SLIME, CONTENTS_WATER
from qw_waypoint_gen.sim.hull_trace import HullTracer, Trace

# QuakeWorld movement constants.
GRAVITY = 800.0
JUMP_VELOCITY = 270.0
RUN_SPEED = 320.0
STEP_HEIGHT = 18.0
# Player origin sits 24 units above the feet (hull mins.z = -24).
ORIGIN_ABOVE_FLOOR = 24.0
# Maximum height gain of a jump: v^2 / (2 g).
MAX_JUMP_UP = JUMP_VELOCITY * JUMP_VELOCITY / (2.0 * GRAVITY)
# Falling faster than this on landing hurts in QW.
SAFE_FALL_VELOCITY = 650.0
SAFE_FALL_HEIGHT = SAFE_FALL_VELOCITY * SAFE_FALL_VELOCITY / (2.0 * GRAVITY)

_WALK_STEP = 16.0
_ARRIVE_DISTANCE_SQ = 24.0 * 24.0


class MoveResult(str, Enum):
    """Outcome of a simulated move between two standing positions."""

    WALK = "walk"
    WALK_WITH_FALL = "walk_fall"
    JUMP = "jump"
    SWIM = "swim"
    BLOCKED = "blocked"


@dataclass(frozen=True)
class GroundInfo:
    """Standing position information.

    Attributes:
        origin: Player origin (feet + 24) when standing here.
        in_liquid: True if the position is inside water, slime, or lava.
        contents: CONTENTS_* value at the origin.
    """

    origin: Vec3
    in_liquid: bool
    contents: int


class MovementSimulator:
    """Simulates simplified QuakeWorld player movement between points.

    All positions are player origins (24 units above the feet). The player
    hull tracer already accounts for the player bounding box, so every query
    is a point query.
    """

    def __init__(
        self,
        player_tracer: HullTracer,
        point_tracer: HullTracer,
        max_fall_height: float = SAFE_FALL_HEIGHT,
    ) -> None:
        """Initialize the simulator.

        Args:
            player_tracer: Tracer for hull 1 (player collision).
            point_tracer: Tracer for hull 0 (point contents, liquids).
            max_fall_height: Falls beyond this are rejected as deadly/unwanted.
        """
        self._hull = player_tracer
        self._point = point_tracer
        self._max_fall_height = max_fall_height

    def snap_to_ground(self, position: Vec3, max_drop: float = 2048.0) -> GroundInfo | None:
        """Drop a position to the floor and return the standing origin.

        Args:
            position: Approximate position to snap (any z near the floor).
            max_drop: Maximum distance to search downwards.

        Returns:
            GroundInfo | None: Standing info, or None if the position is in
            solid space or no floor exists within max_drop.
        """
        start = position
        if self._hull.is_solid(start):
            lifted = vec3(start.x, start.y, start.z + STEP_HEIGHT + 1.0)
            if self._hull.is_solid(lifted):
                return None
            start = lifted

        contents = self._point.point_contents(start)
        in_liquid = contents in (CONTENTS_WATER, CONTENTS_SLIME, CONTENTS_LAVA)
        if in_liquid:
            return GroundInfo(origin=start, in_liquid=True, contents=contents)

        down = vec3(start.x, start.y, start.z - max_drop)
        trace = self._hull.trace(start, down)
        if trace.start_solid or not trace.hit_something():
            return None
        if trace.plane_normal.z < 0.7:
            return None
        origin = vec3(trace.end_pos.x, trace.end_pos.y, trace.end_pos.z + 0.25)
        contents = self._point.point_contents(origin)
        return GroundInfo(
            origin=origin,
            in_liquid=contents in (CONTENTS_WATER, CONTENTS_SLIME, CONTENTS_LAVA),
            contents=contents,
        )

    def classify_move(self, start: Vec3, goal: Vec3) -> MoveResult:
        """Classify how a player can move from start to goal.

        Both endpoints must be standing origins (or swim positions). Walking
        is attempted first, then a ballistic jump if walking fails.

        Args:
            start: Starting player origin.
            goal: Target player origin.

        Returns:
            MoveResult: WALK, WALK_WITH_FALL, JUMP, SWIM, or BLOCKED.
        """
        if self._is_liquid(start) and self._is_liquid(goal):
            return self._classify_swim(start, goal)

        walk = self._try_walk(start, goal)
        if walk is not None:
            return walk
        if self._try_jump(start, goal):
            return MoveResult.JUMP
        return MoveResult.BLOCKED

    def _is_liquid(self, position: Vec3) -> bool:
        """Return True if the position is inside a liquid volume."""
        return self._point.point_contents(position) in (
            CONTENTS_WATER,
            CONTENTS_SLIME,
            CONTENTS_LAVA,
        )

    def _classify_swim(self, start: Vec3, goal: Vec3) -> MoveResult:
        """Classify an underwater move: clear line through non-solid space."""
        trace = self._hull.trace(start, goal)
        if trace.start_solid or trace.hit_something():
            return MoveResult.BLOCKED
        return MoveResult.SWIM

    def _try_walk(self, start: Vec3, goal: Vec3) -> MoveResult | None:
        """Step-walk towards goal; return a result or None if walking fails.

        Simulates ground movement in _WALK_STEP increments with 18-unit
        step-up and downward ground snapping. Drops taller than STEP_HEIGHT
        mark the move as WALK_WITH_FALL; drops beyond max_fall_height fail.
        """
        position = start
        fell = False
        distance = _horizontal_distance(position, goal)
        max_iterations = int(distance / _WALK_STEP) * 2 + 16

        for _ in range(max_iterations):
            if _horizontal_distance_sq(position, goal) <= _ARRIVE_DISTANCE_SQ:
                if abs(position.z - goal.z) <= STEP_HEIGHT + 1.0:
                    return MoveResult.WALK_WITH_FALL if fell else MoveResult.WALK
                return None

            step = _horizontal_step(position, goal, _WALK_STEP)
            advanced = self._step_move(position, step)
            if advanced is None:
                return None
            moved, drop = advanced
            if drop > self._max_fall_height:
                return None
            if drop > STEP_HEIGHT:
                fell = True
            if _horizontal_distance_sq(position, moved) < 0.25:
                return None
            position = moved
        return None

    def _step_move(self, position: Vec3, step: Vec3) -> tuple[Vec3, float] | None:
        """Advance one step with step-up and ground snap.

        Args:
            position: Current standing origin.
            step: Horizontal step vector to apply.

        Returns:
            tuple[Vec3, float] | None: New origin and drop height, or None if
            blocked by a wall or hovering over a bottomless drop.
        """
        target = position + step
        trace = self._hull.trace(position, target)
        reached = trace.end_pos

        if trace.hit_something():
            stepped = self._step_up_move(position, target, trace.fraction)
            if stepped is None:
                stepped = self._slide_move(position, step, trace)
            if stepped is None:
                return None
            reached = stepped

        if self._is_liquid(reached):
            return reached, 0.0

        down = vec3(reached.x, reached.y, reached.z - self._max_fall_height - STEP_HEIGHT)
        ground = self._hull.trace(reached, down)
        if not ground.hit_something():
            return None
        if ground.plane_normal.z < 0.7 and ground.end_pos.z < reached.z - STEP_HEIGHT:
            return None
        drop = max(0.0, reached.z - ground.end_pos.z)
        landed = vec3(ground.end_pos.x, ground.end_pos.y, ground.end_pos.z + 0.25)
        return landed, drop

    def simulate_launch(self, start: Vec3, velocity: Vec3, max_time: float = 3.0) -> Vec3 | None:
        """Simulate a ballistic launch (jump pad) and return the landing origin.

        Integrates gravity in small steps with hull traces; wall and ceiling
        hits clip the velocity along the plane (PM_ClipVelocity) and flight
        continues until a floor is hit or time runs out.

        Args:
            start: Launch position (player origin).
            velocity: Initial velocity in units per second.
            max_time: Simulation cutoff in seconds.

        Returns:
            Vec3 | None: Landing origin, or None if no floor was reached.
        """
        dt = 0.05
        position = start
        vx, vy, vz = velocity.x, velocity.y, velocity.z
        for _ in range(int(max_time / dt)):
            vz -= GRAVITY * dt
            target = vec3(position.x + vx * dt, position.y + vy * dt, position.z + vz * dt)
            trace = self._hull.trace(position, target)
            if trace.start_solid:
                return None
            if trace.hit_something():
                normal = trace.plane_normal
                if normal.z >= 0.7:
                    return vec3(trace.end_pos.x, trace.end_pos.y, trace.end_pos.z + 0.25)
                into = vx * normal.x + vy * normal.y + vz * normal.z
                vx -= normal.x * into
                vy -= normal.y * into
                vz -= normal.z * into
            position = trace.end_pos
        return None

    def _step_up_move(self, position: Vec3, target: Vec3, flat_fraction: float) -> Vec3 | None:
        """Retry a blocked move after an 18-unit step up (stair climbing).

        Args:
            position: Current standing origin.
            target: Intended horizontal move target.
            flat_fraction: Fraction the flat move achieved before blocking.

        Returns:
            Vec3 | None: Reached position, or None if stepping up gains
            nothing over the flat move.
        """
        lifted = vec3(position.x, position.y, position.z + STEP_HEIGHT)
        up_trace = self._hull.trace(position, lifted)
        if up_trace.start_solid:
            return None
        raised_target = vec3(target.x, target.y, up_trace.end_pos.z)
        step_trace = self._hull.trace(up_trace.end_pos, raised_target)
        if step_trace.fraction <= flat_fraction + 1e-6:
            return None
        return step_trace.end_pos

    def _slide_move(self, position: Vec3, step: Vec3, blocked: Trace) -> Vec3 | None:
        """Slide along the blocking plane like PM_FlyMove velocity clipping.

        Args:
            position: Current standing origin.
            step: Intended horizontal step vector.
            blocked: Trace that hit the blocking plane.

        Returns:
            Vec3 | None: Position after sliding, or None when sliding gains
            no meaningful distance.
        """
        normal = blocked.plane_normal
        into = step.dot(normal)
        slide = vec3(
            step.x - normal.x * into,
            step.y - normal.y * into,
            0.0,
        )
        if slide.length() < 1.0:
            return None
        start = blocked.end_pos
        slide_trace = self._hull.trace(start, start + slide)
        moved_sq = _horizontal_distance_sq(position, slide_trace.end_pos)
        if moved_sq < 1.0:
            return None
        return slide_trace.end_pos

    def _try_jump(self, start: Vec3, goal: Vec3) -> bool:
        """Check a running-jump ballistic arc from start to goal.

        Uses flat-ground kinematics with run speed and jump velocity, then
        verifies hull clearance along the sampled arc.
        """
        dz = goal.z - start.z
        if dz > MAX_JUMP_UP - 0.5:
            return False
        horizontal = _horizontal_distance(start, goal)
        if horizontal < 1.0:
            return False

        flight_time = _ballistic_flight_time(dz)
        if flight_time is None:
            return False
        if horizontal > RUN_SPEED * flight_time + 16.0:
            return False

        direction = _horizontal_step(start, goal, 1.0)
        speed = min(RUN_SPEED, horizontal / flight_time)
        samples = max(4, int(flight_time / 0.05))
        previous = start
        for i in range(1, samples + 1):
            t = flight_time * i / samples
            point = vec3(
                start.x + direction.x * speed * t,
                start.y + direction.y * speed * t,
                start.z + JUMP_VELOCITY * t - 0.5 * GRAVITY * t * t,
            )
            trace = self._hull.trace(previous, point)
            if trace.start_solid:
                return False
            if trace.hit_something():
                landing = trace.end_pos
                near_goal = _horizontal_distance_sq(landing, goal) <= _ARRIVE_DISTANCE_SQ * 4.0
                return near_goal and trace.plane_normal.z >= 0.7
            previous = point
        return _horizontal_distance_sq(previous, goal) <= _ARRIVE_DISTANCE_SQ * 4.0


def _ballistic_flight_time(dz: float) -> float | None:
    """Return time until a jump arc crosses height dz on the way down.

    Args:
        dz: Height difference (positive is upward) between landing and launch.

    Returns:
        float | None: Flight time in seconds, or None if unreachable.
    """
    # Solve 0.5 g t^2 - v t + dz = 0 for the later root.
    discriminant = JUMP_VELOCITY * JUMP_VELOCITY - 2.0 * GRAVITY * dz
    if discriminant <= 0.0:
        return None
    return (JUMP_VELOCITY + discriminant**0.5) / GRAVITY


def _horizontal_distance(a: Vec3, b: Vec3) -> float:
    """Return the XY-plane distance between two points."""
    return _horizontal_distance_sq(a, b) ** 0.5


def _horizontal_distance_sq(a: Vec3, b: Vec3) -> float:
    """Return the squared XY-plane distance between two points."""
    dx = a.x - b.x
    dy = a.y - b.y
    return dx * dx + dy * dy


def _horizontal_step(position: Vec3, goal: Vec3, length: float) -> Vec3:
    """Return a horizontal vector of the given length pointing at goal."""
    dx = goal.x - position.x
    dy = goal.y - position.y
    distance = (dx * dx + dy * dy) ** 0.5
    if distance < 1e-6:
        return vec3(0.0, 0.0, 0.0)
    scale = min(length, distance) / distance
    return vec3(dx * scale, dy * scale, 0.0)
