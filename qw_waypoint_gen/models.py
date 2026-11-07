"""
:File:      models.py

:Details:   Shared data models for parsed map data (entities and render
            surfaces) used by the waypoint generation pipeline.

:Date:      2026-11-07
:Author:    Mick K
"""

from __future__ import annotations

from pydantic import BaseModel, Field

from qw_waypoint_gen.geom.vector import Vec3


class Entity(BaseModel):
    """One entity from the BSP entity lump."""

    classname: str
    origin: Vec3 | None = None
    properties: dict[str, str] = Field(default_factory=dict)


class Surface(BaseModel):
    """Compiled BSP face polygon.

    Render geometry, not collision geometry; used for floor sampling. All
    collision queries go through the clipnode hull tracer instead.
    """

    vertices: list[Vec3] = Field(default_factory=list)
    normal: Vec3
    texture: str = ""
    face_index: int = 0


class MapDocument(BaseModel):
    """Container for parsed map data."""

    entities: list[Entity] = Field(default_factory=list)
    surfaces: list[Surface] = Field(default_factory=list)
    source_format: str = "bsp29"
