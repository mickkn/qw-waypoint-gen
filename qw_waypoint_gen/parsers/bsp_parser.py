"""
:File:      parsers/bsp_parser.py

:Details:   Quake BSP29 parser exposing entities, render geometry, collision
            hulls (clipnodes), submodels, and texture names for waypoint
            generation.

:Date:      2026-11-07
:Author:    Mick K
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path

from qw_waypoint_gen.geom.vector import Vec3, vec3
from qw_waypoint_gen.models import Entity, MapDocument, Surface
from qw_waypoint_gen.parsers.entity_parser import parse_entity_lump

LUMP_ENTITIES = 0
LUMP_PLANES = 1
LUMP_TEXTURES = 2
LUMP_VERTEXES = 3
LUMP_VISIBILITY = 4
LUMP_NODES = 5
LUMP_TEXINFO = 6
LUMP_FACES = 7
LUMP_LIGHTING = 8
LUMP_CLIPNODES = 9
LUMP_LEAFS = 10
LUMP_MARKSURFACES = 11
LUMP_EDGES = 12
LUMP_SURFEDGES = 13
LUMP_MODELS = 14
HEADER_LUMPS = 15
BSP_VERSION_QUAKE = 29

CONTENTS_EMPTY = -1
CONTENTS_SOLID = -2
CONTENTS_WATER = -3
CONTENTS_SLIME = -4
CONTENTS_LAVA = -5
CONTENTS_SKY = -6

MAX_HULLS = 4


class BspParseError(ValueError):
    """Raised when a file cannot be parsed as a Quake BSP29 map."""


@dataclass(frozen=True)
class Lump:
    """Byte range of one BSP header lump."""

    offset: int
    length: int


@dataclass(frozen=True)
class Plane:
    """BSP split plane."""

    normal: Vec3
    dist: float
    type_id: int


@dataclass(frozen=True)
class Face:
    """BSP render face referencing surfedges and a plane."""

    plane_num: int
    side: int
    first_edge: int
    num_edges: int
    texinfo: int
    light_ofs: int


@dataclass(frozen=True)
class ClipNode:
    """Collision BSP node. Children < 0 are CONTENTS_* values."""

    plane_num: int
    children: tuple[int, int]


@dataclass(frozen=True)
class BspNode:
    """Render BSP node. Children < 0 encode leaf index as -(leaf + 1)."""

    plane_num: int
    children: tuple[int, int]


@dataclass(frozen=True)
class BspLeaf:
    """Render BSP leaf carrying its CONTENTS_* value."""

    contents: int


@dataclass(frozen=True)
class TexInfo:
    """Texture projection info; miptex indexes the texture name table."""

    miptex: int
    flags: int


@dataclass(frozen=True)
class BspModel:
    """BSP submodel. Model 0 is the world; *N entities reference model N."""

    mins: Vec3
    maxs: Vec3
    origin: Vec3
    head_nodes: tuple[int, int, int, int]
    vis_leafs: int
    first_face: int
    num_faces: int

    def center(self) -> Vec3:
        """Return the center point of the model bounding box.

        Returns:
            Vec3: Midpoint between mins and maxs.
        """
        return vec3(
            (self.mins.x + self.maxs.x) * 0.5,
            (self.mins.y + self.maxs.y) * 0.5,
            (self.mins.z + self.maxs.z) * 0.5,
        )


class QuakeBsp:
    """Parsed Quake BSP29 file with lazy lump decoding and cached tables."""

    def __init__(self, path: Path, data: bytes, lumps: list[Lump]) -> None:
        """Initialize from raw file data and the decoded lump directory.

        Args:
            path: Source file path, kept for diagnostics.
            data: Complete raw file contents.
            lumps: Decoded lump directory (HEADER_LUMPS entries).
        """
        self.path = path
        self.data = data
        self.lumps = lumps
        self._planes: list[Plane] | None = None
        self._clipnodes: list[ClipNode] | None = None
        self._nodes: list[BspNode] | None = None
        self._leafs: list[BspLeaf] | None = None
        self._models: list[BspModel] | None = None
        self._hull0: list[ClipNode] | None = None

    @classmethod
    def load(cls, path: str | Path) -> "QuakeBsp":
        """Load and validate a BSP29 file.

        Args:
            path: Path to the .bsp file.

        Returns:
            QuakeBsp: Parsed BSP wrapper.

        Raises:
            BspParseError: If the file is truncated, has a wrong version, or a
                corrupt lump directory.
        """
        path = Path(path)
        data = path.read_bytes()
        if len(data) < 4 + HEADER_LUMPS * 8:
            raise BspParseError(f"File is too small to be a Quake BSP: {path}")

        version = struct.unpack_from("<i", data, 0)[0]
        if BSP_VERSION_QUAKE != version:
            raise BspParseError(
                f"Unsupported BSP version {version}; expected Quake BSP29. " "QuakeWorld maps should normally be BSP29."
            )

        lumps: list[Lump] = []
        cursor = 4
        for _ in range(HEADER_LUMPS):
            offset, length = struct.unpack_from("<ii", data, cursor)
            cursor += 8
            if offset < 0 or length < 0 or offset + length > len(data):
                raise BspParseError(f"Invalid BSP lump offset={offset} length={length}")
            lumps.append(Lump(offset, length))
        return cls(path, data, lumps)

    def lump_bytes(self, lump_id: int) -> bytes:
        """Return the raw bytes of one lump.

        Args:
            lump_id: LUMP_* index.

        Returns:
            bytes: Slice of the file covering the lump.
        """
        lump = self.lumps[lump_id]
        return self.data[lump.offset : lump.offset + lump.length]

    def entity_text(self) -> str:
        """Return the entity lump as text (up to the first NUL)."""
        return self.lump_bytes(LUMP_ENTITIES).split(b"\x00", 1)[0].decode("latin-1", errors="replace")

    def planes(self) -> list[Plane]:
        """Decode and cache the planes lump."""
        if self._planes is None:
            raw = self.lump_bytes(LUMP_PLANES)
            if 0 != len(raw) % 20:
                raise BspParseError("Plane lump length is not divisible by 20")
            out: list[Plane] = []
            for off in range(0, len(raw), 20):
                nx, ny, nz, dist, type_id = struct.unpack_from("<ffffi", raw, off)
                out.append(Plane(vec3(nx, ny, nz), dist, type_id))
            self._planes = out
        return self._planes

    def vertices(self) -> list[Vec3]:
        """Decode the vertex lump."""
        raw = self.lump_bytes(LUMP_VERTEXES)
        if 0 != len(raw) % 12:
            raise BspParseError("Vertex lump length is not divisible by 12")
        return [vec3(*struct.unpack_from("<fff", raw, off)) for off in range(0, len(raw), 12)]

    def edges(self) -> list[tuple[int, int]]:
        """Decode the edge lump as (vertex_a, vertex_b) pairs."""
        raw = self.lump_bytes(LUMP_EDGES)
        if 0 != len(raw) % 4:
            raise BspParseError("Edge lump length is not divisible by 4")
        return [struct.unpack_from("<HH", raw, off) for off in range(0, len(raw), 4)]

    def surfedges(self) -> list[int]:
        """Decode the surfedge lump (signed edge references)."""
        raw = self.lump_bytes(LUMP_SURFEDGES)
        if 0 != len(raw) % 4:
            raise BspParseError("Surfedge lump length is not divisible by 4")
        return [struct.unpack_from("<i", raw, off)[0] for off in range(0, len(raw), 4)]

    def faces(self) -> list[Face]:
        """Decode the face lump."""
        raw = self.lump_bytes(LUMP_FACES)
        if 0 != len(raw) % 20:
            raise BspParseError("Face lump length is not divisible by 20")
        out: list[Face] = []
        for off in range(0, len(raw), 20):
            plane_num, side, first_edge, num_edges, texinfo = struct.unpack_from("<HHIHH", raw, off)
            light_ofs = struct.unpack_from("<i", raw, off + 16)[0]
            out.append(Face(plane_num, side, first_edge, num_edges, texinfo, light_ofs))
        return out

    def clipnodes(self) -> list[ClipNode]:
        """Decode and cache the clipnodes lump (hulls 1 and 2)."""
        if self._clipnodes is None:
            raw = self.lump_bytes(LUMP_CLIPNODES)
            if 0 != len(raw) % 8:
                raise BspParseError("Clipnode lump length is not divisible by 8")
            out: list[ClipNode] = []
            for off in range(0, len(raw), 8):
                plane_num, child0, child1 = struct.unpack_from("<ihh", raw, off)
                out.append(ClipNode(plane_num, (child0, child1)))
            self._clipnodes = out
        return self._clipnodes

    def nodes(self) -> list[BspNode]:
        """Decode and cache the render node lump."""
        if self._nodes is None:
            raw = self.lump_bytes(LUMP_NODES)
            if 0 != len(raw) % 24:
                raise BspParseError("Node lump length is not divisible by 24")
            out: list[BspNode] = []
            for off in range(0, len(raw), 24):
                plane_num, child0, child1 = struct.unpack_from("<ihh", raw, off)
                out.append(BspNode(plane_num, (child0, child1)))
            self._nodes = out
        return self._nodes

    def leafs(self) -> list[BspLeaf]:
        """Decode and cache the leaf lump (contents only)."""
        if self._leafs is None:
            raw = self.lump_bytes(LUMP_LEAFS)
            if 0 != len(raw) % 28:
                raise BspParseError("Leaf lump length is not divisible by 28")
            self._leafs = [BspLeaf(struct.unpack_from("<i", raw, off)[0]) for off in range(0, len(raw), 28)]
        return self._leafs

    def models(self) -> list[BspModel]:
        """Decode and cache the submodel lump."""
        if self._models is None:
            raw = self.lump_bytes(LUMP_MODELS)
            if 0 != len(raw) % 64:
                raise BspParseError("Model lump length is not divisible by 64")
            out: list[BspModel] = []
            for off in range(0, len(raw), 64):
                values = struct.unpack_from("<9f7i", raw, off)
                out.append(
                    BspModel(
                        mins=vec3(values[0], values[1], values[2]),
                        maxs=vec3(values[3], values[4], values[5]),
                        origin=vec3(values[6], values[7], values[8]),
                        head_nodes=(values[9], values[10], values[11], values[12]),
                        vis_leafs=values[13],
                        first_face=values[14],
                        num_faces=values[15],
                    )
                )
            self._models = out
        return self._models

    def texinfos(self) -> list[TexInfo]:
        """Decode the texinfo lump (miptex index and flags per entry)."""
        raw = self.lump_bytes(LUMP_TEXINFO)
        if 0 != len(raw) % 40:
            raise BspParseError("Texinfo lump length is not divisible by 40")
        out: list[TexInfo] = []
        for off in range(0, len(raw), 40):
            miptex, flags = struct.unpack_from("<ii", raw, off + 32)
            out.append(TexInfo(miptex, flags))
        return out

    def texture_names(self) -> list[str]:
        """Decode the miptex texture name table.

        Returns:
            list[str]: One name per miptex slot; missing entries become "".
        """
        raw = self.lump_bytes(LUMP_TEXTURES)
        if len(raw) < 4:
            return []
        count = struct.unpack_from("<i", raw, 0)[0]
        if count <= 0 or len(raw) < 4 + count * 4:
            return []
        names: list[str] = []
        for i in range(count):
            offset = struct.unpack_from("<i", raw, 4 + i * 4)[0]
            if offset < 0 or offset + 16 > len(raw):
                names.append("")
                continue
            name_bytes = raw[offset : offset + 16].split(b"\x00", 1)[0]
            names.append(name_bytes.decode("latin-1", errors="replace"))
        return names

    def hull0_clipnodes(self) -> list[ClipNode]:
        """Build point-hull clipnodes from the render nodes (Mod_MakeHull0).

        Children pointing at leaves are replaced by the leaf CONTENTS_* value,
        matching how the engine constructs hull 0 for point traces.

        Returns:
            list[ClipNode]: Clipnode view of the render BSP tree.
        """
        if self._hull0 is None:
            leafs = self.leafs()
            out: list[ClipNode] = []
            for node in self.nodes():
                children = []
                for child in node.children:
                    if child >= 0:
                        children.append(child)
                    else:
                        leaf_index = -child - 1
                        contents = leafs[leaf_index].contents if leaf_index < len(leafs) else CONTENTS_SOLID
                        children.append(contents)
                out.append(ClipNode(node.plane_num, (children[0], children[1])))
            self._hull0 = out
        return self._hull0

    def surfaces(self, model_index: int | None = 0) -> list[Surface]:
        """Build render face polygons, optionally restricted to one submodel.

        Args:
            model_index: Submodel whose faces to return; None returns all faces.

        Returns:
            list[Surface]: Face polygons with outward normals and texture names.
        """
        planes = self.planes()
        vertices = self.vertices()
        edges = self.edges()
        surfedges = self.surfedges()
        faces = self.faces()
        texinfos = self.texinfos()
        texture_names = self.texture_names()

        face_range: range | None = None
        if model_index is not None:
            models = self.models()
            if model_index < len(models):
                model = models[model_index]
                face_range = range(model.first_face, model.first_face + model.num_faces)

        surfaces: list[Surface] = []
        for face_index, face in enumerate(faces):
            if face_range is not None and face_index not in face_range:
                continue
            if face.plane_num >= len(planes):
                continue
            face_vertices: list[Vec3] = []
            for i in range(face.num_edges):
                surfedge_index = face.first_edge + i
                if surfedge_index >= len(surfedges):
                    continue
                edge_ref = surfedges[surfedge_index]
                edge_index = abs(edge_ref)
                if edge_index >= len(edges):
                    continue
                a_idx, b_idx = edges[edge_index]
                vertex_index = a_idx if edge_ref >= 0 else b_idx
                if vertex_index < len(vertices):
                    face_vertices.append(vertices[vertex_index])

            if len(face_vertices) < 3:
                continue
            normal = planes[face.plane_num].normal
            if face.side:
                normal = normal * -1.0
            texture = ""
            if face.texinfo < len(texinfos):
                miptex = texinfos[face.texinfo].miptex
                if 0 <= miptex < len(texture_names):
                    texture = texture_names[miptex]
            surfaces.append(
                Surface(
                    vertices=face_vertices,
                    normal=normal,
                    texture=texture,
                    face_index=face_index,
                )
            )
        return surfaces

    def entities(self) -> list[Entity]:
        """Parse the entity lump into Entity models."""
        return parse_entity_lump(self.entity_text())


def parse_bsp(path: str | Path) -> MapDocument:
    """Parse a Quake BSP29 file into a MapDocument for waypoint generation.

    Args:
        path: Path to the .bsp file.

    Returns:
        MapDocument: Entities plus world render surfaces (source "bsp29").
    """
    bsp = QuakeBsp.load(path)
    return MapDocument(
        entities=bsp.entities(),
        surfaces=bsp.surfaces(model_index=0),
        source_format="bsp29",
    )
