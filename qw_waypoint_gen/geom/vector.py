"""
:File:      geom/vector.py

:Details:   Define the three-dimensional vector model and vector operations.

:Date:      2026-11-07
:Author:    Mick K
"""

from __future__ import annotations

from math import sqrt

from pydantic import RootModel


class Vec3(RootModel[tuple[float, float, float]]):
    @property
    def x(self) -> float:
        return self.root[0]

    @property
    def y(self) -> float:
        return self.root[1]

    @property
    def z(self) -> float:
        return self.root[2]

    def __add__(self, other: "Vec3") -> "Vec3":
        return Vec3((self.x + other.x, self.y + other.y, self.z + other.z))

    def __sub__(self, other: "Vec3") -> "Vec3":
        return Vec3((self.x - other.x, self.y - other.y, self.z - other.z))

    def __mul__(self, scale: float) -> "Vec3":
        return Vec3((self.x * scale, self.y * scale, self.z * scale))

    def dot(self, other: "Vec3") -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def length(self) -> float:
        return sqrt(self.dot(self))

    def distance_to(self, other: "Vec3") -> float:
        return (self - other).length()

    def as_tuple(self) -> tuple[float, float, float]:
        return self.root

    def qvec(self) -> str:
        return f"'{self.x:.1f} {self.y:.1f} {self.z:.1f}'"


def vec3(x: float, y: float, z: float) -> Vec3:
    return Vec3((float(x), float(y), float(z)))
