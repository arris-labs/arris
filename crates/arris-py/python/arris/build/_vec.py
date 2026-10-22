"""Plain 3-vector arithmetic on tuples, shared by the layer's modules."""

from __future__ import annotations

import math
from collections.abc import Sequence

Point = tuple[float, float, float]


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Sequence[float], b: Sequence[float]) -> Point:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def add(a: Point, b: Point) -> Point:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Point, b: Point) -> Point:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a: Point, s: float) -> Point:
    return (a[0] * s, a[1] * s, a[2] * s)


def fmt(v: Sequence[float]) -> str:
    return "(" + ", ".join(f"{float(c):g}" for c in v) + ")"


def unit(v: Sequence[float]) -> Point:
    """`v` scaled to length one; `ValueError` for a vector with no length."""
    x, y, z = (float(c) for c in v)
    n = math.sqrt(x * x + y * y + z * z)
    if not n > 0 or not math.isfinite(n):
        raise ValueError(f"direction {fmt(v)} has no length")
    return (x / n, y / n, z / n)
