"""`Workplane` and `Sketch`: a frame to draw on, the 2D shapes drawn on it,
and the solids a sketch makes."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, replace
from typing import Literal

from .._arris import ArrisError, Frame, FrameLaw, Loop, Model, Path, Profile, Segment
from ._solid import Solid, default_model
from ._vec import Point, add, cross, dot, fmt, scale, sub, unit

_Uv = tuple[float, float]


class SketchError(ArrisError):
    """A sketch or workplane that cannot be drawn as asked: a path closed
    with no segments, an arc with no tangent or a straight one, a slot not
    longer than it is wide, a hole from another plane.

    ```python
    from arris.build import SketchError, Workplane

    try:
        Workplane.XY().start((0, 0)).arc_to((1, 1))
    except SketchError as error:
        assert "tangent" in str(error)
    else:
        raise AssertionError("the first arc has no tangent to follow")
    ```
    """


@dataclass(frozen=True)
class Workplane:
    """A plane to draw on, as a frame (`arris.Frame`'s `origin`, `x`, `y`
    and `z`, orthonormal and right-handed): sketches are drawn in `x` and
    `y`, and a sketch's solids grow along `z`.

    ```python
    from arris.build import Workplane

    top = Workplane.XY(offset=5).rect(4, 2).extrude(3)
    assert top.model.bounds(top.body) is not None
    assert Workplane.XZ().z == (0.0, -1.0, 0.0)
    ```
    """

    origin: Point
    x: Point
    y: Point
    z: Point

    @classmethod
    def of(
        cls, origin: Sequence[float], normal: Sequence[float], x_axis: Sequence[float] | None = None
    ) -> Workplane:
        """The plane through `origin` with the unit of `normal` as its
        `z`. `x_axis` is made square to the normal, so any direction not
        along it will do; without one, the world axis least aligned with the
        normal is used. Raises `SketchError` for a zero `normal` or an
        `x_axis` along it.

        ```python
        from arris.build import Workplane

        plane = Workplane.of((1, 2, 3), (0, 0, 2), x_axis=(1, 1, 0))
        assert plane.z == (0.0, 0.0, 1.0) and abs(plane.x[0] - plane.x[1]) < 1e-12
        ```
        """
        try:
            z = unit(normal)
            if x_axis is None:
                # The coordinate axis the normal has least of, so the cross
                # product below is well conditioned.
                least = min(range(3), key=lambda i: (abs(z[i]), i))
                x_axis = tuple(1.0 if i == least else 0.0 for i in range(3))
            x_raw = tuple(float(c) for c in x_axis)
            x = unit(sub((x_raw[0], x_raw[1], x_raw[2]), scale(z, dot(x_raw, z))))
        except ValueError as error:
            raise SketchError(f"workplane through {fmt(origin)}: {error}") from error
        o = (float(origin[0]), float(origin[1]), float(origin[2]))
        return cls(o, x, cross(z, x), z)

    @classmethod
    def from_frame(cls, frame: Frame) -> Workplane:
        """The plane of a binding `Frame` (`Model.face_frame`).

        ```python
        import arris
        from arris.build import Solid, Workplane

        box = Solid.box((0, 0, 0), (1, 2, 3))
        frame = box.model.face_frame(box.faces().extreme((0, 0, 1)).one())
        assert Workplane.from_frame(frame).origin[2] == 3.0
        ```
        """
        return cls(frame.origin, frame.x, frame.y, frame.z)

    @classmethod
    def XY(cls, offset: float = 0.0) -> Workplane:
        """The plane of `x` and `y`, moved `offset` along `z`.

        ```python
        from arris.build import Workplane

        assert Workplane.XY(2).origin == (0.0, 0.0, 2.0)
        ```
        """
        return cls((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)).offset(
            offset
        )

    @classmethod
    def YZ(cls, offset: float = 0.0) -> Workplane:
        """The plane of `y` and `z` (drawn `x` is `y`, drawn `y` is `z`), moved
        `offset` along the world `x`.

        ```python
        from arris.build import Workplane

        assert Workplane.YZ().z == (1.0, 0.0, 0.0)
        ```
        """
        return cls((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)).offset(
            offset
        )

    @classmethod
    def XZ(cls, offset: float = 0.0) -> Workplane:
        """The plane of `x` and `z` (drawn `y` is `z`), moved `offset` along
        its normal, the world `-y`: the frame stays right-handed.

        ```python
        from arris.build import Workplane

        assert Workplane.XZ(2).origin == (0.0, -2.0, 0.0)
        ```
        """
        return cls((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0)).offset(
            offset
        )

    def offset(self, distance: float) -> Workplane:
        """This plane moved `distance` along its normal.

        ```python
        from arris.build import Workplane

        assert Workplane.XY().offset(1).offset(2) == Workplane.XY(3)
        ```
        """
        return replace(self, origin=add(self.origin, scale(self.z, float(distance))))

    def rotated(self, angle: float, axis: Sequence[float] | None = None) -> Workplane:
        """This plane turned by `angle` radians about `axis` (a world
        direction through the plane's origin, right-handed), by default its
        own normal.

        ```python
        import math
        from arris.build import Workplane

        tilted = Workplane.XY().rotated(math.pi / 2, (1, 0, 0))
        assert max(abs(a - b) for a, b in zip(tilted.z, (0, -1, 0))) < 1e-12
        ```
        """
        try:
            k = unit(axis) if axis is not None else self.z
        except ValueError as error:
            raise SketchError(f"rotated(): {error}") from error
        c, s = math.cos(angle), math.sin(angle)

        def turn(v: Point) -> Point:
            # Rodrigues: v cos + (k × v) sin + k (k · v) (1 - cos).
            return add(add(scale(v, c), scale(cross(k, v), s)), scale(k, dot(k, v) * (1 - c)))

        return Workplane(self.origin, turn(self.x), turn(self.y), turn(self.z))

    def point(self, u: float, v: float, w: float = 0.0) -> Point:
        """The world point at `(u, v)` on the plane and `w` along its normal.

        ```python
        from arris.build import Workplane

        assert Workplane.XZ().point(1, 2) == (1.0, 0.0, 2.0)
        ```
        """
        return add(
            self.origin, add(add(scale(self.x, u), scale(self.y, v)), scale(self.z, w))
        )

    def direction(self, u: float, v: float) -> Point:
        """The world direction of the plane's vector `(u, v)`.

        ```python
        from arris.build import Workplane

        assert Workplane.YZ().direction(1, 0) == (0.0, 1.0, 0.0)
        ```
        """
        return add(scale(self.x, u), scale(self.y, v))

    def at(self, points: Sequence[Sequence[float]]) -> list[Workplane]:
        """One plane per point of `points` (in this plane's coordinates), each
        with this plane's axes and its origin at the point.

        ```python
        from arris.build import Workplane

        spots = Workplane.XY(2).at([(1, 0), (0, 3)])
        assert [spot.origin for spot in spots] == [(1.0, 0.0, 2.0), (0.0, 3.0, 2.0)]
        ```
        """
        return [replace(self, origin=self.point(p[0], p[1])) for p in points]

    def polar(self, count: int, radius: float, start: float = 0.0) -> list[Workplane]:
        """`count` planes evenly round the origin on a circle of `radius`,
        the first `start` radians from `x`, each turned so its own `x` points
        away from the centre: a shape drawn on one is the same shape at every
        place.

        ```python
        import math
        from arris.build import Workplane

        planes = Workplane.XY().polar(4, 2.0)
        assert planes[1].origin == (planes[1].origin[0], 2.0, 0.0)
        assert abs(planes[1].origin[0]) < 1e-12 and planes[1].x[1] > 1 - 1e-12
        ```
        """
        planes = []
        for i in range(count):
            a = start + 2 * math.pi * i / count
            c, s = math.cos(a), math.sin(a)
            planes.append(
                Workplane(
                    self.point(radius * c, radius * s),
                    self.direction(c, s),
                    self.direction(-s, c),
                    self.z,
                )
            )
        return planes

    def grid(self, nx: int, ny: int, dx: float, dy: float) -> list[Workplane]:
        """`nx` by `ny` planes a step of `dx` and `dy` apart, centred on the
        origin, `x` the slow direction.

        ```python
        from arris.build import Workplane

        corners = Workplane.XY().grid(2, 2, 10, 4)
        assert [c.origin[:2] for c in corners] == [(-5.0, -2.0), (-5.0, 2.0), (5.0, -2.0), (5.0, 2.0)]
        ```
        """
        return self.at(
            [
                ((i - (nx - 1) / 2) * dx, (j - (ny - 1) / 2) * dy)
                for i in range(nx)
                for j in range(ny)
            ]
        )

    def rect(self, width: float, height: float, *, center: Sequence[float] = (0, 0)) -> Sketch:
        """A `width` by `height` rectangle centred on `center`.

        ```python
        from arris.build import Workplane

        slab = Workplane.XY().rect(4, 2).extrude(1)
        assert abs(slab.model.mass_properties(slab.body).volume - 8.0) < 1e-9
        ```
        """
        cu, cv = center
        hw, hh = width / 2, height / 2
        return self.polygon(
            [(cu - hw, cv - hh), (cu + hw, cv - hh), (cu + hw, cv + hh), (cu - hw, cv + hh)]
        )

    def circle(self, radius: float, *, center: Sequence[float] = (0, 0)) -> Sketch:
        """A circle of `radius` about `center`.

        ```python
        import math
        from arris.build import Workplane

        disc = Workplane.XY().circle(2).extrude(1)
        assert abs(disc.model.mass_properties(disc.body).volume - 4 * math.pi) < 1e-9
        ```
        """
        return Sketch(self, Loop.circle((center[0], center[1]), radius))

    def polygon(self, points: Sequence[Sequence[float]]) -> Sketch:
        """The polygon through `points` in order, closed back to the first.

        ```python
        from arris.build import Workplane

        wedge = Workplane.XY().polygon([(0, 0), (4, 0), (0, 3)]).extrude(2)
        assert abs(wedge.model.mass_properties(wedge.body).volume - 12.0) < 1e-9
        ```
        """
        return Sketch(self, Loop.polygon([(p[0], p[1]) for p in points]))

    def slot(
        self,
        length: float,
        width: float,
        *,
        center: Sequence[float] = (0, 0),
        angle: float = 0.0,
    ) -> Sketch:
        """A stadium: two straight sides of `width` apart closed by half
        circles, `length` overall along the direction `angle` radians from
        the plane's `x`, centred on `center`. Raises `SketchError` unless
        `length` exceeds `width`.

        ```python
        import math
        from arris.build import Workplane

        bar = Workplane.XY().slot(10, 4).extrude(1)
        volume = bar.model.mass_properties(bar.body).volume
        assert abs(volume - (6 * 4 + math.pi * 2**2)) < 1e-9
        ```
        """
        if not (length > width > 0):
            raise SketchError(f"slot: length {length:g} must exceed width {width:g}, both positive")
        ux, uy = math.cos(angle), math.sin(angle)
        nx, ny = -uy, ux
        half, reach = width / 2, length / 2 - width / 2
        cu, cv = center

        def at(along: float, across: float) -> _Uv:
            return (cu + ux * along + nx * across, cv + uy * along + ny * across)

        path = [
            Segment.line(at(reach, -half)),
            Segment.arc(at(reach, half), at(reach + half, 0)),
            Segment.line(at(-reach, half)),
            Segment.arc(at(-reach, -half), at(-reach - half, 0)),
        ]
        return Sketch(self, Loop.path(at(-reach, -half), path))

    def start(self, to: Sequence[float]) -> PathSketch:
        """Begin a path at `to`, drawn with `line_to` and `arc_to` and ended
        by `close`.

        ```python
        from arris.build import Workplane

        tab = Workplane.XY().start((0, 0)).line_to((4, 0)).line_to((4, 2)).close()
        assert len(tab.holes) == 0
        ```
        """
        point = (float(to[0]), float(to[1]))
        return PathSketch(self, point, (), point, None)


@dataclass(frozen=True, eq=False)
class PathSketch:
    """A path under construction on `plane`: from `start` through
    `segments` to `cursor`. Each call returns a new path; `close` makes the
    `Sketch`.

    ```python
    from arris.build import Workplane

    path = Workplane.XY().start((0, 0)).line_to((2, 0))
    assert path.cursor == (2.0, 0.0) and len(path.segments) == 1
    ```
    """

    plane: Workplane
    start: _Uv
    segments: tuple[Segment, ...]
    cursor: _Uv
    tangent: _Uv | None

    def line_to(self, to: Sequence[float]) -> PathSketch:
        """A straight segment from the cursor to `to`.

        ```python
        from arris.build import Workplane

        path = Workplane.XY().start((0, 0)).line_to((3, 4))
        assert path.cursor == (3.0, 4.0) and path.tangent == (0.6, 0.8)
        ```
        """
        end = (float(to[0]), float(to[1]))
        dx, dy = end[0] - self.cursor[0], end[1] - self.cursor[1]
        length = math.hypot(dx, dy)
        if not length > 0:
            raise SketchError(f"line_to({end}): the segment has no length")
        return replace(
            self,
            segments=(*self.segments, Segment.line(end)),
            cursor=end,
            tangent=(dx / length, dy / length),
        )

    def arc_to(self, to: Sequence[float], tangent: Sequence[float] | None = None) -> PathSketch:
        """A circular arc from the cursor to `to`, leaving the cursor along
        `tangent` (a direction in the plane, by default the direction the
        path arrived in). The arc's via-point is computed. Raises
        `SketchError` with no tangent to follow, or when `to` lies on the
        tangent line and the arc would be straight.

        ```python
        from arris.build import Workplane

        # A half disc: the chord, then an arc that leaves the chord's end
        # square to it.
        half = (
            Workplane.XY().start((-2, 0)).line_to((2, 0)).arc_to((-2, 0), tangent=(0, 1)).close()
        )
        body = half.extrude(1)
        assert abs(body.model.mass_properties(body.body).volume - 2 * 3.141592653589793) < 1e-9
        ```
        """
        end = (float(to[0]), float(to[1]))
        t = self.tangent
        if tangent is not None:
            length = math.hypot(tangent[0], tangent[1])
            if not length > 0:
                raise SketchError(f"arc_to({end}): the tangent {tuple(tangent)} has no length")
            t = (tangent[0] / length, tangent[1] / length)
        if t is None:
            raise SketchError(f"arc_to({end}): the first segment needs a tangent=(u, v) to leave by")
        p = self.cursor
        d = (end[0] - p[0], end[1] - p[1])
        n = (-t[1], t[0])
        side = d[0] * n[0] + d[1] * n[1]
        if side == 0:
            raise SketchError(
                f"arc_to({end}): lies on the tangent line from {p}, the arc would be straight"
            )
        # The centre lies on the normal at the cursor, as far from it as from
        # the end: r = |d|² / (2 d·n), negative when it is on the right.
        r = (d[0] * d[0] + d[1] * d[1]) / (2 * side)
        centre = (p[0] + r * n[0], p[1] + r * n[1])
        turn = 1.0 if r > 0 else -1.0
        # The arc sweeps twice the angle between the tangent and the chord;
        # the via-point is half way round it.
        half = math.atan2(abs(side), d[0] * t[0] + d[1] * t[1])
        c, s = math.cos(turn * half), math.sin(turn * half)
        rx, ry = p[0] - centre[0], p[1] - centre[1]
        via = (centre[0] + rx * c - ry * s, centre[1] + rx * s + ry * c)
        ex, ey = end[0] - centre[0], end[1] - centre[1]
        radius = abs(r)
        leaving = (-turn * ey / radius, turn * ex / radius)
        return replace(
            self,
            segments=(*self.segments, Segment.arc(end, via)),
            cursor=end,
            tangent=leaving,
        )

    def close(self) -> Sketch:
        """The sketch of the path closed by a straight segment back to its
        start (none when it already ends there). Raises `SketchError` for a
        path with no segment.

        ```python
        from arris.build import Workplane

        triangle = Workplane.XY().start((0, 0)).line_to((4, 0)).line_to((0, 3)).close()
        assert triangle.extrude(1).model is not None
        ```
        """
        if not self.segments:
            raise SketchError(f"close(): the path from {self.start} has no segment")
        segments = self.segments
        if self.cursor != self.start:
            segments = (*segments, Segment.line(self.start))
        return Sketch(self.plane, Loop.path(self.start, list(segments)))


@dataclass(frozen=True, eq=False)
class Sketch:
    """A closed outline on a `plane`, with the `holes` cut out of it: a
    value drawn in the plane's coordinates and made a `Profile` only on its
    way to an operation (`profile`).

    ```python
    from arris.build import Workplane

    ring = Workplane.XY().circle(5).cut(Workplane.XY().circle(3)).extrude(2)
    assert len(ring.faces()) == 4
    ```
    """

    plane: Workplane
    outer: Loop
    holes: tuple[Loop, ...] = ()

    def cut(self, *inner: Sketch) -> Sketch:
        """This sketch with the outlines of `inner` cut out of it, which must
        lie on the same plane and inside this outline; raises `SketchError`
        for a sketch of another plane, and a hole that crosses the outline is
        the kernel's refusal when the sketch is made a solid.

        ```python
        from arris.build import Workplane

        plane = Workplane.XY()
        plate = plane.rect(10, 10).cut(plane.circle(1, center=(2, 2)), plane.circle(1))
        assert len(plate.holes) == 2
        ```
        """
        for sketch in inner:
            if sketch.plane != self.plane:
                raise SketchError(f"cut(): a sketch on {sketch.plane} is not on {self.plane}")
        return replace(self, holes=(*self.holes, *(sketch.outer for sketch in inner)))

    def moved_to(self, plane: Workplane) -> Sketch:
        """This outline and its holes drawn on `plane` instead, same
        coordinates.

        ```python
        from arris.build import Workplane

        up = Workplane.XY().rect(2, 2).moved_to(Workplane.XY(5))
        assert up.plane.origin == (0.0, 0.0, 5.0)
        ```
        """
        return replace(self, plane=plane)

    def profile(self) -> Profile:
        """The binding's `Profile`: this outline and its holes, placed on
        the plane.

        ```python
        from arris.build import Workplane

        profile = Workplane.XY(3).rect(2, 2).profile()
        assert len(profile.holes) == 0
        ```
        """
        return self._placed(0.0)

    def extrude(self, distance: float, both: bool = False, *, model: Model | None = None) -> Solid:
        """The sketch pushed `distance` along the plane's normal (against it
        when negative); with `both`, `distance` to each side of the plane.

        ```python
        from arris.build import Workplane

        post = Workplane.XY().circle(1).extrude(4, both=True)
        assert post.model.bounds(post.body)[0][2] < -3.9
        ```
        """
        model = model if model is not None else default_model()
        z = self.plane.z
        if both:
            reach = abs(distance)
            return Solid.of(model.extrude(self._placed(-reach), z, 2 * reach))
        direction = z if distance >= 0 else scale(z, -1.0)
        return Solid.of(model.extrude(self._placed(0.0), direction, abs(distance)))

    def revolve(
        self,
        axis: Literal["x", "y"] | tuple[Sequence[float], Sequence[float]],
        angle: float,
        *,
        model: Model | None = None,
    ) -> Solid:
        """The sketch turned `angle` radians about `axis`: `"x"` or `"y"`
        for the plane's own axes, or two points of the plane (in its
        coordinates) the axis runs through.

        ```python
        import math
        from arris.build import Workplane

        ring = Workplane.XZ().rect(2, 2, center=(5, 0)).revolve("y", 2 * math.pi)
        volume = ring.model.mass_properties(ring.body).volume
        assert abs(volume - 4 * 2 * math.pi * 5) < 1e-9
        ```
        """
        model = model if model is not None else default_model()
        if axis == "x":
            through, onward = (0.0, 0.0), (1.0, 0.0)
        elif axis == "y":
            through, onward = (0.0, 0.0), (0.0, 1.0)
        elif isinstance(axis, str):
            raise SketchError(f"revolve(): axis {axis!r} is not 'x', 'y' or two points")
        else:
            through, onward = (axis[0][0], axis[0][1]), (axis[1][0], axis[1][1])
        origin = self.plane.point(*through)
        direction = self.plane.direction(onward[0] - through[0], onward[1] - through[1])
        try:
            unit(direction)
        except ValueError as error:
            raise SketchError(f"revolve(): the axis {axis!r} has no length") from error
        return Solid.of(model.revolve(self._placed(0.0), origin, direction, angle))

    def sweep(self, path: Path, law: FrameLaw | None = None, *, model: Model | None = None) -> Solid:
        """The sketch swept along `path` (the binding's `Path`) under the
        frame `law`. The plane should sit at the path's start, its normal
        along the path there.

        ```python
        import arris
        from arris.build import Workplane

        spine = arris.Path((0, 0, 0), [arris.PathSegment.line((0, 0, 10))])
        pipe = Workplane.XY().circle(1).sweep(spine)
        assert pipe.model.mass_properties(pipe.body).volume > 31
        ```
        """
        model = model if model is not None else default_model()
        return Solid.of(model.sweep(self._placed(0.0), path, law))

    @staticmethod
    def loft(
        sections: Sequence[Sketch],
        kind: str = "ruled",
        closed: bool = False,
        *,
        model: Model | None = None,
    ) -> Solid:
        """The solid through `sections`, each on its own plane, in order
        (`Model.loft`: `kind` is `"ruled"` or `"smooth"`, `closed` joins
        the last to the first).

        ```python
        from arris.build import Sketch, Workplane

        base = Workplane.XY().rect(4, 4)
        top = Workplane.XY(5).rect(2, 2)
        frustum = Sketch.loft([base, top])
        assert abs(frustum.model.mass_properties(frustum.body).volume - 5 / 3 * (16 + 8 + 4)) < 1e-9
        ```
        """
        model = model if model is not None else default_model()
        return Solid.of(model.loft([s._placed(0.0) for s in sections], kind, closed))

    def _placed(self, along: float) -> Profile:
        """The profile on this plane moved `along` its normal."""
        plane = self.plane.offset(along) if along else self.plane
        return Profile(
            self.outer,
            list(self.holes),
            origin=plane.origin,
            normal=plane.z,
            x_axis=plane.x,
        )
