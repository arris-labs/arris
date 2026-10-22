"""`Selection`: some of a solid's faces or edges, picked by provenance first,
then by geometry, in the kernel's id order."""

from __future__ import annotations

import math
from collections.abc import Callable, Iterable, Iterator, Sequence
from dataclasses import dataclass
from typing import TYPE_CHECKING, Generic, TypeVar

from .._arris import ArrisError, Body, Edge, Face, Model, Provenance, Role, Shell, Vertex
from ._vec import Point
from ._vec import add as _add
from ._vec import dot as _dot
from ._vec import fmt as _fmt
from ._vec import scale as _scale
from ._vec import sub as _sub
from ._vec import unit as _unit

if TYPE_CHECKING:
    from ._solid import Solid
    from ._workplane import Workplane

M = TypeVar("M", Face, Edge)

_Point = Point
_Handle = Body | Shell | Face | Edge | Vertex

# Picks a step record's outputs, given the tool's own entities.
_Outputs = Callable[[Provenance, set[_Handle]], set[_Handle]]

# How many model tolerances `within` grows its box by. `Model.bounds` grows
# a face's box by the face's tolerance and unions its edges', each grown by
# theirs: a face lying on the box's side overshoots it by two tolerances at
# the default precision (measured on a box), so four leave a margin.
_BOX_SLACK = 4


class SelectionError(ArrisError):
    """`Selection.one()` found no member or several: the message names the
    selection's chain and its count, also kept as `chain` and `count`.

    ```python
    from arris.build import Solid, SelectionError

    box = Solid.box((0, 0, 0), (1, 1, 1))
    try:
        box.faces().kind("plane").one()
    except SelectionError as error:
        assert error.count == 6 and error.chain == "faces().kind('plane')"
    else:
        raise AssertionError("six faces are not one")
    ```
    """

    def __init__(self, chain: str, count: int) -> None:
        super().__init__(f"{chain} selected {count}, one() needs exactly one")
        self.chain = chain
        self.count = count


@dataclass(frozen=True, eq=False)
class Selection(Generic[M]):
    """Faces or edges of `solid`, as a set: its `members` in ascending handle
    index, the kernel's deterministic id order, and the `chain` of calls
    that picked them, which an error names.

    Pick by provenance first (`created_by`, `modified_by`, `from_role`,
    `tagged`, `last`), then by geometry (`kind`, `normal`, `parallel`,
    `perpendicular`, `extreme`, `within`); combine with `|`, `&` and `-`.
    There is no indexing: `one()` is the single member or a
    `SelectionError`.

    ```python
    from arris.build import Solid

    plate = Solid.box((0, 0, 0), (10, 10, 2))
    holed = plate.cut(Solid.cylinder((5, 5, -1), (0, 0, 1), 1, 4))
    top = holed.faces().extreme((0, 0, 1)).one()
    rims = holed.faces().kind("cylinder").edges() & holed.faces().extreme((0, 0, 1)).edges()
    assert len(rims) == 1 and top in holed.faces()
    ```
    """

    solid: Solid
    members: tuple[M, ...]
    chain: str

    @classmethod
    def of(cls, solid: Solid, members: Iterable[M], chain: str) -> Selection[M]:
        """The selection of `members` of `solid`, sorted into handle order.

        ```python
        from arris.build import Selection, Solid

        box = Solid.box((0, 0, 0), (1, 1, 1))
        faces = box.model.faces(box.body)
        picked = Selection.of(box, reversed(faces[:2]), "first two")
        assert picked.members == tuple(faces[:2])
        ```
        """
        return cls(solid, tuple(sorted(set(members), key=_order)), chain)

    def __iter__(self) -> Iterator[M]:
        return iter(self.members)

    def __len__(self) -> int:
        return len(self.members)

    def __contains__(self, member: object) -> bool:
        return member in self.members

    def __repr__(self) -> str:
        return f"<Selection {self.chain}: {len(self.members)}>"

    def one(self) -> M:
        """The single member; raises `SelectionError` naming the chain and
        the count otherwise.

        ```python
        from arris.build import Solid

        box = Solid.box((0, 0, 0), (1, 1, 1))
        assert box.faces().extreme((1, 0, 0)).one() in box.faces()
        ```
        """
        if len(self.members) != 1:
            raise SelectionError(self.chain, len(self.members))
        return self.members[0]

    def workplane(self) -> Workplane:
        """The plane of the one planar face selected, outward: its origin
        on the face, `z` its normal. Raises `SelectionError` unless the
        selection is one face, and `SketchError` for a face that is not
        planar.

        ```python
        from arris.build import Solid

        block = Solid.box((0, 0, 0), (4, 4, 4))
        top = block.faces().extreme((0, 0, 1)).workplane()
        assert top.z == (0.0, 0.0, 1.0) and top.origin[2] == 4.0
        ```
        """
        from ._workplane import SketchError, Workplane

        face = self._faces_only("workplane").one()
        model = self.solid.model
        kind = model.surface(face).kind
        if kind != "plane":
            raise SketchError(f"{self.chain}.workplane(): {face!r} is a {kind}, not a plane")
        return Workplane.from_frame(model.face_frame(face))

    # Provenance.

    def created_by(self, tool: Solid) -> Selection[M]:
        """The members an operation with `tool` among its tools made from
        `tool`'s entities (a cut's walls, a fuse's new faces), followed
        through every later operation.

        ```python
        from arris.build import Solid

        plate = Solid.box((0, 0, 0), (10, 10, 2))
        drill = Solid.cylinder((5, 5, -1), (0, 0, 1), 1, 4)
        wall = plate.cut(drill).faces().created_by(drill).one()
        assert plate.model.surface(wall).kind == "cylinder"
        ```
        """
        made = self._through(tool, lambda record, own: _outputs(record.generated, own, True))
        made |= self._through(tool, lambda record, own: _outputs(record.modified, own, True))
        return self._keep(made, f"created_by({tool.body!r})")

    def modified_by(self, tool: Solid) -> Selection[M]:
        """The members an operation with `tool` among its tools modified
        from entities not `tool`'s (a face a hole now pierces), followed
        through every later operation. A record names no tool per output,
        so with several tools in one operation this is what any of them
        modified.

        ```python
        from arris.build import Solid

        plate = Solid.box((0, 0, 0), (10, 10, 2))
        drill = Solid.cylinder((5, 5, -1), (0, 0, 1), 1, 4)
        pierced = plate.cut(drill).faces().modified_by(drill)
        assert len(pierced) == 2
        ```
        """
        return self._keep(
            self._through(tool, lambda record, own: _outputs(record.modified, own, False)),
            f"modified_by({tool.body!r})",
        )

    def from_role(self, role: Role) -> Selection[M]:
        """The members the solid's provenance made from `role`, a
        constructor's named part (`Role("box", "Face", "Z", "Max")`).

        ```python
        import arris
        from arris.build import Solid

        box = Solid.box((0, 0, 0), (1, 1, 1))
        top = box.faces().from_role(arris.Role("box", "Face", "Z", "Max")).one()
        assert box.model.surface(top).normal == (0.0, 0.0, 1.0)
        ```
        """
        chain = self.solid.provenance
        made = set(chain.generated_from(role)) | set(chain.modified_from(role))
        return self._keep(made, f"from_role({role!r})")

    def tagged(self, name: str) -> Selection[M]:
        """The members carrying the tag `name` (`Solid.tag`); a tag holds
        faces, so an edge selection keeps none. Raises `KeyError` for a name
        never tagged.

        ```python
        from arris.build import Solid

        box = Solid.box((0, 0, 0), (1, 1, 1))
        box = box.tag("floor", box.faces().extreme((0, 0, -1)))
        assert len(box.faces().tagged("floor")) == 1
        ```
        """
        return self._keep(self.solid.tagged(name), f"tagged({name!r})")

    def last(self) -> Selection[M]:
        """The members the solid's last operation made or modified.

        ```python
        from arris.build import Solid

        plate = Solid.box((0, 0, 0), (10, 10, 2))
        holed = plate.cut(Solid.cylinder((5, 5, -1), (0, 0, 1), 1, 4))
        assert len(holed.faces().last()) == 3  # the wall, the top, the bottom
        ```
        """
        final: Provenance = self.solid.last
        made: set[_Handle] = set()
        for _, outs in (*final.generated, *final.modified):
            made.update(outs)
        return self._keep(made, "last()")

    # Geometry.

    def kind(self, *kinds: str) -> Selection[M]:
        """The members whose geometry is one of `kinds`: a surface's
        (`SurfaceInfo.kind`) for faces, a curve's (`CurveInfo.kind`) for
        edges.

        ```python
        from arris.build import Solid

        rod = Solid.cylinder((0, 0, 0), (0, 0, 1), 1, 5)
        assert len(rod.faces().kind("plane")) == 2
        assert len(rod.edges().kind("circle", "line")) == 3
        ```
        """
        model = self.solid.model
        picked = [m for m in self.members if _kind(model, m) in kinds]
        return self._keep(picked, f"kind({', '.join(map(repr, kinds))})")

    def normal(self, direction: Sequence[float]) -> Selection[Face]:
        """The planar faces whose outward normal points along `direction`,
        within the model's angular tolerance. Raises `TypeError` on edges.

        ```python
        from arris.build import Solid

        box = Solid.box((0, 0, 0), (1, 1, 1))
        assert len(box.faces().normal((0, 0, -1))) == 1
        ```
        """
        d = _unit(direction)
        faces = self._faces_only("normal")
        model = self.solid.model
        picked = []
        for face in faces.members:
            info = model.surface(face)
            if info.kind == "plane" and info.normal is not None:
                if _along(info.normal, d, model) and _dot(info.normal, d) > 0:
                    picked.append(face)
        return faces._keep(picked, f"normal({_fmt(direction)})")

    def parallel(self, direction: Sequence[float]) -> Selection[M]:
        """The members `direction` lies along at every point: a plane with
        its normal across it or a cylinder with its axis along it; a line
        along it. Either sense; within the model's angular tolerance.

        ```python
        from arris.build import Solid

        rod = Solid.cylinder((0, 0, 0), (0, 0, 1), 1, 5)
        assert rod.model.surface(rod.faces().parallel((0, 0, 1)).one()).kind == "cylinder"
        assert len(rod.edges().parallel((0, 0, -1))) == 1  # the seam
        ```
        """
        d = _unit(direction)
        model = self.solid.model
        picked = []
        for member in self.members:
            if isinstance(member, Face):
                info = model.surface(member)
                if info.kind == "plane" and info.normal is not None:
                    hit = _across(info.normal, d, model)
                elif info.kind in ("cylinder", "elliptic_cylinder") and info.axis is not None:
                    hit = _along(info.axis, d, model)
                else:
                    hit = False
            else:
                curve = model.curve(member)
                axis = curve.axis
                hit = curve.kind == "line" and axis is not None and _along(axis, d, model)
            if hit:
                picked.append(member)
        return self._keep(picked, f"parallel({_fmt(direction)})")

    def perpendicular(self, direction: Sequence[float]) -> Selection[M]:
        """The members every tangent of which is square to `direction`: a
        plane with its normal along it; a line across it, or a circle or an
        ellipse whose axis is along it. Either sense; within the model's
        angular tolerance.

        ```python
        from arris.build import Solid

        rod = Solid.cylinder((0, 0, 0), (0, 0, 1), 1, 5)
        assert len(rod.faces().perpendicular((0, 0, 1))) == 2  # the caps
        assert len(rod.edges().perpendicular((0, 0, 1))) == 2  # their circles
        ```
        """
        d = _unit(direction)
        model = self.solid.model
        picked = []
        for member in self.members:
            if isinstance(member, Face):
                info = model.surface(member)
                normal = info.normal
                hit = info.kind == "plane" and normal is not None and _along(normal, d, model)
            else:
                curve = model.curve(member)
                if curve.axis is None:
                    hit = False
                elif curve.kind == "line":
                    hit = _across(curve.axis, d, model)
                else:
                    hit = curve.kind in ("circle", "ellipse") and _along(curve.axis, d, model)
            if hit:
                picked.append(member)
        return self._keep(picked, f"perpendicular({_fmt(direction)})")

    def extreme(self, direction: Sequence[float]) -> Selection[M]:
        """The members whose centre lies furthest along `direction`, every
        one within the model's tolerance of the furthest: coplanar faces
        tie. A member's centre is its box's (`Model.bounds`), moved onto its
        plane, its line or its circle's plane where it has one, so it is
        exact for those and loose on a NURBS member.

        ```python
        from arris.build import Solid

        base = Solid.box((0, 0, 0), (10, 10, 2))
        steps = base.cut(Solid.box((4, -1, 1), (6, 11, 3)))
        assert len(steps.faces().extreme((0, 0, 1))) == 2  # both treads
        ```
        """
        d = _unit(direction)
        model = self.solid.model
        heights = [(_dot(_centre(model, m), d), m) for m in self.members]
        chain = f"extreme({_fmt(direction)})"
        if not heights:
            return self._keep((), chain)
        top = max(h for h, _ in heights)
        return self._keep((m for h, m in heights if h >= top - model.tolerance), chain)

    def within(self, box: tuple[Sequence[float], Sequence[float]]) -> Selection[M]:
        """The members whose box (`Model.bounds`) lies inside `box`, given
        as `(min, max)` and grown by a few model tolerances, so a member on
        its side is inside; a member with a looser tolerance of its own may
        need a box grown by it.

        ```python
        from arris.build import Solid

        box = Solid.box((0, 0, 0), (2, 2, 2))
        assert len(box.edges().within(((0, 0, 0), (2, 2, 0)))) == 4
        ```
        """
        lo, hi = box
        model = self.solid.model
        tol = _BOX_SLACK * model.tolerance
        picked = []
        for member in self.members:
            corners = _bounds(model, member)
            if all(lo[i] - tol <= corners[0][i] and corners[1][i] <= hi[i] + tol for i in range(3)):
                picked.append(member)
        return self._keep(picked, f"within({_fmt(lo)}, {_fmt(hi)})")

    # Algebra and adjacency.

    def __or__(self, other: Selection[M]) -> Selection[M]:
        self._same(other, "|")
        both = sorted(set(self.members) | set(other.members), key=_order)
        return Selection(self.solid, tuple(both), f"({self.chain} | {other.chain})")

    def __and__(self, other: Selection[M]) -> Selection[M]:
        self._same(other, "&")
        return self._named(other.members, f"({self.chain} & {other.chain})")

    def __sub__(self, other: Selection[M]) -> Selection[M]:
        self._same(other, "-")
        dropped = set(other.members)
        kept = [m for m in self.members if m not in dropped]
        return Selection.of(self.solid, kept, f"({self.chain} - {other.chain})")

    def edges(self) -> Selection[Edge]:
        """The edges of the member faces; an edge selection itself.

        ```python
        from arris.build import Solid

        box = Solid.box((0, 0, 0), (1, 1, 1))
        assert len(box.faces().normal((0, 0, 1)).edges()) == 4
        ```
        """
        model, body = self.solid.model, self.solid.body
        own = {edge: edge for edge in model.edges(body)}
        found: list[Edge] = []
        for member in self.members:
            if isinstance(member, Face):
                found.extend(own[edge] for edge in model.edges_of(member) if edge in own)
            else:
                found.append(member)
        return Selection.of(self.solid, found, f"{self.chain}.edges()")

    def faces(self) -> Selection[Face]:
        """The faces meeting the member edges; a face selection itself.

        ```python
        from arris.build import Solid

        box = Solid.box((0, 0, 0), (1, 1, 1))
        assert len(box.edges().within(((0, 0, 1), (1, 0, 1))).faces()) == 2
        ```
        """
        model, body = self.solid.model, self.solid.body
        found: list[Face] = []
        for member in self.members:
            if isinstance(member, Edge):
                found.extend(model.faces_of(body, member))
            else:
                found.append(member)
        return Selection.of(self.solid, found, f"{self.chain}.faces()")

    # Helpers.

    def _keep(self, allowed: Iterable[object], step: str) -> Selection[M]:
        """The members also in `allowed`, the chain extended by `step`."""
        return self._named(allowed, f"{self.chain}.{step}")

    def _named(self, allowed: Iterable[object], chain: str) -> Selection[M]:
        """The members also in `allowed`, under `chain`."""
        kept = set(allowed)
        return Selection(self.solid, tuple(m for m in self.members if m in kept), chain)

    def _same(self, other: Selection[M], op: str) -> None:
        if self.solid.body != other.solid.body:
            raise ValueError(
                f"{self.chain} {op} {other.chain}: selections of {self.solid.body!r} "
                f"and {other.solid.body!r}"
            )

    def _faces_only(self, name: str) -> Selection[Face]:
        if any(isinstance(m, Edge) for m in self.members):
            raise TypeError(f"{self.chain}.{name}(): {name} selects faces, not edges")
        return self  # type: ignore[return-value]

    def _through(self, tool: Solid, outputs: _Outputs) -> set[_Handle]:
        """What `outputs` picks from each step's record whose tools include
        `tool`, followed through the steps after it."""
        own = _entities(tool.model, tool.body)
        steps = self.solid.steps
        found: set[_Handle] = set()
        for i, (record, tools) in enumerate(steps):
            if tool.body in tools:
                found |= _forward(outputs(record, own), [r for r, _ in steps[i + 1 :]])
        return found


def _outputs(
    relation: Sequence[tuple[_Handle | Role, list[_Handle]]], own: set[_Handle], from_own: bool
) -> set[_Handle]:
    """The outputs of `relation` whose origin is (`from_own`) or is not one
    of `own`."""
    return {out for origin, outs in relation if (origin in own) == from_own for out in outs}


def _forward(handles: set[_Handle], records: Sequence[Provenance]) -> set[_Handle]:
    """`handles` carried through `records`: an entity becomes what a record
    modified or generated from it, stays when the record leaves it alone,
    and is gone when the record deleted it."""
    for record in records:
        carried: set[_Handle] = set()
        for handle in handles:
            outs = record.modified_from(handle) + record.generated_from(handle)
            if outs:
                carried.update(outs)
            elif not record.is_deleted(handle):
                carried.add(handle)
        handles = carried
    return handles


def _entities(model: Model, body: Body) -> set[_Handle]:
    """`body` and every shell, face, edge and vertex of it."""
    return {
        body,
        *model.shells(body),
        *model.faces(body),
        *model.edges(body),
        *model.vertices(body),
    }


def _order(member: Face | Edge) -> tuple[int, int]:
    return (member.index, member.generation)


def _kind(model: Model, member: Face | Edge) -> str:
    if isinstance(member, Face):
        return model.surface(member).kind
    return model.curve(member).kind


def _bounds(model: Model, member: Face | Edge) -> tuple[_Point, _Point]:
    corners = model.bounds(member)
    if corners is None:
        raise ValueError(f"{member!r} is unbounded")
    return corners


def _centre(model: Model, member: Face | Edge) -> _Point:
    """`member`'s box centre, moved onto its plane, line or circle's plane."""
    lo, hi = _bounds(model, member)
    c = (0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), 0.5 * (lo[2] + hi[2]))
    if isinstance(member, Face):
        info = model.surface(member)
        if info.kind == "plane" and info.origin is not None and info.normal is not None:
            return _onto_plane(c, info.origin, info.normal)
        return c
    curve = model.curve(member)
    if curve.origin is None or curve.axis is None:
        return c
    if curve.kind == "line":
        t = _dot(_sub(c, curve.origin), curve.axis)
        return _add(curve.origin, _scale(curve.axis, t))
    if curve.kind in ("circle", "ellipse"):
        return _onto_plane(c, curve.origin, curve.axis)
    return c


def _onto_plane(p: _Point, origin: _Point, normal: _Point) -> _Point:
    return _sub(p, _scale(normal, _dot(_sub(p, origin), normal)))


def _along(a: _Point, d: _Point, model: Model) -> bool:
    """`a` and `d` (units) parallel, in either sense."""
    c = (a[1] * d[2] - a[2] * d[1], a[2] * d[0] - a[0] * d[2], a[0] * d[1] - a[1] * d[0])
    return math.sqrt(_dot(c, c)) <= model.angular_tolerance


def _across(a: _Point, d: _Point, model: Model) -> bool:
    """`a` and `d` (units) square to each other."""
    return abs(_dot(a, d)) <= model.angular_tolerance
