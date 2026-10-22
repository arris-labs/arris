"""`Solid`: a body, the provenance that made it and the tags on its faces."""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import TYPE_CHECKING, Literal

from .._arris import Body, Edge, Face, Model, Provenance
from ._selection import Selection
from ._vec import dot, sub

if TYPE_CHECKING:
    from ._workplane import Sketch, Workplane

# How many model tolerances a through tool reaches past the solid on each
# side: far enough that no tolerance band of a face reaches it, near enough
# to leave the tool's size to the solid.
_THROUGH_MARGIN = 1000

_default: Model | None = None


def default_model() -> Model:
    """The model a constructor uses when it is given none: one per process,
    made on first use.

    ```python
    from arris.build import Solid, default_model

    assert Solid.box((0, 0, 0), (1, 1, 1)).model == default_model()
    ```
    """
    global _default
    if _default is None:
        _default = Model()
    return _default


@dataclass(frozen=True, eq=False)
class Solid:
    """A solid body as a value: its `model`, its `body` handle, the
    `provenance` of every operation since it was made (`Provenance.then`),
    the `last` operation's record, and its `tags`. `steps` keeps each
    operation's record with its tools' bodies, which `Selection.created_by`
    and `modified_by` read.

    Operations return a new `Solid` and leave this one as it was. A kernel
    refusal reaches the caller as the binding's typed exception, unchanged;
    so does `ForeignHandleError` when two solids of different models meet.

    ```python
    import arris
    from arris.build import Solid

    block = Solid.box((0, 0, 0), (10, 10, 10))
    notched = block.cut(Solid.box((5, 5, 5), (15, 15, 15)))
    assert notched.last.modified_from(block.body) == [notched.body]
    assert notched.provenance.generated_from(arris.Role("box", "Body")) == [notched.body]
    ```
    """

    model: Model
    body: Body
    provenance: Provenance
    last: Provenance
    tags: Mapping[str, frozenset[Face]]
    steps: tuple[tuple[Provenance, frozenset[Body]], ...] = field(default=(), repr=False)

    @classmethod
    def of(cls, made: tuple[Body, Provenance]) -> Solid:
        """A solid from a binding operation's `(body, record)`, untagged; its
        model is the body's.

        ```python
        import arris
        from arris.build import Solid

        model = arris.Model()
        solid = Solid.of(model.primitive_box((0, 0, 0), (2, 3, 4)))
        assert solid.model == model and solid.tags == {}
        ```
        """
        body, record = made
        return cls(body.model, body, record, record, MappingProxyType({}), ((record, frozenset()),))

    @classmethod
    def box(
        cls, min: Sequence[float], max: Sequence[float], *, model: Model | None = None
    ) -> Solid:
        """The box between the corners `min` and `max` (`Model.primitive_box`).

        ```python
        from arris.build import Solid

        assert len(Solid.box((0, 0, 0), (2, 3, 4)).faces()) == 6
        ```
        """
        model = model if model is not None else default_model()
        return cls.of(model.primitive_box(min, max))

    @classmethod
    def cylinder(
        cls,
        origin: Sequence[float],
        axis: Sequence[float],
        radius: float,
        height: float,
        *,
        model: Model | None = None,
    ) -> Solid:
        """The cylinder of `radius` from `origin` along `axis` for `height`
        (`Model.primitive_cylinder`).

        ```python
        from arris.build import Solid

        assert len(Solid.cylinder((0, 0, 0), (0, 0, 1), 1, 2).faces()) == 3
        ```
        """
        model = model if model is not None else default_model()
        return cls.of(model.primitive_cylinder(origin, axis, radius, height))

    def faces(self) -> Selection[Face]:
        """Every face of the body, as a `Selection` to narrow.

        ```python
        from arris.build import Solid

        box = Solid.box((0, 0, 0), (1, 1, 1))
        assert list(box.faces()) == box.model.faces(box.body)
        ```
        """
        return Selection.of(self, self.model.faces(self.body), "faces()")

    def edges(self) -> Selection[Edge]:
        """Every edge of the body, as a `Selection` to narrow.

        ```python
        from arris.build import Solid

        assert len(Solid.box((0, 0, 0), (1, 1, 1)).edges()) == 12
        ```
        """
        return Selection.of(self, self.model.edges(self.body), "edges()")

    def tag(self, name: str, faces: Iterable[Face]) -> Solid:
        """This solid with `faces` added to the tag `name`. A tag follows
        each later operation's provenance (`apply`); raises `ValueError`
        naming a face that is not this body's.

        ```python
        from arris.build import Solid

        box = Solid.box((0, 0, 0), (1, 1, 1))
        tagged = box.tag("skin", box.faces())
        assert tagged.tagged("skin") == frozenset(box.faces()) and box.tags == {}
        ```
        """
        added = frozenset(faces)
        stray = sorted(added - set(self.faces()), key=lambda face: face.index)
        if stray:
            raise ValueError(f"tag {name!r}: {stray[0]!r} is not a face of {self.body!r}")
        tags = dict(self.tags)
        tags[name] = tags.get(name, frozenset()) | added
        return Solid(
            self.model, self.body, self.provenance, self.last, MappingProxyType(tags), self.steps
        )

    def tagged(self, name: str) -> frozenset[Face]:
        """The faces carrying the tag `name`, empty once an operation deleted
        them all; raises `KeyError` for a name never tagged.

        ```python
        from arris.build import Solid

        box = Solid.box((0, 0, 0), (1, 1, 1))
        assert box.tag("all", box.faces()).tagged("all") == frozenset(box.faces())
        ```
        """
        if name not in self.tags:
            raise KeyError(f"no tag {name!r} on {self.body!r}; tags: {sorted(self.tags)}")
        return self.tags[name]

    def apply(self, made: tuple[Body, Provenance], tools: Sequence[Solid] = ()) -> Solid:
        """The solid a binding operation made from this one (and `tools`),
        for an operation the layer does not wrap: its provenance chained on,
        and its tags followed through the record.

        A face of the result carries a tag when a face it was modified from
        carried it; a face modified from no face, when a face it was
        generated from did; a face the record leaves alone keeps its own.
        Tags on `tools` come along; a face the operation deleted drops its
        tags.

        ```python
        from arris.build import Solid

        box = Solid.box((0, 0, 0), (4, 4, 4))
        box = box.tag("skin", box.faces())
        top = list(box.faces().extreme((0, 0, 1)))
        grown = box.apply(box.model.offset_faces(box.body, top, 1.0))
        assert grown.tagged("skin") == frozenset(grown.faces())
        ```
        """
        body, record = made
        return Solid(
            self.model,
            body,
            self.provenance.then(record),
            record,
            MappingProxyType(_follow([self, *tools], record, self.model.faces(body))),
            (*self.steps, (record, frozenset(tool.body for tool in tools))),
        )

    def cut(self, *tools: Solid) -> Solid:
        """This solid less `tools`: `Model.cut` for one, `Model.cut_many` for
        several.

        ```python
        from arris.build import Solid

        plate = Solid.box((0, 0, 0), (10, 10, 2))
        holes = [Solid.cylinder((x, 5, -1), (0, 0, 1), 1, 4) for x in (3, 7)]
        assert len(plate.cut(*holes).faces()) == 8
        ```
        """
        bodies = [tool.body for tool in tools]
        if len(bodies) == 1:
            return self.apply(self.model.cut(self.body, bodies[0]), tools)
        return self.apply(self.model.cut_many(self.body, bodies), tools)

    def fuse(self, *others: Solid) -> Solid:
        """This solid joined with `others`: `Model.fuse` for one,
        `Model.fuse_many` for several.

        ```python
        from arris.build import Solid

        base = Solid.box((0, 0, 0), (10, 10, 2))
        post = Solid.cylinder((5, 5, 2), (0, 0, 1), 1, 3)
        assert len(base.fuse(post).faces()) == 8
        ```
        """
        if len(others) == 1:
            return self.apply(self.model.fuse(self.body, others[0].body), others)
        bodies = [self.body, *(other.body for other in others)]
        return self.apply(self.model.fuse_many(bodies), others)

    def common(self, other: Solid) -> Solid:
        """The part of this solid inside `other` (`Model.common`).

        ```python
        from arris.build import Solid

        both = Solid.box((0, 0, 0), (2, 2, 2)).common(Solid.box((1, 1, 1), (3, 3, 3)))
        assert abs(both.model.mass_properties(both.body).volume - 1.0) < 1e-9
        ```
        """
        return self.apply(self.model.common(self.body, other.body), (other,))
    def hole(
        self,
        at: Iterable[Workplane],
        diameter: float,
        depth: float | Literal["through"] = "through",
    ) -> Solid:
        """A round hole of `diameter` at each plane of `at`, from the plane
        into the solid (against its normal) `depth` deep, or all the way
        through. The holes are cut in one operation.

        ```python
        from arris.build import Solid, Workplane

        plate = Solid.box((0, 0, 0), (20, 10, 4))
        drilled = plate.hole(Workplane.XY(4).at([(5, 5), (15, 5)]), 3)
        assert len(drilled.faces()) == 6 + 2
        ```
        """
        return self.pocket([plane.circle(diameter / 2) for plane in at], depth)

    def pocket(
        self,
        sketches: Sketch | Iterable[Sketch],
        depth: float | Literal["through"] = "through",
    ) -> Solid:
        """Each sketch cut from its plane into the solid (against the
        plane's normal) `depth` deep, or all the way through; one operation
        for all. A through tool is sized from the solid's `Model.bounds`
        along each plane's normal, past it by a margin of the model's
        tolerance, so the sketch can sit on a face, inside the solid or
        clear of it. A depth tool starts the same margin above its plane.

        ```python
        from arris.build import Solid, Workplane

        block = Solid.box((0, 0, 0), (10, 10, 6))
        slotted = block.pocket(Workplane.XY(6).slot(8, 2, center=(5, 5)), depth=2)
        assert abs(slotted.model.mass_properties(slotted.body).volume - (600 - 2 * (6 * 2 + 3.141592653589793))) < 1e-9
        ```
        """
        tools = [self._tool(sketch, depth) for sketch in _each(sketches)]
        return self.cut(*tools) if tools else self

    def boss(self, sketches: Sketch | Iterable[Sketch], height: float) -> Solid:
        """Each sketch raised `height` along its plane's normal and fused
        to the solid, in one operation.

        ```python
        from arris.build import Solid, Workplane

        base = Solid.box((0, 0, 0), (10, 10, 2))
        posts = base.boss(Workplane.XY(2).grid(2, 1, 4, 0)[0].circle(1), 3)
        assert abs(posts.model.bounds(posts.body)[1][2] - 5.0) < 1e-6
        ```
        """
        raised = [sketch.extrude(height, model=self.model) for sketch in _each(sketches)]
        return self.fuse(*raised) if raised else self

    def _tool(self, sketch: Sketch, depth: float | Literal["through"]) -> Solid:
        margin = self.model.tolerance * _THROUGH_MARGIN
        plane = sketch.plane
        if depth == "through":
            lo, hi = self._span(plane)
            top = sketch.moved_to(plane.offset(hi + margin))
            return top.extrude(-(hi - lo + 2 * margin), model=self.model)
        raised = sketch.moved_to(plane.offset(margin))
        return raised.extrude(-(depth + margin), model=self.model)

    def _span(self, plane: Workplane) -> tuple[float, float]:
        """How far this solid reaches along `plane`'s normal from the plane:
        the lowest and the highest signed distance of its box's corners."""
        box = self.model.bounds(self.body)
        if box is None:
            raise ValueError(f"{self.body!r} has no bounds to size a through tool from")
        lo, hi = box
        along = [
            dot(
                sub((x, y, z), plane.origin),
                plane.z,
            )
            for x in (lo[0], hi[0])
            for y in (lo[1], hi[1])
            for z in (lo[2], hi[2])
        ]
        return min(along), max(along)


def _each(sketches: Sketch | Iterable[Sketch]) -> list[Sketch]:
    """One sketch or several, as a list."""
    from ._workplane import Sketch

    return [sketches] if isinstance(sketches, Sketch) else list(sketches)


def _follow(
    inputs: Sequence[Solid], record: Provenance, faces: Sequence[Face]
) -> dict[str, frozenset[Face]]:
    """The tags of `inputs` carried onto `faces` through `record` (see
    `Solid.apply`)."""
    output = set(faces)
    modified: dict[Face, list[Face]] = {}
    generated: dict[Face, list[Face]] = {}
    for relation, into in ((record.modified, modified), (record.generated, generated)):
        for origin, outputs in relation:
            if isinstance(origin, Face):
                for made in outputs:
                    if isinstance(made, Face) and made in output:
                        into.setdefault(made, []).append(origin)
    names: dict[str, frozenset[Face]] = {}
    for solid in inputs:
        for name, tagged in solid.tags.items():
            names[name] = names.get(name, frozenset()) | tagged
    tags: dict[str, frozenset[Face]] = {}
    for name, carried in names.items():
        kept: set[Face] = set()
        for face in faces:
            origins = modified.get(face) or generated.get(face)
            if origins is None:
                if face in carried:
                    kept.add(face)
            elif any(origin in carried for origin in origins):
                kept.add(face)
        tags[name] = frozenset(kept)
    return tags
