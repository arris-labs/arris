"""`Solid`: a body, the provenance that made it and the tags on its faces."""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from types import MappingProxyType

from .._arris import Body, Edge, Face, Model, Provenance
from ._selection import Selection

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
