"""`arris.build.Solid`: values, provenance and tags (plans/modelling-layer
step 2).

The bracket rebuilt through `Solid.tag` carries the labels
`examples/bracket.py`'s `Labelled` gives each triangle; tags survive a
random sequence of cuts and fuses on boxes, each face on a plane of the
box its tag names; a face the operation deleted drops its tag.
"""

import importlib.util
import random
from pathlib import Path

import arris
import pytest
from arris.build import Solid

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"

# Distance of a face's plane from the plane of the box face it came from:
# boxes are exact, so the model's default tolerance bounds it.
PLANE_TOL = 1e-7  # tol-literal: the model's default tolerance (Precision)


def load_bracket():
    spec = importlib.util.spec_from_file_location("bracket", EXAMPLES / "bracket.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def untagged(solid):
    carried = set().union(*solid.tags.values())
    return [face for face in solid.faces() if face not in carried]


def finish(solid, name):
    """`solid` with every face no tag reached tagged `name`: the bracket's
    default label for a face its operation made from nothing tagged."""
    return solid.tag(name, untagged(solid))


def edges_between(solid, a, b):
    found = []
    for edge in solid.model.edges(solid.body):
        faces = solid.model.faces_of(solid.body, edge)
        if len(faces) == 2 and faces[0] != faces[1]:
            names = sorted(one_tag(solid, face) for face in faces)
            if names == sorted([a, b]):
                found.append(edge)
    return found


def one_tag(solid, face):
    [name] = [name for name, faces in solid.tags.items() if face in faces]
    return name


def tool(model, name, origin, axis, radius, height):
    cylinder = Solid.cylinder(origin, axis, radius, height, model=model)
    return cylinder.tag(name, cylinder.faces())


def bracket_on_the_layer(model):
    """`examples/bracket.py`'s `build`, written with `Solid` and tags."""
    profile = arris.Profile(
        arris.Loop.polygon([(0, 0), (60, 0), (60, 8), (8, 8), (8, 50), (0, 50)]),
        origin=(0, 0, 0),
        normal=(0, -1, 0),
        x_axis=(1, 0, 0),
    )
    plate = Solid.of(model.extrude(profile, (0, 1, 0), 40))
    part = plate.tag("plate", plate.faces())

    part = finish(part.fuse(tool(model, "boss", (8, 20, 32), (1, 0, 0), 14, 12)), "boss")
    tools = [tool(model, "bore", (-1, 20, 32), (1, 0, 0), 8, 23)]
    for y in (8, 32):
        tools.append(tool(model, "bolt holes", (40, y, -1), (0, 0, 1), 3.5, 10))
        tools.append(tool(model, "counterbores", (40, y, 5), (0, 0, 1), 6, 4))
    part = finish(part.cut(*tools), "bolt holes")

    [bend] = [
        e
        for e in part.provenance.generated_from(arris.Role("extrude", "Rise", 0, 3))
        if isinstance(e, arris.Edge)
    ]
    part = finish(part.apply(model.fillet(part.body, [bend], 4)), "bend fillet")
    rims = edges_between(part, "plate", "counterbores")
    part = finish(part.apply(model.fillet(part.body, rims, 0.8)), "rim fillets")
    [lip] = edges_between(part, "boss", "boss")
    return finish(part.apply(model.chamfer(part.body, [lip], 1.5)), "chamfer")


def triangle_labels(model, body, label_of, chord=0.02):
    mesh = model.tessellate(body, chord)
    labels = [None] * mesh.n_triangles
    for face, start, stop in mesh.faces:
        labels[start:stop] = [label_of(face)] * (stop - start)
    return mesh.positions, labels


def test_the_bracket_rebuilt_with_tags_labels_every_triangle_as_labelled_does():
    raw = load_bracket().build(arris.Model())
    layered = bracket_on_the_layer(arris.Model())
    assert untagged(layered) == []
    for face in layered.faces():
        one_tag(layered, face)  # exactly one tag per face, as one label
    positions, want = triangle_labels(raw.model, raw.body, raw.labels.__getitem__)
    got_positions, got = triangle_labels(
        layered.model, layered.body, lambda face: one_tag(layered, face)
    )
    assert got_positions == positions
    assert got == want


def test_the_provenance_chains_every_operation_and_last_is_the_final_record():
    model = arris.Model()
    block = Solid.box((0, 0, 0), (10, 10, 10), model=model)
    once = block.cut(Solid.box((5, 5, 5), (15, 15, 15), model=model))
    twice = once.fuse(Solid.box((-2, 0, 0), (1, 10, 10), model=model))
    assert twice.last.modified_from(once.body) == [twice.body]
    # The chain starts at the box's own record, whose inputs are its roles.
    assert twice.provenance.generated_from(arris.Role("box", "Body")) == [twice.body]
    assert twice.provenance.modified_from(block.body) == []
    assert block.provenance == block.last


def test_a_solid_is_a_value_its_operations_leave_it_as_it_was():
    model = arris.Model()
    block = Solid.box((0, 0, 0), (10, 10, 10), model=model)
    block = block.tag("skin", block.faces())
    before = (block.body, block.last, dict(block.tags))
    block.cut(Solid.box((5, 5, 5), (15, 15, 15), model=model)).tag("other", [])
    assert (block.body, block.last, dict(block.tags)) == before
    with pytest.raises(TypeError):
        block.tags["new"] = frozenset()


def test_mixing_models_raises_foreign_handle_error():
    a = Solid.box((0, 0, 0), (1, 1, 1), model=arris.Model())
    b = Solid.box((0, 0, 0), (2, 2, 2), model=arris.Model())
    with pytest.raises(arris.ForeignHandleError):
        a.cut(b)


def test_tagging_a_face_of_another_body_names_it():
    model = arris.Model()
    a = Solid.box((0, 0, 0), (1, 1, 1), model=model)
    b = Solid.box((2, 0, 0), (3, 1, 1), model=model)
    with pytest.raises(ValueError, match="is not a face of"):
        a.tag("x", b.faces().extreme((1, 0, 0)))
    with pytest.raises(KeyError, match="no tag 'x'"):
        a.tagged("x")


def test_a_face_the_operation_deleted_drops_its_tag():
    model = arris.Model()
    block = Solid.box((0, 0, 0), (10, 10, 10), model=model)
    [top] = [f for f in block.faces() if model.surface(f).normal == pytest.approx((0, 0, 1))]
    block = block.tag("top", [top]).tag("skin", block.faces())
    lower = block.cut(Solid.box((-1, -1, 6), (11, 11, 11), model=model))
    assert lower.last.is_deleted(top)
    assert lower.tagged("top") == frozenset()
    # The new top was made from the tool's face, which carried no tag.
    [new_top] = [f for f in lower.faces() if model.surface(f).normal == pytest.approx((0, 0, 1))]
    assert untagged(lower) == [new_top]
    assert lower.tagged("skin") == frozenset(lower.faces()) - {new_top}


def planes(lo, hi):
    """The six planes of the box `lo`–`hi`, as (axis, coordinate)."""
    return {(axis, c) for axis in range(3) for c in (lo[axis], hi[axis])}


def on_a_plane_of(model, face, box_planes):
    info = model.surface(face)
    assert info.kind == "plane"
    [axis] = [i for i in range(3) if abs(info.normal[i]) > 0.5]
    return any(
        a == axis and abs(info.origin[axis] - c) <= PLANE_TOL for a, c in box_planes
    )


def corner_box(rng, lo, hi):
    """A box from a random point inside `lo`–`hi` out past a random corner:
    it overlaps the start box and never splits it in two."""
    inner = [rng.uniform(lo[i] + 1, hi[i] - 1) for i in range(3)]
    corner = [rng.choice((lo[i] - rng.uniform(1, 3), hi[i] + rng.uniform(1, 3))) for i in range(3)]
    return (
        tuple(min(inner[i], corner[i]) for i in range(3)),
        tuple(max(inner[i], corner[i]) for i in range(3)),
    )


@pytest.mark.parametrize("seed", range(16))
def test_tags_survive_random_cuts_and_fuses_on_boxes(seed):
    rng = random.Random(seed)
    model = arris.Model()
    lo, hi = (0.0, 0.0, 0.0), (10.0, 8.0, 6.0)
    solid = Solid.box(lo, hi, model=model)
    solid = solid.tag("start", solid.faces())
    boxes = {"start": planes(lo, hi)}
    for step in range(4):
        name = f"tool {step}"
        tlo, thi = corner_box(rng, lo, hi)
        tool_solid = Solid.box(tlo, thi, model=model)
        tool_solid = tool_solid.tag(name, tool_solid.faces())
        boxes[name] = planes(tlo, thi)
        before = solid
        solid = solid.cut(tool_solid) if rng.random() < 0.5 else solid.fuse(tool_solid)
        assert model.check(solid.body, "full")
        faces = set(solid.faces())
        deleted = {f for f in solid.last.deleted if isinstance(f, arris.Face)}
        for tag, tagged in solid.tags.items():
            assert tagged <= faces, (seed, step, tag)
            assert not tagged & deleted, (seed, step, tag)
            for face in tagged:
                assert on_a_plane_of(model, face, boxes[tag]), (seed, step, tag, face)
        # Every face came from a tagged box face: no face loses its tags.
        assert untagged(solid) == [], (seed, step)
        assert set(solid.tags) == set(before.tags) | {name}
