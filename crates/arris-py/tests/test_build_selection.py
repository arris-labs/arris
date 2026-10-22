"""`arris.build.Selection`: provenance and geometric filters (plans/modelling-layer
step 3).

Each filter on the primitives; the same chains pick the same handles in two
fresh models; `created_by` after a `cut_many` is exactly each tool's wall;
`extreme` returns every coplanar tie, on axes and turned off them.
"""

import math

import arris
import pytest
from arris.build import Selection, SelectionError, Solid


def box(model, lo=(0, 0, 0), hi=(10, 8, 6)):
    return Solid.box(lo, hi, model=model)


def rod(model):
    return Solid.cylinder((0, 0, 0), (0, 0, 1), 2, 5, model=model)


def normals(solid, selection):
    return sorted(solid.model.surface(face).normal for face in selection)


def test_faces_and_edges_are_every_face_and_edge_in_handle_order():
    model = arris.Model()
    solid = box(model)
    faces, edges = solid.faces(), solid.edges()
    assert isinstance(faces, Selection) and len(faces) == 6 and len(edges) == 12
    assert list(faces) == sorted(model.faces(solid.body), key=lambda f: f.index)
    assert list(edges) == sorted(model.edges(solid.body), key=lambda e: e.index)
    assert not hasattr(faces, "__getitem__")


def test_kind_on_a_cylinder():
    model = arris.Model()
    r = rod(model)
    assert len(r.faces().kind("plane")) == 2
    assert len(r.faces().kind("cylinder")) == 1
    assert len(r.faces().kind("sphere")) == 0
    assert len(r.edges().kind("circle")) == 2
    assert len(r.edges().kind("line")) == 1


def test_normal_parallel_and_perpendicular_on_a_box():
    model = arris.Model()
    b = box(model)
    assert normals(b, b.faces().normal((0, 0, 1))) == [(0.0, 0.0, 1.0)]
    assert normals(b, b.faces().normal((0, 0, -3))) == [(0.0, 0.0, -1.0)]
    assert len(b.faces().normal((1, 1, 0))) == 0
    # A plane is parallel to a direction lying in it, perpendicular to its normal.
    assert len(b.faces().parallel((0, 0, 1))) == 4
    assert normals(b, b.faces().perpendicular((0, 0, 1))) == [(0.0, 0.0, -1.0), (0.0, 0.0, 1.0)]
    assert len(b.edges().parallel((1, 0, 0))) == 4
    assert len(b.edges().perpendicular((1, 0, 0))) == 8
    with pytest.raises(TypeError, match="normal selects faces"):
        b.edges().normal((0, 0, 1))
    with pytest.raises(ValueError, match="no length"):
        b.faces().normal((0, 0, 0))


def test_parallel_and_perpendicular_on_a_cylinder():
    model = arris.Model()
    r = rod(model)
    [side] = r.faces().kind("cylinder")
    assert r.faces().parallel((0, 0, 1)).one() == side
    # The caps hold every direction across the axis; the side holds none.
    assert r.faces().parallel((1, 0, 0)).members == r.faces().kind("plane").members
    assert len(r.faces().perpendicular((0, 0, 1))) == 2
    assert r.edges().perpendicular((0, 0, 1)).members == r.edges().kind("circle").members
    assert r.edges().parallel((0, 0, 1)).members == r.edges().kind("line").members


def test_extreme_and_within_on_a_box():
    model = arris.Model()
    b = box(model)
    for axis in range(3):
        for sign in (1, -1):
            d = [0, 0, 0]
            d[axis] = sign
            [n] = normals(b, b.faces().extreme(d))
            assert n == tuple(float(c) for c in d)
    assert len(b.edges().extreme((0, 0, 1))) == 4
    # Along the diagonal of a 10 x 8 x 6 box the vertical corner edge's
    # centre (10, 8, 3) is furthest: the other two at the corner sit at
    # (5, 8, 6) and (10, 4, 6).
    corner = b.edges().extreme((1, 1, 1)).one()
    assert sorted((model.curve(corner).start, model.curve(corner).end)) == [
        (10.0, 8.0, 0.0),
        (10.0, 8.0, 6.0),
    ]
    assert len(b.edges().within(((0, 0, 0), (10, 8, 0)))) == 4
    assert len(b.faces().within(((-1, -1, -1), (11, 9, 7)))) == 6
    assert len(b.faces().within(((0, 0, 0), (10, 8, 5)))) == 1


def test_from_role_tagged_and_last():
    model = arris.Model()
    b = box(model)
    top = b.faces().from_role(arris.Role("box", "Face", "Z", "Max")).one()
    assert top == b.faces().extreme((0, 0, 1)).one()
    b = b.tag("top", [top])
    assert b.faces().tagged("top").one() == top
    assert len(b.edges().tagged("top")) == 0
    with pytest.raises(KeyError):
        b.faces().tagged("bottom")
    cut = b.cut(Solid.cylinder((5, 4, -1), (0, 0, 1), 1, 8, model=model))
    # The role and the tag follow the top through the cut.
    new_top = cut.faces().from_role(arris.Role("box", "Face", "Z", "Max")).one()
    assert new_top == cut.faces().extreme((0, 0, 1)).one() == cut.faces().tagged("top").one()
    assert len(cut.faces().last()) == 3
    assert cut.faces().last().kind("cylinder").one() in cut.faces()


def test_algebra_and_adjacency():
    model = arris.Model()
    b = box(model)
    top = b.faces().extreme((0, 0, 1))
    sides = b.faces().parallel((0, 0, 1))
    assert len(top | sides) == 5
    assert len((top | sides) - top) == 4
    assert len(sides & top) == 0
    assert len(top.edges()) == 4
    assert len(top.edges() & sides.edges()) == 4
    assert len(top.edges().faces()) == 5
    assert (top | sides).chain == "(faces().extreme((0, 0, 1)) | faces().parallel((0, 0, 1)))"
    other = box(model, (20, 0, 0), (21, 1, 1))
    with pytest.raises(ValueError, match="selections of"):
        top | other.faces()


def test_one_names_the_chain_and_the_count():
    model = arris.Model()
    b = box(model)
    with pytest.raises(SelectionError) as raised:
        b.faces().parallel((0, 0, 1)).one()
    assert raised.value.count == 4
    assert raised.value.chain == "faces().parallel((0, 0, 1))"
    assert "faces().parallel((0, 0, 1)) selected 4" in str(raised.value)
    assert isinstance(raised.value, arris.ArrisError)
    with pytest.raises(SelectionError, match="selected 0"):
        b.faces().kind("torus").one()


def build_holed_plate(model):
    plate = box(model, (0, 0, 0), (40, 30, 4))
    drills = [
        Solid.cylinder((x, y, -1), (0, 0, 1), r, 6, model=model)
        for x, y, r in ((8, 8, 2), (32, 8, 3), (8, 22, 2.5), (32, 22, 1.5))
    ]
    return plate, drills, plate.cut(*drills)


def test_created_by_after_cut_many_is_exactly_each_tools_wall():
    model = arris.Model()
    plate, drills, holed = build_holed_plate(model)
    walls = holed.faces().kind("cylinder")
    assert len(walls) == 4
    seen = set()
    for drill in drills:
        wall = holed.faces().created_by(drill).one()
        info = model.surface(wall)
        drill_side = model.surface(drill.faces().kind("cylinder").one())
        assert info.kind == "cylinder"
        assert info.radius == pytest.approx(drill_side.radius)
        assert info.origin[:2] == pytest.approx(drill_side.origin[:2])
        seen.add(wall)
    assert seen == set(walls)
    # Neither the plate's faces nor a tool the solid never met.
    assert len(holed.faces().created_by(plate)) == 0
    stray = Solid.cylinder((0, 0, 0), (0, 0, 1), 1, 1, model=model)
    assert len(holed.faces().created_by(stray)) == 0
    # The top and bottom are what the drills modified, and the walls'
    # rims are what they created on the edges.
    assert normals(holed, holed.faces().modified_by(drills[0])) == [(0, 0, -1), (0, 0, 1)]
    rims = holed.edges().created_by(drills[1]).kind("circle")
    assert len(rims) == 2


def test_created_by_follows_later_operations():
    model = arris.Model()
    plate, drills, holed = build_holed_plate(model)
    # A later cut through the first hole's wall splits it; both halves are
    # still what the drill created.
    later = holed.cut(box(model, (7, -1, -1), (9, 31, 2)))
    halves = later.faces().created_by(drills[0])
    assert len(halves) >= 1
    assert all(model.surface(f).kind == "cylinder" for f in halves)
    assert all(model.surface(f).radius == pytest.approx(2) for f in halves)
    assert len(later.faces().created_by(drills[3])) == 1


def chains(solid):
    faces, edges = solid.faces(), solid.edges()
    return {
        "planes": faces.kind("plane"),
        "top": faces.extreme((0, 0, 1)),
        "walls": faces.kind("cylinder"),
        "rims": faces.extreme((0, 0, 1)).edges().kind("circle"),
        "vertical": edges.parallel((0, 0, 1)),
        "corner": faces.within(((-1, -1, -1), (20, 15, 5))),
    }


def test_the_same_chains_pick_the_same_handles_in_two_runs():
    runs = []
    for _ in range(2):
        _, _, holed = build_holed_plate(arris.Model())
        runs.append({k: [m.index for m in v] for k, v in chains(holed).items()})
    assert runs[0] == runs[1]
    for indices in runs[0].values():
        assert indices == sorted(indices)


def grooved(model):
    """A block with two grooves across its top: three coplanar treads."""
    block = box(model, (0, 0, 0), (30, 10, 6))
    return block.cut(
        box(model, (8, -1, 3), (12, 11, 7)),
        box(model, (18, -1, 3), (22, 11, 7)),
    )


def test_extreme_returns_every_coplanar_tie():
    model = arris.Model()
    part = grooved(model)
    treads = part.faces().extreme((0, 0, 1))
    assert len(treads) == 3
    assert all(model.surface(f).normal == (0.0, 0.0, 1.0) for f in treads)
    floors = part.faces().normal((0, 0, 1)) - treads
    assert len(floors) == 2


@pytest.mark.parametrize("angle", [math.radians(30), math.radians(-47), 1.0])
def test_extreme_ties_hold_on_a_part_turned_off_the_axes(angle):
    model = arris.Model()
    part = grooved(model)
    axis = (1.0, 2.0, 0.5)
    turned = part.apply(model.transform(part.body, (3, -2, 1), axis=axis, angle=angle))
    # The treads' normal after the turn, read off one of them.
    tread = turned.faces().from_role(arris.Role("box", "Face", "Z", "Max")).members[0]
    up = model.surface(tread).normal
    treads = turned.faces().extreme(up)
    assert len(treads) == 3
    assert set(treads) <= set(turned.faces().normal(up))
    assert len(turned.faces().normal(up)) == 5
    assert len(turned.faces().perpendicular(up)) == 6
