"""Placements and features of `arris.build`.

A through-hole in a plate tilted 30° has the closed form's volume (a
cylinder wall through a slab: exact, so no oracle run is needed), and a
24-tooth polar pattern is the raw corpus gear's `toothed` stage.
"""

import math

import arris
import pytest
from arris.build import Solid, Workplane

REL = 1e-9


def volume(solid):
    return solid.model.mass_properties(solid.body).volume


def tilted_plate(model, width=40.0, depth=30.0, thickness=5.0):
    """A plate whose faces are turned 30° about x, on a plane off the axes."""
    base = Workplane.of((3.0, -1.0, 2.0), (0, 0, 1), x_axis=(1, 0, 0)).rotated(
        math.radians(30), (1, 0, 0)
    )
    plate = base.rect(width, depth).extrude(thickness, model=model)
    return base, plate


def test_placements_are_planes_with_the_parents_axes():
    plane = Workplane.XY(2).rotated(0.3, (1, 0, 0))
    spots = plane.at([(1, 0), (0, 3)])
    assert [s.z for s in spots] == [plane.z, plane.z]
    assert spots[1].origin == pytest.approx(plane.point(0, 3), abs=1e-12)
    assert plane.at([]) == []

    ring = plane.polar(6, 4.0, start=0.2)
    for i, spot in enumerate(ring):
        a = 0.2 + 2 * math.pi * i / 6
        assert spot.origin == pytest.approx(plane.point(4 * math.cos(a), 4 * math.sin(a)), abs=1e-12)
        # Its x points away from the centre, along the radius.
        radial = tuple(o - c for o, c in zip(spot.origin, plane.origin))
        assert spot.x == pytest.approx(tuple(r / 4.0 for r in radial), abs=1e-12)

    grid = Workplane.XY().grid(3, 2, 10, 4)
    assert [g.origin[:2] for g in grid] == [
        (-10.0, -2.0), (-10.0, 2.0), (0.0, -2.0), (0.0, 2.0), (10.0, -2.0), (10.0, 2.0),
    ]


def test_a_through_hole_in_a_tilted_plate_has_the_closed_form_volume():
    model = arris.Model()
    base, plate = tilted_plate(model)
    top = base.offset(5)
    drilled = plate.hole(top.at([(4, -3), (-10, 5)]), 4)
    assert volume(drilled) == pytest.approx(40 * 30 * 5 - 2 * math.pi * 2**2 * 5, rel=REL)
    walls = drilled.faces().kind("cylinder")
    assert len(walls) == 2
    assert all(model.surface(w).axis == pytest.approx(base.z, abs=1e-9) or
               model.surface(w).axis == pytest.approx(tuple(-c for c in base.z), abs=1e-9)
               for w in walls)
    assert drilled.model.check(drilled.body, "full")


def test_the_plane_may_sit_inside_or_clear_of_the_solid():
    model = arris.Model()
    base, plate = tilted_plate(model)
    want = 40 * 30 * 5 - math.pi * 3**2 * 5
    for lift in (-3.0, 2.5, 5.0, 12.0, -9.0):
        drilled = plate.hole(base.offset(lift).at([(0, 0)]), 6)
        assert volume(drilled) == pytest.approx(want, rel=REL), lift


def test_a_blind_hole_stops_at_its_depth():
    model = arris.Model()
    base, plate = tilted_plate(model)
    blind = plate.hole(base.offset(5).at([(0, 0)]), 4, depth=2)
    assert volume(blind) == pytest.approx(40 * 30 * 5 - math.pi * 2**2 * 2, rel=REL)
    assert len(blind.faces().kind("cylinder")) == 1
    assert len(blind.faces()) == len(plate.faces()) + 2


def test_a_pocket_cuts_its_sketch_to_a_depth():
    model = arris.Model()
    block = Solid.box((0, 0, 0), (20, 20, 6), model=model)
    pocketed = block.pocket(Workplane.XY(6).slot(10, 4, center=(10, 10), angle=0.5), depth=3)
    area = 6 * 4 + math.pi * 2**2
    assert volume(pocketed) == pytest.approx(20 * 20 * 6 - 3 * area, rel=REL)
    through = block.pocket(Workplane.XY(6).rect(4, 4, center=(10, 10)))
    assert volume(through) == pytest.approx(20 * 20 * 6 - 4 * 4 * 6, rel=REL)
    assert len(through.faces()) == 6 + 4


def test_a_boss_is_fused_along_the_planes_normal():
    model = arris.Model()
    base = Solid.box((0, 0, 0), (20, 20, 2), model=model)
    top = base.faces().extreme((0, 0, 1)).workplane()
    posts = base.boss([p.circle(1.5) for p in top.grid(2, 2, 8, 8)], 5)
    assert volume(posts) == pytest.approx(20 * 20 * 2 + 4 * math.pi * 1.5**2 * 5, rel=REL)
    assert len(posts.faces().kind("cylinder")) == 4
    one = base.boss(top.rect(4, 4), 1)
    assert volume(one) == pytest.approx(20 * 20 * 2 + 16, rel=REL)


def test_an_empty_pattern_is_the_solid_itself():
    block = Solid.box((0, 0, 0), (2, 2, 2))
    assert block.hole([], 1) is block
    assert block.pocket([]) is block
    assert block.boss([], 1) is block


def test_a_hole_off_the_solid_cuts_nothing():
    block = Solid.box((0, 0, 0), (10, 10, 4), model=arris.Model())
    away = block.hole(Workplane.XY(4).at([(50, 50)]), 2)
    assert volume(away) == pytest.approx(400, rel=REL)


def test_tags_survive_a_hole_and_a_boss():
    block = Solid.box((0, 0, 0), (10, 10, 4))
    block = block.tag("skin", block.faces())
    top = block.faces().extreme((0, 0, 1)).workplane()
    made = block.hole(top.at([(0, 0)]), 2).boss(top.rect(1, 1, center=(3, 3)), 1)
    assert made.tagged("skin") <= frozenset(made.faces())
    assert len(made.tagged("skin")) >= 6


def test_a_polar_pattern_is_the_corpus_gears_toothed_stage():
    disc_r, thickness, hub_r, hub_h = 30.0, 8.0, 10.0, 14.0
    teeth, width = 24, 3.0
    model = arris.Model()
    disc = Solid.cylinder((0, 0, 0), (0, 0, 1), disc_r, thickness, model=model)
    hub = Solid.cylinder((0, 0, 0), (0, 0, 1), hub_r, hub_h, model=model)
    step = 2 * math.pi / teeth
    # A tooth drawn once, on a plane whose x points out along its radius.
    root = Workplane.XY().polar(teeth, disc_r, start=step / 2)
    built = disc.fuse(hub).boss([p.rect(5, width, center=(1.5, 0)) for p in root], thickness)

    tooth = model.primitive_box(
        (disc_r - 1, -width / 2, 0), (disc_r + 4, width / 2, thickness)
    )[0]
    raw_teeth = [
        model.transform(tooth, axis=(0, 0, 1), angle=(i + 0.5) * step)[0] for i in range(teeth)
    ]
    blank = model.fuse(disc.body, hub.body)[0]
    raw = model.fuse_many([blank, *raw_teeth])[0]

    got, want = model.mass_properties(built.body), model.mass_properties(raw)
    assert got.volume == pytest.approx(want.volume, rel=REL)
    assert got.area == pytest.approx(want.area, rel=REL)
    assert got.centroid == pytest.approx(want.centroid, abs=1e-7)
    for row_got, row_want in zip(got.inertia, want.inertia):
        assert row_got == pytest.approx(row_want, rel=1e-8, abs=1e-8 * want.volume)
    assert model.check(built.body, "full")
