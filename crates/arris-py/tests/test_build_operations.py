"""The rest of `arris.build.Solid`'s operations: each one is the raw binding
call plus provenance and tags."""

import math

import arris
import pytest
from arris.build import Solid

REL = 1e-9


def box(model=None):
    return Solid.box((0, 0, 0), (10, 8, 6), model=model)


def test_fillet_and_chamfer_match_the_raw_calls_and_keep_tags():
    model = arris.Model()
    block = box(model)
    block = block.tag("skin", block.faces())
    edges = list(block.edges().parallel((0, 0, 1)))
    rounded = block.fillet(edges, 1.5)
    raw = model.fillet(block.body, edges, 1.5)[0]
    assert rounded.measure().volume == pytest.approx(model.mass_properties(raw).volume, rel=REL)
    assert rounded.measure().volume == pytest.approx(480 - 4 * (1 - math.pi / 4) * 1.5**2 * 6, rel=REL)
    # Blend faces are made from edges, so only the six faces carry the tag.
    assert len(rounded.tagged("skin")) == 6 and len(rounded.faces()) == 10
    assert rounded.check("full")

    top = block.faces().extreme((0, 0, 1)).edges()
    cut = block.chamfer(top, 1)
    raw = model.chamfer(block.body, list(top), 1)[0]
    assert cut.measure().volume == pytest.approx(model.mass_properties(raw).volume, rel=REL)
    assert cut.measure().volume < 480
    assert len(cut.faces()) == 10


def test_shell_hollows_to_the_wall_thickness():
    block = box()
    cup = block.shell(block.faces().extreme((0, 0, 1)), 1)
    assert cup.measure().volume == pytest.approx(480 - 8 * 6 * 5, rel=REL)
    assert cup.check("full")
    top = block.faces().extreme((0, 0, 1))
    model = block.model
    raw = model.shell(block.body, list(top), 1, "outward")[0]
    out = block.shell(top, 1, "outward")
    assert out.measure().volume == pytest.approx(model.mass_properties(raw).volume, rel=REL)


def test_moved_and_mirrored_place_the_solid():
    block = box()
    moved = block.moved((5, -2, 1), axis=(0, 0, 1), angle=math.pi / 2)
    assert moved.measure().volume == pytest.approx(480, rel=REL)
    assert moved.measure().centroid == pytest.approx((5 - 4, -2 + 5, 1 + 3), abs=1e-9)
    image = block.mirrored((0, 0, 0), (1, 0, 0))
    assert image.measure().centroid == pytest.approx((-5, 4, 3), abs=1e-9)
    assert image.measure().volume == pytest.approx(480, rel=REL)
    assert image.check("full")
    assert moved.last != block.last


def test_a_refusal_reaches_the_caller_as_the_bindings_exception():
    block = box()
    with pytest.raises(arris.OpError):
        block.fillet(block.edges().parallel((0, 0, 1)), 100)
    with pytest.raises(arris.OpError):
        block.shell(block.faces().extreme((0, 0, 1)), 50)


def test_queries_and_step_round_trip(tmp_path):
    block = box()
    assert block.mesh(0.1).n_triangles == 12
    text = block.to_step()
    assert text.startswith("ISO-10303-21")
    target = tmp_path / "block.step"
    block.export_step(target)
    assert target.read_text() == text
    back = arris.Model().read_step(text)
    assert back.solids[0].body is not None
