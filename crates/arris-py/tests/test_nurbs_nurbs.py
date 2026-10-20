"""Two NURBS faces as operands of each other (nurbs-nurbs-operands step 8):
a spline sweep cut by a smooth loft, both fitted walls, and the crest of a
swept thread chamfered between its two fitted flanks, each held to Open
CASCADE through the corpus fixture it repeats.
"""

import math

import arris
import pytest

from conftest import oracle, tolerances

LOFT_SWEEP = "loft/loft-smooth-fuse-spline-sweep"
THREAD = "sweep/sweep-thread-chamfer"


def smooth_loft(model):
    """Three circles lofted smooth: one NURBS wall."""

    def ring(centre, radius):
        return arris.Profile(arris.Loop.circle((0, 0), radius), origin=centre, normal=(0, 0, 1))

    sections = [ring((0, 0, 0), 2), ring((1, 0, 5), 3), ring((0, 0, 10), 2)]
    body, _ = model.loft(sections, kind="smooth")
    return body


def spline_bar(model):
    """A circle of radius 1 along a cubic across the loft: one NURBS wall."""
    profile = arris.Profile(
        arris.Loop.circle((0, 0), 1), origin=(-7, 0, 4), normal=(1, 0, 0), x_axis=(0, 1, 0)
    )
    cubic = arris.PathSegment.nurbs(
        3, [0, 0, 0, 0, 1, 1, 1, 1], [(-7, 0, 4), (-2, 0, 4), (2, 0, 6), (7, 0, 6)]
    )
    body, _ = model.sweep(
        profile, arris.Path((-7, 0, 4), [cubic]), arris.FrameLaw.rotation_minimizing()
    )
    return body


def test_a_spline_sweep_cut_by_a_smooth_loft_adds_back_to_the_sweep():
    model = arris.Model()
    loft, bar = smooth_loft(model), spline_bar(model)
    fused, _ = model.fuse(loft, bar)
    expected, held = oracle(LOFT_SWEEP), tolerances(LOFT_SWEEP)
    union = model.mass_properties(fused)
    assert union.volume == pytest.approx(expected["volume"], rel=held["volume_rel"])
    assert union.area == pytest.approx(expected["area"], rel=held["area_rel"])

    cut, record = model.cut(bar, loft)
    common, _ = model.common(bar, loft)
    assert record.generated
    assert model.check(cut, "fast")
    whole = model.mass_properties(bar).volume
    outside = model.mass_properties(cut).volume
    inside = model.mass_properties(common).volume
    # The bar runs through the loft and out of both sides: what the cut
    # keeps and what the common keeps make the bar again, and the union is
    # the loft and the bar less the part they share.
    assert 0 < inside < outside
    assert outside + inside == pytest.approx(whole, rel=1e-9)
    loft_volume = model.mass_properties(loft).volume
    assert union.volume == pytest.approx(loft_volume + whole - inside, rel=1e-9)


def thread(model, turns=3):
    """The triangle of `sweep/sweep-thread` along a helix of radius 10,
    pitch 4, under the axis as binormal: three fitted faces."""
    profile = arris.Profile(
        arris.Loop.polygon([(0, -1.2), (1.5, 0), (0, 1.2)]),
        origin=(10, 0, 0),
        normal=(0, -1, 0),
        x_axis=(1, 0, 0),
    )
    coil = arris.PathSegment.helix((0, 0, 0), (0, 0, 1), pitch=4.0, angle=2 * math.pi * turns)
    body, _ = model.sweep(
        profile, arris.Path((10, 0, 0), [coil]), arris.FrameLaw.fixed_binormal((0, 0, 1))
    )
    return body


def test_a_threads_crest_chamfers_between_its_two_fitted_flanks():
    model = arris.Model()
    body = thread(model)
    crest = min(model.edges(body), key=lambda e: model.distance((0, 11.5, 1.0), e).distance)
    chamfered, record = model.chamfer(body, [crest], 0.1)
    assert record.generated
    expected, held = oracle(THREAD), tolerances(THREAD)
    props = model.mass_properties(chamfered)
    assert props.volume == pytest.approx(expected["volume"], rel=held["volume_rel"])
    assert props.area == pytest.approx(expected["area"], rel=held["area_rel"])
    assert len(model.faces(chamfered)) == expected["counts"]["faces"]
