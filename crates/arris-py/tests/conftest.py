"""Shared helpers: the corpus fixtures' oracle values, read from their
`expected.json`, and the fixtures' default tolerances."""

import json
from pathlib import Path

import pytest

CORPUS = Path(__file__).resolve().parents[3] / "tests" / "fixtures"

# `arris_debug::fixtures::Tolerances::default()`: what a fixture is held to
# unless its own `tolerances` says otherwise.
VOLUME_REL = 1e-9
AREA_REL = 1e-9
CENTROID_ABS = 1e-7
INERTIA_REL = 1e-9
MESH_VOLUME_REL = 2e-3
MESH_CHORD = 1e-3


def oracle(fixture, variant="default"):
    """The Open CASCADE oracle's answer for `area/slug`."""
    path = CORPUS / fixture / "expected.json"
    return json.loads(path.read_text())["results"][variant]


def tolerances(fixture):
    """The fixture's own tolerances over the defaults."""
    recipe = json.loads((CORPUS / fixture / "fixture.json").read_text())
    return {
        "volume_rel": VOLUME_REL,
        "area_rel": AREA_REL,
        "centroid_abs": CENTROID_ABS,
        "inertia_rel": INERTIA_REL,
        "mesh_volume_rel": MESH_VOLUME_REL,
        "mesh_chord": MESH_CHORD,
        **recipe.get("tolerances", {}),
    }


def matches_oracle(model, body, fixture, variant="default"):
    """Asserts `body` has the oracle's volume, area, centroid, inertia and
    counts for `fixture`, to the fixture's tolerances, and passes the
    checker at the `full` level."""
    expected, held = oracle(fixture, variant), tolerances(fixture)
    props = model.mass_properties(body)
    assert props.volume == pytest.approx(expected["volume"], rel=held["volume_rel"], abs=0)
    assert props.area == pytest.approx(expected["area"], rel=held["area_rel"], abs=0)
    assert props.centroid == pytest.approx(
        tuple(expected["centroid"]), abs=held["centroid_abs"], rel=0
    )
    scale = max(abs(x) for row in expected["inertia"] for x in row)
    for got, want in zip(props.inertia, expected["inertia"]):
        assert got == pytest.approx(tuple(want), abs=held["inertia_rel"] * scale, rel=0)
    report = model.check(body, "full")
    assert report, str(report)
    assert not report.unchecked
    counts = expected["counts"]
    line = report.euler
    assert (line.vertices, line.edges, line.faces, line.loops, line.shells) == (
        counts["vertices"],
        counts["edges"],
        counts["faces"],
        counts["loops"],
        counts["shells"],
    )
    assert line.genus == expected["genus"]
    return props
