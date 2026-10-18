"""The script corpus (`crates/arris-py/corpus/`): each part is a script on the
raw binding. Every stage it reaches is checker-green (the harness holds that),
matches Open CASCADE's build of the same part at that stage
(`expected.json`'s checkpoints, at the fixture defaults of `conftest`), and
every stage's outcome, built or refused, is the committed one. `ARRIS_BLESS=1`
rewrites `outcome.json`; a lifted refusal is a `fixtures:` commit."""

import json
import os
import sys
from pathlib import Path

import arris
import pytest

from conftest import (
    AREA_REL,
    CENTROID_ABS,
    INERTIA_REL,
    VOLUME_REL,
)

CORPUS = Path(__file__).resolve().parents[1] / "corpus"
REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(CORPUS))

import harness  # noqa: E402


def waits():
    """`corpus/waits`: `<part> <regression/slug>` per line, the faults the
    corpus has met (`tools/real-parts.waits`'s format)."""
    path = CORPUS / "waits"
    if not path.exists():
        return {}
    lines = (line.split("#")[0].split() for line in path.read_text().splitlines())
    return {line[0]: line[1] for line in lines if line}


WAITS = waits()


def tolerances(directory):
    """The part's own `tolerances` (the fixtures' grammar, set where Open
    CASCADE's approximation of a blend or a helical sweep is the limit) over
    the defaults of `conftest`."""
    own = json.loads((directory / "oracle.json").read_text()).get("tolerances", {})
    return {"volume_rel": VOLUME_REL, "area_rel": AREA_REL, "centroid_abs": CENTROID_ABS, "inertia_rel": INERTIA_REL, **own}


def hold_to_oracle(model, body, want, tol):
    props = model.mass_properties(body)
    assert props.volume == pytest.approx(want["volume"], rel=tol["volume_rel"], abs=0)
    assert props.area == pytest.approx(want["area"], rel=tol["area_rel"], abs=0)
    assert props.centroid == pytest.approx(tuple(want["centroid"]), abs=tol["centroid_abs"], rel=0)
    scale = max(abs(x) for row in want["inertia"] for x in row)
    for got, expected in zip(props.inertia, want["inertia"]):
        assert got == pytest.approx(tuple(expected), abs=tol["inertia_rel"] * scale, rel=0)
    line = model.check(body, "full").euler
    counts = want["counts"]
    assert (line.vertices, line.edges, line.faces, line.loops, line.shells) == (
        counts["vertices"],
        counts["edges"],
        counts["faces"],
        counts["loops"],
        counts["shells"],
    )
    assert line.genus == want["genus"]


def step_bytes(slug):
    run = harness.run_part(slug)
    return run.model.write_step([run.last]) if run.last is not None else ""


@pytest.mark.parametrize("slug", harness.slugs())
def test_part(slug):
    directory = CORPUS / slug
    if slug in WAITS:
        assert (REPO / "tests" / "fixtures" / WAITS[slug]).is_dir(), f"{slug} waits on a missing fixture"
        with pytest.raises(harness.Fault):
            harness.run_part(slug)
        return

    run = harness.run_part(slug)

    outcome = directory / "outcome.json"
    if os.environ.get("ARRIS_BLESS"):
        outcome.write_text(json.dumps(run.outcome, indent=2) + "\n")
    assert run.outcome == json.loads(outcome.read_text())

    # Each stage built is the oracle's: at the checkpoint of its name, unless
    # Open CASCADE cannot build that step or a refused `finish` stage means
    # the oracle's later steps carry an operation this run lacks.
    checkpoints = json.loads((directory / "expected.json").read_text())["results"]["default"]["checkpoints"]
    tol = tolerances(directory)
    for name, body in run.bodies.items():
        want = checkpoints[name]
        if "refused" in want or not run.comparable[name]:
            continue
        hold_to_oracle(run.model, body, want, tol)

    # The same script twice writes the same bytes.
    if run.last is not None:
        assert run.model.write_step([run.last]) == step_bytes(slug)

    # A labelled part's tagged sets survive and reach the mesh.
    if run.labelled is not None:
        run.labelled.assert_tagged()

