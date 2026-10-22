"""The README's part: `examples/bracket.py` builds it on `arris.build`, and
Open CASCADE's reading of its STEP (`bracket.oracle.json`, written by
`tools/oracle/showcase.py`) holds its volume, area and centroid."""

import importlib.util
import json
from collections import Counter
from pathlib import Path

import arris
import pytest

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"

# Faces are curved and the oracle reads the STEP, not the model: the
# fixture defaults of `conftest`.
VOLUME_REL = 1e-9
AREA_REL = 1e-9
CENTROID_ABS = 1e-7


def load_bracket():
    spec = importlib.util.spec_from_file_location("bracket", EXAMPLES / "bracket.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def bracket():
    module = load_bracket()
    return module, module.build()


def test_the_bracket_is_checker_green(bracket):
    _, part = bracket
    report = part.check("full")
    assert report, str(report)
    assert not report.unchecked


def test_the_bracket_matches_the_oracles_reading_of_its_step(bracket):
    _, part = bracket
    expected = json.loads((EXAMPLES / "bracket.oracle.json").read_text())
    props = part.measure()
    assert props.volume == pytest.approx(expected["volume"], rel=VOLUME_REL, abs=0)
    assert props.area == pytest.approx(expected["area"], rel=AREA_REL, abs=0)
    assert props.centroid == pytest.approx(tuple(expected["centroid"]), abs=CENTROID_ABS, rel=0)
    line = part.check("full").euler
    counts = expected["counts"]
    assert (line.vertices, line.edges, line.faces, line.loops, line.shells) == (
        counts["vertices"],
        counts["edges"],
        counts["faces"],
        counts["loops"],
        counts["shells"],
    )
    assert line.genus == expected["genus"]


def test_every_face_carries_the_operation_that_made_it(bracket):
    module, part = bracket
    labels = [name for name, faces in part.tags.items() for _ in faces]
    tagged = [face for faces in part.tags.values() for face in faces]
    assert len(tagged) == len(set(tagged)) and set(tagged) == set(part.faces())
    assert Counter(labels) == {
        "plate": 8,
        "boss": 2,
        "bore": 1,
        "bolt holes": 2,
        "counterbores": 4,
        "bend fillet": 1,
        "rim fillets": 2,
        "chamfer": 1,
    }
    assert set(part.tags) == set(module.LABELS)


def test_the_files_are_written_and_the_step_round_trips(bracket, tmp_path):
    module, part = bracket
    module.write(part, tmp_path)
    text = (tmp_path / "bracket.step").read_text()
    back = arris.Model()
    [solid] = back.read_step(text).solids
    assert solid.ok and back.check(solid.body, "full")
    assert back.mass_properties(solid.body).volume == pytest.approx(
        part.measure().volume, rel=1e-9
    )
    labels = json.loads((tmp_path / "bracket.labels.json").read_text())
    mesh = (tmp_path / "bracket.mesh").read_bytes()
    n_positions, n_triangles = int.from_bytes(mesh[:4], "little"), int.from_bytes(mesh[4:8], "little")
    assert len(mesh) == 8 + 24 * n_positions + 12 * n_triangles
    assert len(labels["triangles"]) == n_triangles
    assert set(labels["triangles"]) == set(range(len(module.LABELS)))


def test_two_runs_write_the_same_bytes(tmp_path):
    module = load_bracket()
    for name in ("a", "b"):
        module.write(module.build(), tmp_path / name)
    for file in ("bracket.step", "bracket.mesh", "bracket.labels.json"):
        assert (tmp_path / "a" / file).read_bytes() == (tmp_path / "b" / file).read_bytes()


def test_the_readmes_snippet_builds_the_same_part():
    readme = (Path(__file__).resolve().parents[3] / "README.md").read_text()
    start = readme.index("```python\n# A motor-mount bracket") + len("```python\n")
    scope: dict[str, object] = {}
    exec(readme[start : readme.index("```\n", start)], scope)  # noqa: S102
    part = scope["part"]
    expected = json.loads((EXAMPLES / "bracket.oracle.json").read_text())
    assert part.measure().volume == pytest.approx(expected["volume"], rel=VOLUME_REL, abs=0)
    assert len(part.faces()) == expected["counts"]["faces"]
