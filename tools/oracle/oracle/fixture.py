"""Fixture directories on disk."""

import hashlib
import json
from pathlib import Path

from OCP.BRepCheck import BRepCheck_Analyzer

from . import OracleError, occt_version, step
from .measure import DEFAULT_TOLERANCES, measure, own_measures
from .geometry import compute_geometry
from .recipe import DIR_KEY, build, fixture_kind, probes, recipe_hash, variant_names

FIXTURE = "fixture.json"
EXPECTED = "expected.json"


def repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def corpus_root() -> Path:
    return repo_root() / "tests" / "fixtures"


def load_fixture(directory: Path) -> dict:
    """The recipe in `directory`, with the directory itself under
    `recipe.DIR_KEY`, where a `step` operand's file is looked for: a key
    no recipe writes, and outside the hash."""
    path = directory / FIXTURE
    if not path.exists():
        raise OracleError(f"no {FIXTURE} in {directory}")
    with path.open() as f:
        fixture = json.load(f)
    fixture[DIR_KEY] = str(directory)
    return fixture


def load_expected(directory: Path) -> dict | None:
    path = directory / EXPECTED
    if not path.exists():
        return None
    with path.open() as f:
        return json.load(f)


def fixture_dirs(root: Path | None = None) -> list[Path]:
    root = root or corpus_root()
    return sorted(p.parent for p in root.glob(f"**/{FIXTURE}"))


def compute_expected(fixture: dict, own: bool = False) -> dict:
    """The expected.json content for a recipe: one result per variant for
    the solid kind, the samples and pairs for the geometry kind. `own`
    adds each solid result's `own_measures` under `"own"`: what the
    differential bounds its comparison by, never written for a corpus
    fixture."""
    if fixture_kind(fixture) == "geometry":
        return {"occt": occt_version(), "recipe_sha256": recipe_hash(fixture), "kind": "geometry", **compute_geometry(fixture)}
    if fixture_kind(fixture) == "part":
        expected = {"occt": occt_version(), "recipe_sha256": recipe_hash(fixture), "kind": "part", "solids": compute_part(fixture)}
        if fixture.get("battery"):
            expected["battery"] = compute_battery(fixture)
        return expected
    tol = {**DEFAULT_TOLERANCES, **fixture.get("tolerances", {})}
    # A result Arris refuses as non-manifold is Open CASCADE's compound of
    # solids sharing an edge or a vertex, and one it refuses as a tangent
    # contact carries the contact as an edge of four faces; either may have
    # an odd Euler characteristic.
    manifold = fixture.get("analytic", {}).get("expect_error") not in ("non-manifold", "tangent-contact")
    results = {}
    for variant in variant_names(fixture):
        shape, shapes = build(fixture, variant)
        results[variant] = measure(shape, probes(fixture, variant), tol["probe"], manifold)
        if (half := shapes.get(f"{fixture['result']}.half_space")) is not None:
            # A split's result: what the same side is as the common (or
            # cut) of the body with a half-space box, so a disagreement of
            # Open CASCADE's splitter with its own booleans is on record.
            kept = measure(half, [], tol["probe"], manifold)
            results[variant]["half_space"] = {k: kept[k] for k in ("degenerate", "counts", "volume", "area") if k in kept}
        if not results[variant]["degenerate"] and manifold:
            # The counts of the result converted to B-splines, which may
            # gain seams: what Arris's reader of Open CASCADE's STEP of it
            # is held to (ADR-0025). A conversion Open
            # CASCADE itself fails is recorded instead, and that read-back
            # skipped.
            try:
                results[variant]["nurbs_counts"] = measure(step.nurbs(shape), [], tol["probe"], manifold)["counts"]
            except Exception as e:  # OracleError, or Open CASCADE's own
                results[variant]["nurbs_fails"] = " ".join(str(e).split()) or type(e).__name__
        if own and (measures := own_measures(shape, results[variant])) is not None:
            results[variant]["own"] = measures
    return {"occt": occt_version(), "recipe_sha256": recipe_hash(fixture), "results": results}


def compute_part(fixture: dict) -> list[dict]:
    """A part fixture's solids as Open CASCADE reads its file: each solid of
    the healed reading (ADR-0026 §3), in the transfer's order, with the
    `#id` it came from, what `measure` records of it, and `occt_heals`.
    That is true where the solid needed healing to be one: with healing
    off, a solid of that `#id` fails `BRepCheck_Analyzer`, has other
    counts, or is not read at all. `unhealed_counts` are the counts of the
    unhealed reading, `None` where there is none to count: where they are
    the healed ones, healing changed no topology, and Arris's counts are
    held to them even under `occt_heals`."""
    path = Path(fixture[DIR_KEY]) / fixture["file"]
    if not path.exists():
        raise OracleError(f"no such STEP file: {path}")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != fixture["sha256"]:
        raise OracleError(f"{path} hashes to {digest}, not the fixture's {fixture['sha256']}")
    tol = {**DEFAULT_TOLERANCES, **fixture.get("tolerances", {})}
    healed = [(label, measure(shape, [], tol["probe"], read=True)) for label, shape in step.solids(path)]
    try:
        unhealed = [(label, shape) for label, shape in step.solids(path, heal=False)]
    except Exception:  # OracleError, or Open CASCADE's own on a file it cannot take unhealed
        unhealed = []
    heals: dict[int, bool] = {}
    raw_counts: dict[int, dict | None] = {}
    for label, result in healed:
        raw = [shape for other, shape in unhealed if other == label]
        counts = [_counts_of(shape, tol) for shape in raw]
        heals[label] = heals.get(label, False) or not raw or any(
            not BRepCheck_Analyzer(shape).IsValid() or c != result["counts"] for shape, c in zip(raw, counts)
        )
        # One count for the entity: the unhealed readings' where they all
        # agree, else none.
        same = counts and all(c == counts[0] for c in counts)
        raw_counts[label] = counts[0] if same else None
    return [
        {"id": label, "occt_heals": heals[label], "unhealed_counts": raw_counts[label], **result}
        for label, result in healed
    ]


def compute_battery(fixture: dict) -> dict:
    """Open CASCADE's answer to each case of a part's battery
    (`arris_debug::battery`), by solid and stage: what `measure` records of
    the result with its `own_measures` — the differential's classes hold
    the two kernels to both shapes' tolerances (ADR-0024 §2) — or
    `{"refused": why}` where Open CASCADE raised or built nothing it can
    measure."""
    tol = {**DEFAULT_TOLERANCES, **fixture.get("tolerances", {})}
    out: dict[str, dict] = {}
    for key, cases in sorted(fixture["battery"].items()):
        out[key] = {}
        for stage, case in sorted(cases.items()):
            recipe = {"steps": case["steps"], "result": case["result"], "tolerances": fixture.get("tolerances", {}), DIR_KEY: fixture[DIR_KEY]}
            try:
                shape, _ = build(recipe)
                result = measure(shape, [], tol["probe"])
            except Exception as e:  # OracleError, or Open CASCADE's own on a case it cannot build
                out[key][stage] = {"refused": " ".join(str(e).split()) or type(e).__name__}
                continue
            if (own := own_measures(shape, result)) is not None:
                result["own"] = own
            out[key][stage] = result
    return out


def _counts_of(shape, tol: dict) -> dict | None:
    """The counts `measure` records of a shape, or `None` where it cannot
    measure it (an unhealed solid may not close)."""
    try:
        return measure(shape, [], tol["probe"], manifold=False)["counts"]
    except Exception:  # OracleError, or Open CASCADE's own on a shape it cannot measure
        return None


def dump_expected(expected: dict, path: Path) -> None:
    # sort_keys and a trailing newline: the file diffs cleanly; floats are
    # repr'd by json, full precision.
    path.write_text(json.dumps(expected, indent=2, sort_keys=True) + "\n")


def summary_lines(name: str, expected: dict) -> list[str]:
    """One line per result: per variant for a solid, one for a geometry."""
    if expected.get("kind") == "part":
        return [summary_line(f"{name}[#{s['id']}{', healed' if s['occt_heals'] else ''}]", s) for s in expected["solids"]] + [
            f"{name}{key} {stage}: refused: {result['refused']}" if "refused" in result else summary_line(f"{name}{key} {stage}", result)
            for key, cases in expected.get("battery", {}).items()
            for stage, result in cases.items()
        ]
    if expected.get("kind") == "geometry":
        evals = sum(len(s["evaluations"]) for s in expected["samples"])
        projs = sum(len(s["projections"]) for s in expected["samples"])
        return [f"{name}: geometry, {evals} evaluations, {projs} projections, {len(expected['pairs'])} pairs"]
    return [summary_line(f"{name}[{variant}]", result) for variant, result in expected["results"].items()]


def summary_line(name: str, result: dict) -> str:
    c = result["counts"]
    if result["degenerate"]:
        return f"{name}: degenerate (no solid), counts V/E/F/L={c['vertices']}/{c['edges']}/{c['faces']}/{c['loops']}"
    return (
        f"{name}: volume {result['volume']:.10g}, area {result['area']:.10g}, "
        f"V/E/F/L/S={c['vertices']}/{c['edges']}/{c['faces']}/{c['loops']}/{c['shells']}, "
        f"genus {result['genus']}, "
        + ", ".join(f"{p['label']}={p['class']}" for p in result["probes"])
    )
