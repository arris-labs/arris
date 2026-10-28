"""A second oracle's record (`second_oracle.json`, ADR-0073): a second
kernel's measurements of a fixture's result, run as a black box. Under
`analytic.measure_differs` a fixture is held to it in place of closed forms.
Written only by `tools/oracle/second_oracle.py`, never by hand."""

import json
from pathlib import Path

from oracle import OracleError

SECOND_ORACLE = "second_oracle.json"
MEASURES = ("volume", "area", "centroid", "inertia")

# The power of the run's length scale each measure carries.
POWERS = {"volume": 3, "area": 2, "centroid": 1, "inertia": 5}


def load_second(directory: Path) -> dict | None:
    """The directory's record, or None where it has none."""
    path = directory / SECOND_ORACLE
    if not path.exists():
        return None
    with path.open() as f:
        return json.load(f)


def held_measures(directory: Path, recipe_sha256: str, variant: str) -> dict | None:
    """The record's four measurements of `variant`, or None where the
    directory has no record. Errors: a stale record, or one without the
    variant."""
    record = load_second(directory)
    if record is None:
        return None
    if record.get("recipe_sha256") != recipe_sha256:
        raise OracleError(f"{directory}: {SECOND_ORACLE} is stale (recipe hash differs); rerun the second oracle")
    result = record.get("results", {}).get(variant)
    if result is None:
        raise OracleError(f"{directory}: {SECOND_ORACLE} has no result for variant {variant!r}")
    return {k: result[k] for k in MEASURES}


def unscaled(run: dict, scale: float) -> dict:
    """A run's measurements in the fixture's units: each divided by the
    power of `scale` it carries."""
    out = {}
    for key, power in POWERS.items():
        value = run[key]
        k = scale**power
        out[key] = [[x / k for x in row] for row in value] if key == "inertia" else (
            [x / k for x in value] if key == "centroid" else value / k)
    return out
