#!/usr/bin/env python3
"""compare.py <fixture-dir> <file.step> [--variant NAME]  — read a STEP
file, measure it, and compare against the fixture's expected.json within
the fixture's tolerances. Prints a table; exits 1 on any mismatch, 2 on a
usage or environment error. Run as
`uv run --project tools/oracle tools/oracle/compare.py`.
"""

import sys
from pathlib import Path

from oracle import OracleError, require_ocp

require_ocp()

from oracle import step  # noqa: E402
from oracle.fixture import load_expected, load_fixture  # noqa: E402
from oracle.measure import DEFAULT_TOLERANCES, compare, format_table, measure  # noqa: E402
from oracle.measure import sheet as measure_sheet  # noqa: E402
from oracle.recipe import fixture_kind, number, probes, recipe_hash, resolve_params  # noqa: E402


def compare_step(directory: Path, step_file: Path, variant: str = "default") -> tuple[bool, str]:
    """(all ok, the table) for one comparison."""
    fixture = load_fixture(directory)
    if fixture_kind(fixture) == "geometry":
        raise OracleError(f"{directory} is a geometry fixture: Arris compares it in crates/arris-geom/tests/oracle.rs, not from a STEP file")
    if fixture_kind(fixture) == "part":
        raise OracleError(f"{directory} is a part fixture: Arris holds its reading to expected.json itself")
    expected = load_expected(directory)
    if expected is None:
        raise OracleError(f"no expected.json in {directory}; run expected.py first")
    if expected.get("recipe_sha256") != recipe_hash(fixture):
        raise OracleError(f"{directory}: expected.json is stale (recipe hash differs); run expected.py")
    if variant not in expected["results"]:
        raise OracleError(f"{directory}: no variant {variant!r} in expected.json")
    tol = {**DEFAULT_TOLERANCES, **fixture.get("tolerances", {})}
    shape = step.read(step_file)
    result = expected["results"][variant]
    if result.get("sheet"):
        actual = {"sheet": True, **measure_sheet(shape)}
    else:
        actual = measure(shape, probes(fixture, variant), tol["probe"])
    analytic = fixture.get("analytic", {})
    if analytic.get("counts_differ") and "counts" in analytic:
        # The recipe states a convention Arris does not follow and gives
        # its own counts; the oracle's stay in expected.json as the record.
        result = {**result, "counts": {"shells": 1, "solids": 1, **analytic["counts"]}}
        if "genus" in analytic:
            result["genus"] = analytic["genus"]
    if analytic.get("measure_differs"):
        # The recipe states the oracle's measurements are wrong and gives
        # closed forms for them (ADR-0015); the oracle's stay in
        # expected.json as the record.
        missing = [k for k in ("volume", "area", "centroid", "inertia") if k not in analytic]
        if missing:
            raise OracleError(f"{directory}: analytic.measure_differs needs analytic.{', analytic.'.join(missing)}")
        params = resolve_params(fixture, variant)
        result = {
            **result,
            "volume": number(analytic["volume"], params),
            "area": number(analytic["area"], params),
            "centroid": [number(x, params) for x in analytic["centroid"]],
            "inertia": [[number(x, params) for x in row] for row in analytic["inertia"]],
        }
    rows = compare(result, actual, tol)
    return all(r[3] for r in rows), format_table(rows)


def main(argv: list[str]) -> int:
    variant = "default"
    if "--variant" in argv:
        i = argv.index("--variant")
        variant = argv[i + 1]
        argv = argv[:i] + argv[i + 2 :]
    if len(argv) != 2:
        print(__doc__)
        return 2
    try:
        ok, table = compare_step(Path(argv[0]), Path(argv[1]), variant)
    except OracleError as e:
        print(f"compare: ERROR {e}", file=sys.stderr)
        return 2
    print(table)
    print("MATCH" if ok else "MISMATCH")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
