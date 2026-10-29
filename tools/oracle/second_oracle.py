#!/usr/bin/env python3
"""second_oracle.py <record.json> <fixture-dir> [--variant NAME]  — import
a second oracle's run of a fixture into its `second_oracle.json`
(ADR-0073): the run's measurements divided by its `scale` into the
fixture's units, its source, run and accuracy kept, the recipe's hash
stamped. The run's record is neutral JSON: `source`, `run`, `accuracy`,
`scale`, `volume`, `area`, `centroid` and `inertia` (about the centroid,
unit density, products negated) at the run's scale, and optionally
`operand`, the same four of the operand the run was given, which makes
the entry a record of changes (ADR-0073 as amended). Other variants
already in the file are kept. Exit 1 on a malformed record, 2 on usage.
Run as `uv run --project tools/oracle tools/oracle/second_oracle.py`.
"""

import json
import sys
from pathlib import Path

from oracle import OracleError, require_ocp

require_ocp()

from oracle.fixture import load_fixture  # noqa: E402
from oracle.recipe import recipe_hash  # noqa: E402
from oracle.second import MEASURES, SECOND_ORACLE, load_second, unscaled  # noqa: E402

KEYS = ("source", "run", "accuracy", "scale", *MEASURES)


def imported(run: dict) -> dict:
    """One variant's entry from a run's record."""
    missing = [k for k in KEYS if k not in run]
    if missing:
        raise OracleError(f"the run's record lacks {', '.join(missing)}")
    if run["run"].endswith("-dirty"):
        raise OracleError(f"the run's script was modified ({run['run']}): rerun it from a commit")
    scale = float(run["scale"])
    if not scale > 0:
        raise OracleError(f"scale {scale} is not positive")
    entry = {"source": str(run["source"]), "run": str(run["run"]), "accuracy": float(run["accuracy"]),
             "scale": scale, **unscaled(run, scale)}
    if "operand" in run:
        missing = [k for k in MEASURES if k not in run["operand"]]
        if missing:
            raise OracleError(f"the run's operand lacks {', '.join(missing)}")
        entry["operand"] = unscaled(run["operand"], scale)
    return entry


def main(argv: list[str]) -> int:
    variant = "default"
    if "--variant" in argv:
        i = argv.index("--variant")
        variant = argv[i + 1] if i + 1 < len(argv) else ""
        argv = argv[:i] + argv[i + 2:]
    if len(argv) != 2 or not variant:
        print(__doc__)
        return 2
    record_path, directory = Path(argv[0]), Path(argv[1])
    try:
        run = json.loads(record_path.read_text())
        entry = imported(run)
        digest = recipe_hash(load_fixture(directory))
    except (OracleError, OSError, ValueError) as e:
        print(f"{record_path}: {e}", file=sys.stderr)
        return 1
    record = load_second(directory) or {}
    results = record.get("results", {}) if record.get("recipe_sha256") == digest else {}
    results[variant] = entry
    out = {"recipe_sha256": digest, "results": results}
    # sort_keys and a trailing newline, as expected.json: the file diffs cleanly.
    (directory / SECOND_ORACLE).write_text(json.dumps(out, indent=2, sort_keys=True) + "\n")
    print(f"{directory / SECOND_ORACLE} [{variant}] volume {entry['volume']!r} area {entry['area']!r}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
