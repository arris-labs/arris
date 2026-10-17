#!/usr/bin/env python3
"""showcase.py <file.step>  — read the showcase part's STEP file (Arris's
output) through Open CASCADE and print its volume, area, centroid and
counts as JSON: what `crates/arris-py/examples/bracket.oracle.json`
records and `crates/arris-py/tests/test_showcase.py` holds the part to. An
independent reader of the bytes, not a fixture comparison. Exits 2 on a
usage or read error. Run as
`uv run --project tools/oracle tools/oracle/showcase.py`.
"""

import json
import sys
from pathlib import Path

from oracle import OracleError, require_ocp

require_ocp()

from oracle import step  # noqa: E402
from oracle.measure import measure  # noqa: E402


def main(argv: list[str]) -> int:
    if len(argv) != 1:
        print(__doc__)
        return 2
    try:
        measured = measure(step.read(Path(argv[0])), [], 1e-6, read=True)
    except OracleError as e:
        print(f"showcase: ERROR {e}", file=sys.stderr)
        return 2
    keep = ("volume", "area", "centroid", "counts", "genus")
    print(json.dumps({k: measured[k] for k in keep}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
