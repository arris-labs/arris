#!/usr/bin/env python3
"""occt_surface.py <fixture-dir> [--variant NAME]  -- build the fixture's
recipe in Open CASCADE and print, as JSON, the surface properties of the
result's faces at unit area density: `area`, `centroid`, `inertia` (about
the centroid, physical convention). What `arris_ops::measure::
surface_properties` is held to. Exits 2 with `occt_surface: ERROR <why>` on
stderr for a recipe the oracle cannot build. Run as
`uv run --project tools/oracle tools/oracle/occt_surface.py`.
"""

import json
import sys
from pathlib import Path

from oracle import OracleError, require_ocp

require_ocp()

from oracle import measure  # noqa: E402
from oracle.fixture import load_fixture  # noqa: E402
from oracle.recipe import build, fixture_kind  # noqa: E402


def main(argv: list[str]) -> int:
    variant = "default"
    if "--variant" in argv:
        i = argv.index("--variant")
        variant = argv[i + 1]
        argv = argv[:i] + argv[i + 2 :]
    if len(argv) != 1:
        print(__doc__)
        return 2
    try:
        fixture = load_fixture(Path(argv[0]))
        if fixture_kind(fixture) in ("geometry", "part"):
            raise OracleError(f"{argv[0]} is a {fixture_kind(fixture)} fixture: it has no recipe result")
        shape, _ = build(fixture, variant)
        out = measure.surface_properties(shape)
    except Exception as e:  # OracleError, or Open CASCADE's own
        message = " ".join(str(e).split()) or type(e).__name__
        print(f"occt_surface: ERROR {message}", file=sys.stderr)
        return 2
    print(json.dumps(out))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
