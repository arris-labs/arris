"""The installed wheel, as `pip install arris` leaves it: no source tree on
the import path and no compiler to fall back on. A fresh interpreter, its
`PATH` cut to its own directory, imports `arris`, builds the showcase
bracket (`examples/bracket.py`) and is held to `bracket.oracle.json`.

`maturin develop` installs the package as a path into the source tree; there
these tests skip, since they would prove nothing a source build did not."""

import json
import os
import subprocess
import sys
from pathlib import Path

import arris
import pytest

PACKAGE_SOURCE = Path(__file__).resolve().parents[1] / "python"
EXAMPLES = Path(__file__).resolve().parents[1] / "examples"

# Faces are curved and the oracle reads the STEP, not the model: the
# fixture defaults of `conftest`, as `test_showcase` holds them.
VOLUME_REL = 1e-9
AREA_REL = 1e-9
CENTROID_ABS = 1e-7

# The tools a missing wheel would send `pip` looking for.
COMPILERS = ("cargo", "rustc", "cc", "gcc", "clang", "cl")

pytestmark = pytest.mark.skipif(
    Path(arris.__file__).resolve().is_relative_to(PACKAGE_SOURCE),
    reason="arris is imported from the source tree (maturin develop), not an installed wheel",
)

# Runs in the child: no compiler on `PATH`, `arris` from site-packages, the
# bracket's measures and counts printed as JSON.
CHILD = """
import importlib.util, json, shutil, sys
from pathlib import Path
missing = [tool for tool in sys.argv[2:] if shutil.which(tool) is None]
import arris
spec = importlib.util.spec_from_file_location("bracket", Path(sys.argv[1]) / "bracket.py")
bracket = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bracket)
part = bracket.build()
report = part.check("full")
props = part.measure()
line = report.euler
print(json.dumps({
    "compilers_absent": missing == sys.argv[2:],
    "module": arris._arris.__file__,
    "green": bool(report) and not report.unchecked,
    "volume": props.volume,
    "area": props.area,
    "centroid": list(props.centroid),
    "counts": [line.vertices, line.edges, line.faces, line.loops, line.shells],
    "genus": line.genus,
}))
"""


@pytest.fixture(scope="module")
def child():
    env = {
        key: value
        for key, value in os.environ.items()
        if not key.startswith(("PYTHON", "VIRTUAL_ENV", "CARGO", "RUSTUP"))
    }
    env["PATH"] = str(Path(sys.executable).parent)
    out = subprocess.run(
        [sys.executable, "-I", "-c", CHILD, str(EXAMPLES), *COMPILERS],
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert out.returncode == 0, out.stderr
    return json.loads(out.stdout)


def test_the_wheel_imports_with_no_compiler_on_path(child):
    assert child["compilers_absent"]
    assert not Path(child["module"]).resolve().is_relative_to(PACKAGE_SOURCE.parent)


def test_the_wheel_builds_the_bracket_the_oracle_measured(child):
    expected = json.loads((EXAMPLES / "bracket.oracle.json").read_text())
    assert child["green"]
    assert child["volume"] == pytest.approx(expected["volume"], rel=VOLUME_REL, abs=0)
    assert child["area"] == pytest.approx(expected["area"], rel=AREA_REL, abs=0)
    assert child["centroid"] == pytest.approx(expected["centroid"], abs=CENTROID_ABS, rel=0)
    counts = expected["counts"]
    assert child["counts"] == [counts[k] for k in ("vertices", "edges", "faces", "loops", "shells")]
    assert child["genus"] == expected["genus"]
