"""The hand-written stubs match the built module (`mypy.stubtest`).

A stub that names something the module does not have, misses something it
has, or gives a method another signature fails here; so does a name in
`arris.__all__` the package stub does not list.
"""

import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("mypy", reason="stubs are checked with mypy.stubtest: pip install 'arris[test]'")

ROOT = Path(__file__).resolve().parents[1]


def test_the_stubs_match_the_module():
    run = subprocess.run(
        [
            sys.executable,
            "-m",
            "mypy.stubtest",
            "arris",
            "--allowlist",
            str(ROOT / "stubtest_allowlist.txt"),
        ],
        capture_output=True,
        text=True,
        cwd=ROOT,
    )
    assert run.returncode == 0, run.stdout + run.stderr


def test_the_package_is_marked_typed():
    assert (ROOT / "python" / "arris" / "py.typed").is_file()
    for stub in ("__init__", "_arris", "_numpy"):
        assert (ROOT / "python" / "arris" / f"{stub}.pyi").is_file(), stub
