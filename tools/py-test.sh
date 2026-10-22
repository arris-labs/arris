#!/usr/bin/env bash
# The binding's tests, the way CI's `python` job runs them (ADR-0034): one
# virtual environment, `maturin develop` with the test extras, the suite on
# xdist workers, the docstring examples serially, `mypy --strict -p
# arris.build`.
#
#   tools/py-test.sh                  the whole suite, then mypy
#   tools/py-test.sh <pytest args>    a slice, serially, from crates/arris-py
#                                     (`tests/test_mesh.py`, `-k cylinder`);
#                                     no mypy
#
#   ARRIS_PYENV     the environment (default target/py-venv, made with uv
#                   when missing; CI points it at the runner's)
#   ARRIS_PYTHON    the interpreter uv makes it with (default uv's pick)
#   PYTEST_WORKERS  xdist workers for the whole suite (default half the
#                   cores, as the hook's cargo runs; `auto` for all)
#
# One run per environment at a time: a second waits on the first's lock, so
# two sessions never rebuild the same extension at once. The whole suite
# takes minutes: run it in the background with its output in a file.
set -euo pipefail
cd "$(dirname "$0")/.."

env=${ARRIS_PYENV:-$PWD/target/py-venv}
workers=${PYTEST_WORKERS:-$(($(nproc) / 2 > 0 ? $(nproc) / 2 : 1))}
export CARGO_BUILD_JOBS=${CARGO_BUILD_JOBS:-$(($(nproc) / 2 > 0 ? $(nproc) / 2 : 1))}

mkdir -p "$(dirname "$env")"
exec 9>"$env.lock"
flock 9

if [ ! -x "$env/bin/python" ]; then
  uv venv -q ${ARRIS_PYTHON:+--python "$ARRIS_PYTHON"} "$env"
fi
export VIRTUAL_ENV=$env PATH=$env/bin:$PATH
[ -x "$env/bin/maturin" ] || uv pip install -q --python "$env/bin/python" maturin
maturin develop -q --uv --manifest-path crates/arris-py/Cargo.toml --extras test

# xdist's workers must each collect the same tests, which a parametrize over
# a set does only under one hash seed.
export PYTHONHASHSEED=0
cd crates/arris-py
if [ $# -gt 0 ]; then
  python -m pytest "$@"
  exit
fi
# The docstring examples collect differently in each worker even so, which
# xdist refuses; they are seconds serially.
python -m pytest -q -n "$workers" --ignore=tests/test_docstrings.py
python -m pytest -q tests/test_docstrings.py
mypy --strict -p arris.build
