#!/usr/bin/env bash
# The oracle's own check, sized for a commit: `selftest.py` over the
# `primitive/` fixtures (ADR-0085 §1). The whole selftest takes 11 minutes and
# stays CI's. Without `uv` this says so and passes.
#
#   tools/oracle-test.sh
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

if ! command -v uv >/dev/null 2>&1; then
  echo "oracle-test: uv not installed; the oracle's selftest is CI's only"
  exit 0
fi
dirs=(tests/fixtures/primitive/*/)
uv run --project tools/oracle tools/oracle/selftest.py "${dirs[@]}"
