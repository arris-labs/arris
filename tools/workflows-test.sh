#!/usr/bin/env bash
# Checks the GitHub workflows under `.github/workflows/` with `actionlint`
# when it is installed (ADR-0085 §1). Without it this says so and passes: the
# workflows run in CI, and nothing here stands in for them.
#
#   tools/workflows-test.sh
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

if ! command -v actionlint >/dev/null 2>&1; then
  echo "workflows-test: actionlint not installed; the workflows are checked in CI only"
  exit 0
fi
actionlint
echo "workflows-test: actionlint clean ($(ls .github/workflows/*.yml | wc -l) workflows)"
