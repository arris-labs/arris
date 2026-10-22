#!/usr/bin/env bash
# The full gate (`.agents/rules/git.md`): the layer rule, fmt, clippy -D
# warnings, nextest over the workspace at the default 256 cases with
# `real_*`, the doctests, rustdoc with warnings denied. Every stage runs even
# after one fails, so a single run names every red one. All output goes to
# <log>; the end prints each stage's verdict and the failing tests, and the
# exit code is non-zero when anything failed.
#
#   tools/full-gate.sh <log>
#
# About an hour here: run it with `run_in_background` and wait for its
# completion notification. Never `nohup … &`, which sends none. The
# pre-commit hook's `ARRIS_GATE=full` runs the same list, but stops at the
# first red stage.
set -uo pipefail
[ $# -eq 1 ] || { echo "usage: tools/full-gate.sh <log>" >&2; exit 2; }
log=$(realpath -m "$1")
cd "$(git rev-parse --show-toplevel)"
mkdir -p "$(dirname "$log")"
: >"$log"

# The hook's lock: one gate at a time, a commit's or this.
lock=$(git rev-parse --git-path arris-gate.lock)
exec 9>>"$lock"
if ! flock -n 9; then
  echo "full-gate: a gate is running (pid $(cat "$lock" 2>/dev/null)); wait with \`flock $lock true\`" >&2
  exit 1
fi
echo $$ >"$lock"

# Half the cores, as the hook; the case counts are the tests' own defaults.
export CARGO_BUILD_JOBS=${CARGO_BUILD_JOBS:-16}
export NEXTEST_TEST_THREADS=${NEXTEST_TEST_THREADS:-16}
export RAYON_NUM_THREADS=${RAYON_NUM_THREADS:-16}
unset ARRIS_PROPTEST_CASES ARRIS_DIFF_CASES

verdicts=()
stage() {
  local name=$1 start=$SECONDS
  shift
  echo "== $name" >>"$log"
  if "$@" >>"$log" 2>&1; then
    verdicts+=("ok   $name ($((SECONDS - start)) s)")
  else
    verdicts+=("FAIL $name ($((SECONDS - start)) s)")
  fi
}

stage layers tools/check-layers.sh
stage fmt cargo fmt --all -- --check
stage clippy cargo clippy --workspace --all-targets -- -D warnings
stage nextest cargo nextest run --workspace --no-fail-fast --cargo-quiet
stage doctest cargo test --workspace --doc --quiet
stage doc env RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps --quiet

printf '%s\n' "${verdicts[@]}" | tee -a "$log"
grep -E '^\s+(FAIL|SIGSEGV|SIGABRT|TIMEOUT) ' "$log" | sort -u
! printf '%s\n' "${verdicts[@]}" | grep -q '^FAIL'
