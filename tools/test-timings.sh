#!/usr/bin/env bash
# Wall-clock timings of the workspace's test suite, for the before/after of
# a change to how the suite is run (`docs/ARCHITECTURE.md` §Formats and
# tools). Not a benchmark harness: it times whole test binaries, not
# operations; the corpus's operations are timed by
# `tools/bench-compare.sh` (ADR-0024 §4).
#
#   tools/test-timings.sh                       256 cases (the default)
#   ARRIS_PROPTEST_CASES=1000 tools/test-timings.sh
#   tools/test-timings.sh --profile fast        one nextest profile only
#   tools/test-timings.sh --nextest-only        the nextest run, every profile's
#                                               tests, no per-binary section
#
# `--profile P` (a profile of `.config/nextest.toml`) times just the nextest
# run under that profile and the slowest single tests in it, so the profiles
# compare like with like (`docs/plans` lean-gate; ADR-0032). Without it the
# three measurements below are taken as before, plus the slowest tests.
#
# Prints a markdown table, slowest target first, of three measurements taken
# in one run so a before and an after compare like with like:
#
#   * every test binary run alone, one at a time — the per-target column,
#     and, summed, what `cargo test --workspace` costs, since cargo runs the
#     binaries one after another;
#   * the doctests, which no binary holds and `cargo nextest` does not run;
#   * `cargo nextest run --workspace`, which overlaps the binaries.
#
# Compilation is not timed: everything is built first. Needs `jq`.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

command -v jq >/dev/null || { echo "test-timings: needs jq" >&2; exit 2; }
command -v cargo-nextest >/dev/null || { echo "test-timings: needs cargo-nextest (AGENTS.md, \"Setup\")" >&2; exit 2; }

cases="${ARRIS_PROPTEST_CASES:-256}"
profile=""
nextest_only=0
while [ $# -gt 0 ]; do
  case "$1" in
    --profile) profile="${2:?--profile needs a name}"; nextest_only=1; shift 2 ;;
    --nextest-only) nextest_only=1; shift ;;
    *) echo "test-timings: unknown argument $1" >&2; exit 2 ;;
  esac
done

# Seconds with two decimals between two `date +%s%N` readings.
elapsed() { awk -v a="$1" -v b="$2" 'BEGIN { printf "%.2f", (b - a) / 1e9 }'; }

echo "test-timings: building" >&2
cargo test --workspace --no-run --quiet
cargo nextest run --workspace --no-run --cargo-quiet

rows=""
serial=0
doc=""
if [ "$nextest_only" -eq 0 ]; then
# Every test executable, with the package it belongs to and the directory
# cargo would run it in, so a test that reads a relative path sees what it
# sees under `cargo test`.
targets=$(
  cargo test --workspace --no-run --message-format=json 2>/dev/null |
    jq -r 'select(.reason == "compiler-artifact" and .executable != null)
           | [ (.manifest_path | sub("/Cargo.toml$"; "")), .target.name, .executable ]
           | @tsv' | sort
)

while IFS=$'\t' read -r dir name exe; do
  [ -n "${exe:-}" ] || continue
  crate=$(basename "$dir")
  echo "test-timings: $crate::$name" >&2
  start=$(date +%s%N)
  (cd "$dir" && ARRIS_PROPTEST_CASES="$cases" "$exe" --quiet >/dev/null)
  secs=$(elapsed "$start" "$(date +%s%N)")
  rows+="$secs|$crate::$name"$'\n'
  serial=$(awk -v a="$serial" -v b="$secs" 'BEGIN { printf "%.2f", a + b }')
done <<<"$targets"

echo "test-timings: doctests" >&2
start=$(date +%s%N)
ARRIS_PROPTEST_CASES="$cases" cargo test --workspace --doc --quiet >/dev/null
doc=$(elapsed "$start" "$(date +%s%N)")

fi

echo "test-timings: cargo nextest run --workspace${profile:+ --profile $profile}" >&2
pertest=$(mktemp)
start=$(date +%s%N)
ARRIS_PROPTEST_CASES="$cases" cargo nextest run --workspace --cargo-quiet \
  ${profile:+--profile "$profile"} --status-level pass --final-status-level none \
  --no-fail-fast >"$pertest" 2>&1 || true
nex=$(elapsed "$start" "$(date +%s%N)")

# Most targets are far below a second and would bury the two that are not,
# so everything under FLOOR is one summarising row.
FLOOR=0.10
sorted=$(mktemp)
trap 'rm -f "$sorted" "$pertest"' EXIT
printf '%s' "$rows" | sort -t'|' -k1,1gr >"$sorted"

echo
echo "\`ARRIS_PROPTEST_CASES=$cases\`, $(nproc) cores, $(date +%F), profile \`${profile:-default}\`."
echo
if [ "$nextest_only" -eq 0 ]; then
  echo "| Target, run alone | Seconds |"
  echo "|---|---|"
  rest=0
  count=0
  while IFS='|' read -r secs name; do
    if awk -v s="$secs" -v f="$FLOOR" 'BEGIN { exit !(s >= f) }'; then
      echo "| \`$name\` | $secs |"
    else
      count=$((count + 1))
      rest=$(awk -v a="$rest" -v b="$secs" 'BEGIN { printf "%.2f", a + b }')
    fi
  done <"$sorted"
  [ "$count" -gt 0 ] && echo "| *$count further targets, each under $FLOOR* | $rest |"
  echo "| **serial total** — what \`cargo test --workspace\` costs | **$serial** |"
  echo "| \`cargo test --workspace --doc\` | $doc |"
  echo
fi

# nextest prints `PASS [  12.345s] crate::bin test::name` for each test.
echo "| Slowest tests under nextest | Seconds |"
echo "|---|---|"
sed -E -n 's/^ *(PASS|FAIL|SIGABRT|TIMEOUT)[^[]*\[ *([0-9.]+)s\] +(\([0-9]+\/[0-9]+\) +)?(.*)$/\2 \4/p' "$pertest" |
  sort -gr | awk 'NR <= 25 { s = $1; $1 = ""; sub(/^ /, ""); print "| `" $0 "` | " s " |" }'
tests=$(grep -cE '^ *(PASS|FAIL) ' "$pertest" || true)
cpu=$(sed -E -n 's/^ *(PASS|FAIL)[^[]*\[ *([0-9.]+)s\].*/\2/p' "$pertest" | awk '{ t += $1 } END { printf "%.0f", t }')
echo
echo "| **\`cargo nextest run --workspace${profile:+ --profile $profile}\`** | |"
echo "|---|---|"
echo "| wall clock, seconds | **$nex** |"
echo "| tests run | $tests |"
echo "| summed test seconds (CPU) | $cpu |"
