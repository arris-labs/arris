#!/usr/bin/env bash
# Verifies a commit: its test half (ADR-0078) run on the commit's own tree,
# after the commit, and the verdict kept where the hooks read it.
#
#   tools/verify.sh <sha>            run the test half of <sha>, record the state
#   tools/verify.sh --status [<sha>] print the state (default HEAD); waits
#                                    while a run of <sha> is queued or running
#
# The state is a file per sha in `arris-verify/` of the common git dir:
# `<sha>` holds `queued`, `running`, `pass` or `fail`, `<sha>.log` the output,
# `<sha>.pid` the runner while it lives. `--status` prints `none` for a sha
# never verified. One verification runs at a time (`arris-verify/lock`, which
# `tools/full-gate.sh` and `tools/at-commit.sh` hold for their runs too); a
# second waits its turn.
#
# The tree is one linked worktree, `arris-verify/tree`, detached at each sha
# in turn (`tools/verify-tree.sh`, which says why its build output is
# `<checkout>/target/verify`). `tools/at-commit.sh` runs other commands on it
# under the same lock.
#
# Memory (ADR-0078): the run is in its own scope bounded lower than the
# hook's, MemoryHigh 20G and MemoryMax 24G, so a verification beside the
# agent's builds (their own scope, 48G) leaves the 61 GiB machine room.
# ARRIS_VERIFY_MEMORY_HIGH / ARRIS_VERIFY_MEMORY_MAX override.
# ARRIS_VERIFY_CMD replaces the test half (`<cmd>` runs in the tree with the
# base and sha as arguments): the seam `tools/verify-test.sh` uses.
# The body is one brace group, which bash reads whole before running it:
# an edit to this file while it runs (a verification, a gate) cannot
# reach the run.
{
set -uo pipefail
checkout=$(git rev-parse --show-toplevel) || exit 2
common=$(realpath "$(git rev-parse --git-common-dir)")
dir=$common/arris-verify
mkdir -p "$dir"

if [ "${1:-}" = --status ]; then
  sha=$(git rev-parse --verify -q "${2:-HEAD}^{commit}") || { echo "verify: no such commit ${2:-HEAD}" >&2; exit 2; }
  while :; do
    state=$(cat "$dir/$sha" 2>/dev/null || echo none)
    case "$state" in
      queued | running)
        pid=$(cat "$dir/$sha.pid" 2>/dev/null || true)
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then sleep 0.2; continue; fi
        # The runner writes its verdict and then removes its pid file: a
        # state read before the verdict and a pid read after it is not a
        # death. Read the state again.
        state=$(cat "$dir/$sha" 2>/dev/null || echo none)
        case "$state" in queued | running) state=fail ;; esac # died without a verdict
        ;;
    esac
    break
  done
  echo "$state"
  exit 0
fi

[ $# -eq 1 ] || { echo "usage: tools/verify.sh <sha> | --status [<sha>]" >&2; exit 2; }
sha=$(git rev-parse --verify -q "$1^{commit}") || { echo "verify: no such commit $1" >&2; exit 2; }
log=$dir/$sha.log
echo $$ >"$dir/$sha.pid"
echo queued >"$dir/$sha"
: >"$log"

exec 8>>"$dir/lock"
flock 8 # one at a time; a second run waits here
echo running >"$dir/$sha"

verdict=fail
{
  tree=$("$(dirname "$(realpath "$0")")/verify-tree.sh" "$sha") || exit 1
  cd "$tree" || exit 1
  export CARGO_TARGET_DIR=$checkout/target/verify
  export CARGO_BUILD_JOBS=${CARGO_BUILD_JOBS:-16}
  export NEXTEST_TEST_THREADS=${NEXTEST_TEST_THREADS:-16}
  export RAYON_NUM_THREADS=${RAYON_NUM_THREADS:-16}
  export ARRIS_GATE_MEMORY_HIGH=${ARRIS_VERIFY_MEMORY_HIGH:-20G}
  export ARRIS_GATE_MEMORY_MAX=${ARRIS_VERIFY_MEMORY_MAX:-24G}
  base=$(git rev-parse --verify -q "$sha^" || git hash-object -t tree /dev/null)
  if [ -n "${ARRIS_VERIFY_CMD:-}" ]; then
    $ARRIS_VERIFY_CMD "$base" "$sha"
  else
    "$checkout/tools/gate-scope.sh" tools/test-half.sh "$base" "$sha"
  fi
} >>"$log" 2>&1 && verdict=pass

echo "$verdict" >"$dir/$sha"
rm -f "$dir/$sha.pid"
[ "$verdict" = pass ]
exit
}
