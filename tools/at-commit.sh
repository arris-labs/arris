#!/usr/bin/env bash
# Runs a command on a commit's own tree, the verification's (ADR-0078,
# `tools/verify-tree.sh`), so the checkout moves on while it runs: the full
# gate of a plan's tip while the review's fixes are made (ADR-0081).
#
#   tools/at-commit.sh <sha> <command> [args…]
#
# The command runs in the tree detached at <sha>, so `tools/…` is that
# commit's own tool; a path it is given (a log) is best absolute. Its build
# output is `target/verify`. It holds the verification's lock for its run,
# as `tools/full-gate.sh` does in the checkout: a verification in flight
# finishes first and a new one queues behind it (ADR-0078 §5), and the
# command sees ARRIS_VERIFY_LOCKED so the full gate does not take the lock
# again. It does not take the hook's lock: commits go on beside it.
#
# Memory: it runs beside the agent's builds, as a verification does, so in a
# scope bounded as one (ADR-0078 §6): MemoryHigh 20G, MemoryMax 24G,
# ARRIS_VERIFY_MEMORY_HIGH / ARRIS_VERIFY_MEMORY_MAX override.
set -uo pipefail
[ $# -ge 2 ] || { echo "usage: tools/at-commit.sh <sha> <command> [args…]" >&2; exit 2; }
checkout=$(git rev-parse --show-toplevel) || exit 2
here=$(dirname "$(realpath "$0")")
sha=$(git rev-parse --verify -q "$1^{commit}") || { echo "at-commit: no such commit $1" >&2; exit 2; }
shift
if [ -z "${ARRIS_GATE_SCOPED:-}" ]; then
  export ARRIS_GATE_MEMORY_HIGH=${ARRIS_VERIFY_MEMORY_HIGH:-20G}
  export ARRIS_GATE_MEMORY_MAX=${ARRIS_VERIFY_MEMORY_MAX:-24G}
  ARRIS_GATE_SCOPED=1 exec "$here/gate-scope.sh" "$0" "$sha" "$@"
fi

dir=$(realpath "$(git rev-parse --git-common-dir)")/arris-verify
mkdir -p "$dir"
exec 8>>"$dir/lock"
flock -n 8 || { echo "at-commit: a verification is running; waiting for it" >&2; flock 8; }

tree=$("$here/verify-tree.sh" "$sha") || exit 1
cd "$tree" || exit 1
export ARRIS_VERIFY_LOCKED=1
export CARGO_TARGET_DIR=$checkout/target/verify
export CARGO_BUILD_JOBS=${CARGO_BUILD_JOBS:-16}
export NEXTEST_TEST_THREADS=${NEXTEST_TEST_THREADS:-16}
export RAYON_NUM_THREADS=${RAYON_NUM_THREADS:-16}
echo "at-commit: $(git log -1 --format='%h %s') in $tree" >&2
"$@"
