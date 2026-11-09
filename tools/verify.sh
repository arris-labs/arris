#!/usr/bin/env bash
# Verifies a commit: its test half (ADR-0078) run on the commit's own tree,
# after the commit, and the verdict kept where the hooks read it.
#
#   tools/verify.sh <sha>               run the test half of <sha>, record the state
#   tools/verify.sh --status [<sha>]    print the state (default HEAD); waits
#                                       while a run of <sha> is queued or running
#   tools/verify.sh --in-flight [<sha>] exit 0 while a run of <sha> is queued
#                                       or running with a live runner; no wait
#   tools/verify.sh --base [<sha>]      print the base a commit on top of <sha>
#                                       would be tested over (the hook's)
#
# The state is a file per sha in `arris-verify/` of the common git dir:
# `<sha>` holds `queued`, `running`, `pass`, `fail` or `covered <newer-sha>`,
# `<sha>.log` the output, `<sha>.pid` the runner while it lives. `--status`
# prints `none` for a sha never verified. One verification runs at a time
# (`arris-verify/lock`, which `tools/full-gate.sh` and `tools/at-commit.sh`
# hold for their runs too); a second waits its turn.
#
# The queue (ADR-0085 §4): `arris-verify/newest` names the commit queued
# last. A new run whose predecessor there is still `queued` and is its
# ancestor writes `covered <new-sha>` to it and takes its place; a commit of
# another line is left alone. The covered run, given the lock, reads
# `covered` and exits; the newer run's base reaches back over it, so its
# verdict is the covered commit's too, and `--status` prints that verdict.
# Covering and `queued` -> `running` happen under `arris-verify/queue.lock`,
# a short lock, so a run is never covered once it has started.
#
# The base is read after the lock: the nearest `pass` on the first-parent
# walk from `<sha>^`, at most 200 commits, else the empty tree. With a
# verified parent it is `<sha>^`; after a red or a covered run it is wider.
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

# The state of a commit with `covered` followed to the run that holds its
# verdict; `none` for one never verified. A chain is short (one hop per
# queued commit); the bound only keeps a corrupt file from looping.
resolve() {
  local c=$1 s n=0
  while [ "$n" -lt 1000 ]; do
    s=$(cat "$dir/$c" 2>/dev/null || echo none)
    case "$s" in
      "covered "*) c=${s#covered }; n=$((n + 1)) ;;
      *) echo "$c $s"; return ;;
    esac
  done
  echo "$c fail"
}

# The nearest `pass` on the first-parent walk from the given commit (itself
# included), at most 200 commits; the empty tree when none is within reach.
base_from() {
  local c s
  for c in $(git rev-list --first-parent --max-count=200 "$1"); do
    read -r _ s <<<"$(resolve "$c")"
    if [ "$s" = pass ]; then echo "$c"; return; fi
  done
  git hash-object -t tree /dev/null
}

case "${1:-}" in
  --base)
    sha=$(git rev-parse --verify -q "${2:-HEAD}^{commit}") || { echo "verify: no such commit ${2:-HEAD}" >&2; exit 2; }
    base_from "$sha"
    exit 0
    ;;
  --status)
    sha=$(git rev-parse --verify -q "${2:-HEAD}^{commit}") || { echo "verify: no such commit ${2:-HEAD}" >&2; exit 2; }
    while :; do
      read -r run state <<<"$(resolve "$sha")"
      case "$state" in
        queued | running)
          pid=$(cat "$dir/$run.pid" 2>/dev/null || true)
          if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then sleep 0.2; continue; fi
          # The runner writes its verdict (or reads `covered`) and then
          # removes its pid file: a state read before that and a pid read
          # after it is not a death. Read the state again.
          read -r run state <<<"$(resolve "$sha")"
          case "$state" in
            queued | running) state=fail ;; # died without a verdict
            *) continue ;;
          esac
          ;;
      esac
      break
    done
    echo "$state"
    exit 0
    ;;
  --in-flight)
    sha=$(git rev-parse --verify -q "${2:-HEAD}^{commit}") || { echo "verify: no such commit ${2:-HEAD}" >&2; exit 2; }
    read -r run state <<<"$(resolve "$sha")"
    case "$state" in
      queued | running)
        pid=$(cat "$dir/$run.pid" 2>/dev/null || true)
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then exit 0; fi
        read -r run state <<<"$(resolve "$sha")" # as in --status: not a death
        case "$state" in
          queued | running) exit 1 ;;
          *) exec "$0" --in-flight "$sha" ;;
        esac
        ;;
    esac
    exit 1
    ;;
esac

[ $# -eq 1 ] || { echo "usage: tools/verify.sh <sha> | --status [<sha>] | --in-flight [<sha>] | --base [<sha>]" >&2; exit 2; }
sha=$(git rev-parse --verify -q "$1^{commit}") || { echo "verify: no such commit $1" >&2; exit 2; }
log=$dir/$sha.log

# Queue: cover the commit queued before this one when it is still waiting and
# is this one's ancestor, then take its place.
exec 9>>"$dir/queue.lock"
flock 9
echo $$ >"$dir/$sha.pid"
echo queued >"$dir/$sha"
: >"$log"
prev=$(cat "$dir/newest" 2>/dev/null || true)
if [ -n "$prev" ] && [ "$prev" != "$sha" ] && [ "$(cat "$dir/$prev" 2>/dev/null)" = queued ] &&
  git merge-base --is-ancestor "$prev" "$sha" 2>/dev/null; then
  echo "covered $sha" >"$dir/$prev"
  echo "verify: covers ${prev:0:8}" >>"$log"
fi
echo "$sha" >"$dir/newest"
flock -u 9
exec 9>&-

exec 8>>"$dir/lock"
flock 8 # one at a time; a second run waits here

exec 9>>"$dir/queue.lock"
flock 9
state=$(cat "$dir/$sha" 2>/dev/null || true)
case "$state" in
  "covered "*)
    echo "verify: covered by ${state#covered }" >>"$log"
    rm -f "$dir/$sha.pid"
    exit 0
    ;;
esac
echo running >"$dir/$sha"
flock -u 9
exec 9>&-

# The base: the nearest verified commit on this line, so a run after a red
# or over covered commits tests everything since the last `pass`.
if parent=$(git rev-parse --verify -q "$sha^"); then
  base=$(base_from "$parent")
else
  base=$(git hash-object -t tree /dev/null)
fi
echo "verify: test half over ${base:0:8}..${sha:0:8}" >>"$log"

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
