#!/usr/bin/env bash
# Tests `tools/at-commit.sh` (ADR-0081) in a scratch repository: the command
# runs on the commit's own tree, not the checkout's; it holds the
# verification's lock and says so; a verification queues behind it. Runs no
# cargo.
#
#   tools/at-commit-test.sh
set -euo pipefail
src="$(git rev-parse --show-toplevel)"
at="$src/tools/at-commit.sh"
verify="$src/tools/verify.sh"
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
fail=0
equals() { [ "$2" = "$3" ] || { echo "FAIL $1: expected '$3', got '$2'" >&2; fail=1; }; }
export ARRIS_GATE_SCOPE=off

cd "$tmp"
git init -q -b main repo && cd repo
git config user.email t@t && git config user.name t
for i in 1 2; do echo "$i" >f; git add f; git commit -q -m "c$i"; done
c1=$(git rev-parse HEAD~1); c2=$(git rev-parse HEAD)
echo dirty >f # the checkout moves on; the run reads the commit

equals "reads its commit" "$("$at" "$c1" cat f 2>/dev/null)" 1
equals "checkout untouched" "$(cat f)" dirty
equals "lock flag" "$("$at" "$c1" sh -c 'echo $ARRIS_VERIFY_LOCKED' 2>/dev/null)" 1
equals "own build output" "$("$at" "$c1" sh -c 'echo $CARGO_TARGET_DIR' 2>/dev/null)" "$PWD/target/verify"

# a verification started during the run waits for it
"$at" "$c1" sleep 1.5 2>/dev/null & pa=$!
sleep 0.5
ARRIS_VERIFY_CMD=true "$verify" "$c2" & pv=$!
sleep 0.3
equals "verification queued" "$(cat .git/arris-verify/"$c2")" queued
wait "$pa" "$pv"
equals "then ran" "$("$verify" --status "$c2")" pass

"$at" nosuch true 2>/dev/null && rc=0 || rc=$?
equals "bad sha" "$rc" 2

[ "$fail" = 0 ] && echo "at-commit-test: ok" || exit 1
