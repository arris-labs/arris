#!/usr/bin/env bash
# Tests `tools/verify.sh` (ADR-0078) in a scratch repository with stub test
# halves (`ARRIS_VERIFY_CMD`): a pass, a fail, a status read while a run is in
# flight, a second run waiting on the first. Runs no cargo.
#
#   tools/verify-test.sh
set -euo pipefail
src="$(git rev-parse --show-toplevel)"
verify="$src/tools/verify.sh"
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
fail=0
equals() { [ "$2" = "$3" ] || { echo "FAIL $1: expected '$3', got '$2'" >&2; fail=1; }; }

cd "$tmp"
git init -q -b main repo && cd repo
git config user.email t@t && git config user.name t
for i in 1 2 3 4; do echo "$i" >f; git add f; git commit -q -m "c$i"; done
c1=$(git rev-parse HEAD~3); c2=$(git rev-parse HEAD~2); c3=$(git rev-parse HEAD~1); c4=$(git rev-parse HEAD)
dir=$(realpath .git)/arris-verify

# stub: passes unless the commit's file says 2; sleeps when the file says 3
cat >"$tmp/stub" <<'STUB'
#!/usr/bin/env bash
echo "stub $1 $2 at $(cat f)"
[ "$(cat f)" = 3 ] && sleep 2
[ "$(cat f)" != 2 ]
STUB
chmod +x "$tmp/stub"
export ARRIS_VERIFY_CMD="$tmp/stub"

equals "none" "$("$verify" --status "$c1")" none

"$verify" "$c1"; equals "pass exit" $? 0
equals "pass state" "$("$verify" --status "$c1")" pass
grep -q "stub .* at 1" "$dir/$c1.log" || { echo "FAIL log" >&2; fail=1; }
equals "tree at sha" "$(git -C "$dir/tree" rev-parse HEAD)" "$c1"

"$verify" "$c2" && rc=0 || rc=$?
equals "fail exit" "$rc" 1
equals "fail state" "$("$verify" --status "$c2")" fail
equals "pass kept" "$("$verify" --status "$c1")" pass

# status while a run is in flight waits for its verdict; a second run waits
"$verify" "$c3" & p3=$!
sleep 0.5
equals "running" "$(cat "$dir/$c3")" running
"$verify" "$c4" & p4=$!
sleep 0.5
equals "queued" "$(cat "$dir/$c4")" queued
equals "status waits" "$("$verify" --status "$c3")" pass
equals "second still ran after" "$("$verify" --status "$c4")" pass
wait "$p3" "$p4"
equals "tree moved on" "$(git -C "$dir/tree" rev-parse HEAD)" "$c4"
[ ! -e "$dir/$c3.pid" ] || { echo "FAIL pid left" >&2; fail=1; }

[ "$fail" = 0 ] && echo "verify-test: ok" || exit 1
