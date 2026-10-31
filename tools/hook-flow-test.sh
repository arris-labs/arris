#!/usr/bin/env bash
# Tests the three hooks (ADR-0078) in a scratch repository with stub halves
# (ARRIS_STATIC_CMD, ARRIS_DOCS_CMD, ARRIS_TEST_HALF_CMD, ARRIS_VERIFY_CMD):
# a deferred commit returns before its test half ends; a red parent makes the
# next commit run inline over both; a prose commit takes its parent's state;
# a push of an unverified tip is refused. Runs no cargo.
#
#   tools/hook-flow-test.sh
set -euo pipefail
src="$(git rev-parse --show-toplevel)"
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
fail=0
equals() { [ "$2" = "$3" ] || { echo "FAIL $1: expected '$3', got '$2'" >&2; fail=1; }; }

cd "$tmp"
git init -q -b main repo && cd repo
git config user.email t@t && git config user.name t
mkdir tools
cp -r "$src/.githooks" .githooks
cp "$src/tools/verify.sh" "$src/tools/verify-tree.sh" "$src/tools/gate-scope.sh" tools/
git config core.hooksPath .githooks
echo '[workspace]' >Cargo.toml
dir=$(realpath .git)/arris-verify
calls=$tmp/calls

# the stub for the inline run (args: base) and the verification (base, sha):
# records its arguments, sleeps when the code says "slow", fails when "red"
cat >"$tmp/stub" <<'STUB'
#!/usr/bin/env bash
echo "$# $(cat code.rs)" >>"$CALLS"
echo "${1:-}" >>"$CALLS.base"
! grep -q slow code.rs || sleep 2
! grep -q red code.rs
STUB
chmod +x "$tmp/stub"
export CALLS=$calls ARRIS_GATE_SCOPE=off ARRIS_STATIC_CMD=true ARRIS_DOCS_CMD=true
export ARRIS_TEST_HALF_CMD="$tmp/stub" ARRIS_VERIFY_CMD="$tmp/stub"

commit() { # content file
  printf '%s\n' "$1" >"$2"; git add "$2"; git commit -q -m "$1"
}
settle() { tools/verify.sh --status "$1"; }
sha() { git rev-parse "$1"; }

# 1. the first commit has no parent: inline over everything, then pass
commit ok code.rs
equals "first inline" "$(settle HEAD)" pass
equals "first ran inline" "$(cat "$calls")" "1 ok"

# 2. HEAD is pass: the next test half is deferred and the commit returns first
: >"$calls"
start=$(date +%s.%N)
commit slow code.rs
took=$(awk -v a="$start" -v b="$(date +%s.%N)" 'BEGIN { print (b - a < 1.5) ? "fast" : "slow" }')
equals "deferred returns before the test half ends" "$took" fast
equals "deferred is queued or running" "$(cat "$dir/$(sha HEAD)")" "$(cat "$dir/$(sha HEAD)" | grep -E "^(queued|running)$")"
equals "status waits for it" "$(settle HEAD)" pass
equals "verification ran with base and sha" "$(cat "$calls")" "2 slow"

# 3. a prose commit takes its parent's state
commit note notes.md
equals "prose inherits pass" "$(settle HEAD)" pass

# 4. a red verification, then the next commit runs inline over both
good=$(sha HEAD)
commit red code.rs
equals "red verification" "$(settle HEAD)" fail
red=$(sha HEAD)
: >"$calls"; : >"$calls.base"
commit fixed code.rs
equals "inline over the red" "$(cat "$calls")" "1 fixed"
equals "inline base is the last verified commit" "$(cat "$calls.base")" "$good"
equals "inline commit is verified" "$(settle HEAD)" pass

# 5. a red verification, then a red inline commit is refused by the hook
commit red-again code.rs # deferred from a pass parent, so a verification
equals "red again" "$(settle HEAD)" fail
if commit red-twice code.rs 2>/dev/null; then equals "inline red refused" committed refused; fi
git reset -q --hard HEAD # the refused commit left its change staged
commit "ok-again" code.rs
equals "inline over the red passes" "$(settle HEAD)" pass
commit red-three code.rs # deferred, red
equals "red three" "$(settle HEAD)" fail

# 6. a prose commit on a red parent inherits the red
commit prose2 notes.md
equals "prose inherits fail" "$(settle HEAD)" fail

# 7. push: an unverified tip is refused, a verified one is not
zero=0000000000000000000000000000000000000000
push() { printf 'refs/heads/main %s refs/heads/main %s\n' "$1" "$zero" | .githooks/pre-push 2>/dev/null; }
push "$(sha HEAD)" && rc=0 || rc=$?
equals "red tip refused" "$rc" 1
push "$(sha HEAD~2)" && rc=0 || rc=$?
equals "verified tip pushed" "$rc" 0
printf 'refs/heads/gone 0000000000000000000000000000000000000000 refs/heads/gone %s\n' "$zero" | .githooks/pre-push && rc=0 || rc=$?
equals "delete not checked" "$rc" 0

# 8. ARRIS_VERIFY=inline never defers
git checkout -q -B main "$(sha HEAD~2)" # back to the verified commit
: >"$calls"
ARRIS_VERIFY=inline commit fine2 code.rs
equals "inline mode ran the test half" "$(wc -l <"$calls" | tr -d ' ')" 1
equals "inline mode verified" "$(settle HEAD)" pass

[ "$fail" = 0 ] && echo "hook-flow-test: ok" || exit 1
