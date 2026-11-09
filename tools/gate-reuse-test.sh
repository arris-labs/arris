#!/usr/bin/env bash
# Tests `tools/gate-reuse.sh` (ADR-0085 §3) in a scratch repository: a docs
# commit reuses, a code commit does not, a dirty code path refuses `--record`,
# and a red full gate records nothing (a stub `cargo` and the real
# `tools/full-gate.sh` stages replaced by a copy). Runs no cargo.
#
#   tools/gate-reuse-test.sh
set -euo pipefail
src="$(git rev-parse --show-toplevel)"
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
fail=0
equals() { [ "$2" = "$3" ] || { echo "FAIL $1: expected '$3', got '$2'" >&2; fail=1; }; }

cd "$tmp"
git init -q -b main repo && cd repo
git config user.email t@t && git config user.name t
mkdir -p tools crates/a docs tools/oracle
cp "$src/tools/gate-reuse.sh" tools/
reuse=tools/gate-reuse.sh
echo 1 >crates/a/lib.rs; echo d >docs/x.md; echo s >tools/x.sh
git add -A; git commit -q -m base; base=$(git rev-parse HEAD)

"$reuse" && rc=0 || rc=$?
equals "none recorded" "$rc" 1
"$reuse" --record
equals "record at HEAD" "$("$reuse")" "$base"

echo more >>docs/x.md; git commit -qam docs
equals "docs commit reuses" "$("$reuse")" "$base"
echo more >>tools/x.sh; git commit -qam shell
equals "a plain .sh reuses" "$("$reuse")" "$base"

for f in crates/a/lib.rs tools/y.py tools/oracle/z tools/check-q.sh tools/foo-gate.sh Cargo.toml; do
  git reset -q --hard "$base"; mkdir -p "$(dirname "$f")"; echo "$RANDOM" >>"$f"
  git add -A; git commit -qm code
  "$reuse" && rc=0 || rc=$?
  equals "code commit does not reuse ($f)" "$rc" 1
  "$reuse" --reaches-code "$base" HEAD && rc=0 || rc=$?
  equals "reaches-code ($f)" "$rc" 0
done
# a move out of a code path still reaches code (no rename following)
git reset -q --hard "$base"
git mv crates/a/lib.rs docs/lib.md; git commit -qm move
"$reuse" --reaches-code "$base" HEAD && rc=0 || rc=$?
equals "rename out of code reaches" "$rc" 0
git reset -q --hard "$base"
"$reuse" --reaches-code "$base" HEAD && rc=0 || rc=$?
equals "empty diff reaches none" "$rc" 1

echo dirty >>crates/a/lib.rs
"$reuse" --record 2>/dev/null && rc=0 || rc=$?
equals "dirty code path refuses" "$rc" 1
git checkout -q crates/a/lib.rs
echo n >notes.txt
"$reuse" --record && rc=0 || rc=$?
equals "dirty non-code path records" "$rc" 0
rm notes.txt

# a red gate records nothing: full-gate.sh with its stages stubbed
git rev-parse --git-path arris-verify >/dev/null
cp "$src/tools/full-gate.sh" tools/
for f in gate-scope.sh check-layers.sh; do
  printf '#!/usr/bin/env bash\n[ "%s" = gate-scope.sh ] && exec "$@"\nexit 0\n' "$f" >"tools/$f"; chmod +x "tools/$f"
done
mkdir -p bin
printf '#!/usr/bin/env bash\n[ -n "${STUB_MOVE:-}" ] && git commit -q --allow-empty -m moved\n[ -z "${STUB_RED:-}" ]\n' >bin/cargo; chmod +x bin/cargo
git add -A; git commit -qm gate-files
export PATH="$PWD/bin:$PATH" ARRIS_GATE_SCOPED=1
STUB_RED=1 tools/full-gate.sh "$tmp/red.log" >/dev/null && rc=0 || rc=$?
equals "red gate exits non-zero" "$rc" 1
[ ! -e ".git/arris-verify/gated/$(git rev-parse HEAD)" ] || { echo "FAIL red gate recorded" >&2; fail=1; }
tools/full-gate.sh "$tmp/green.log" >/dev/null && rc=0 || rc=$?
equals "green gate exits zero" "$rc" 0
[ -e ".git/arris-verify/gated/$(git rev-parse HEAD)" ] || { echo "FAIL green gate recorded nothing" >&2; fail=1; }
# HEAD moving during the run: nothing is recorded, and the run says so
git reset -q --hard HEAD
STUB_MOVE=1 tools/full-gate.sh "$tmp/move.log" >"$tmp/move.out" && rc=0 || rc=$?
equals "moved gate exits zero" "$rc" 0
grep -q "HEAD moved" "$tmp/move.out" || { echo "FAIL moved gate did not say so" >&2; fail=1; }
[ ! -e ".git/arris-verify/gated/$(git rev-parse HEAD)" ] || { echo "FAIL moved gate recorded" >&2; fail=1; }

[ "$fail" = 0 ] && echo "gate-reuse-test: ok"
exit "$fail"
