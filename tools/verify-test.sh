#!/usr/bin/env bash
# Tests `tools/verify.sh` (ADR-0078) in a scratch repository with stub test
# halves (`ARRIS_VERIFY_CMD`): a pass, a fail, a status read while a run is in
# flight, a second run waiting on the first, the base widened past a red, and
# the queue (ADR-0085 §4): a queued run covered by the next, a covered red, a
# queued commit of another line left alone, `--in-flight`. Runs no cargo.
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
# main: c1..c8, the file saying 1 2 3 4 5 6 7 2; side: s1 off c4, saying 9
for i in 1 2 3 4 5 6 7 8; do
  [ "$i" = 8 ] && echo 2 >f || echo "$i" >f
  git add f; git commit -q -m "c$i"
  eval "c$i=\$(git rev-parse HEAD)"
done
git checkout -q -b side "$c4"; echo 9 >f; git commit -q -am s1; s1=$(git rev-parse HEAD); git checkout -q main
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
# c2 is red, so c3's base reaches back to c1, the nearest pass
grep -q "stub $c1 $c3 at 3" "$dir/$c3.log" || { echo "FAIL base after a red: $(cat "$dir/$c3.log")" >&2; fail=1; }
grep -q "stub $c3 $c4 at 4" "$dir/$c4.log" || { echo "FAIL base after a pass: $(cat "$dir/$c4.log")" >&2; fail=1; }

# the queue: with c3 running, c5 queues and c6 covers it; c6's base reaches
# back over c5 to c4, and c5 reads c6's verdict
"$verify" "$c3" & pb=$!
sleep 0.5
"$verify" "$c5" & p5=$!
sleep 0.3
"$verify" --in-flight "$c5" && rc=0 || rc=$?
equals "in flight while queued" "$rc" 0
"$verify" "$c6" & p6=$!
sleep 0.3
equals "middle covered" "$(cat "$dir/$c5")" "covered $c6"
"$verify" --in-flight "$c5" && rc=0 || rc=$?
equals "covered in flight" "$rc" 0
equals "covered reads the last's verdict" "$("$verify" --status "$c5")" pass
wait "$pb" "$p5" "$p6"
grep -q "covered by $c6" "$dir/$c5.log" || { echo "FAIL covered log: $(cat "$dir/$c5.log")" >&2; fail=1; }
! grep -q stub "$dir/$c5.log" || { echo "FAIL a covered run ran" >&2; fail=1; }
grep -q "stub $c4 $c6 at 6" "$dir/$c6.log" || { echo "FAIL base over covered: $(cat "$dir/$c6.log")" >&2; fail=1; }
[ ! -e "$dir/$c5.pid" ] || { echo "FAIL covered pid left" >&2; fail=1; }
"$verify" --in-flight "$c5" && rc=0 || rc=$?
equals "not in flight once done" "$rc" 1
"$verify" --in-flight "$c1" && rc=0 || rc=$?
equals "never queued is not in flight" "$rc" 1

# a covered red: c8 (red) covers c7, which reads fail
"$verify" "$c3" & pb=$!
sleep 0.5
"$verify" "$c7" & p7=$!
sleep 0.3
"$verify" "$c8" & p8=$!
wait "$pb" "$p7" "$p8" || true
equals "covered red" "$("$verify" --status "$c7")" fail
equals "covered red state" "$(cat "$dir/$c7")" "covered $c8"

# another line's queued commit is left alone: s1 is no descendant of c6
"$verify" "$c3" & pb=$!
sleep 0.5
"$verify" "$c6" & p6=$!
sleep 0.3
"$verify" "$s1" & ps=$!
wait "$pb" "$p6" "$ps"
equals "other line not covered" "$("$verify" --status "$c6")" pass
grep -q "stub .* $c6 at 6" "$dir/$c6.log" || { echo "FAIL other line's run: $(cat "$dir/$c6.log")" >&2; fail=1; }
grep -q "stub $c4 $s1 at 9" "$dir/$s1.log" || { echo "FAIL side base: $(cat "$dir/$s1.log")" >&2; fail=1; }

# a status read racing a run's end never reports its verdict as a death
# (a narrow window: this loop catches a regression only now and then):
# the runner writes the verdict, then removes its pid file
for i in 1 2 3 4 5 6 7 8 9 10; do
  "$verify" "$c1" & pv=$!
  seen=
  while kill -0 "$pv" 2>/dev/null; do seen="$seen $("$verify" --status "$c1")"; done
  wait "$pv"
  case "$seen" in *fail*) echo "FAIL race: a status read fail during a passing run:$seen" >&2; fail=1 ;; esac
done
# a runner that died without a verdict is still a fail
echo running >"$dir/$c1"; echo 999999 >"$dir/$c1.pid"
equals "dead runner" "$("$verify" --status "$c1")" fail
rm -f "$dir/$c1.pid"

[ "$fail" = 0 ] && echo "verify-test: ok" || exit 1
