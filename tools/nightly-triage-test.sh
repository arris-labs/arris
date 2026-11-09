#!/usr/bin/env bash
# Tests `tools/nightly-triage.sh` on a scratch JUnit holding the message
# shapes of the nightly's failures (run 42659620551): each signature class,
# new against known, a known test that passes, the exit codes, a missing file.
#
#   tools/nightly-triage-test.sh
set -euo pipefail
script="$(git rev-parse --show-toplevel)/tools/nightly-triage.sh"
dir=$(mktemp -d); trap 'rm -rf "$dir"' EXIT
cd "$dir"
fail=0
ok() { echo "ok: $1"; }
bad() { echo "FAIL: $1"; fail=1; }

HP="thread 'p' (1) panicked at crates/arris-debug/src/prop.rs:607:25:"
cat > junit.xml <<X
<testsuites><testsuite name="nextest-run">
<testcase name="stem_known::shard_1" classname="arris-ops::blend_prop"><failure message="m" type="test failure with exit code 101">$HP
property failed in shard 1 of 4, chunk 32: nothing unchecked
S5? s5: f43 (plane) against f53 (NURBS) is not decided
minimal failing input: (
    3.0,
)
replay the chunk alone with ARRIS_PROPTEST_SEED=ab ARRIS_PROPTEST_CHUNK=32 ARRIS_PROPTEST_CASES=625</failure>
<system-err>prop-budget: ran 264 of 625 cases in 3600.1 s (stop: budget)</system-err></testcase>
<testcase name="stem_known::shard_2" classname="arris-ops::blend_prop"><failure message="m" type="test failure with exit code 101">$HP
property failed in shard 2 of 4, chunk 44: nothing unchecked
S5? s5: f47 (plane) against f55 (NURBS) is not decided
minimal failing input: (1.0)</failure>
<system-err>prop-budget: ran 300 of 625 cases in 3500.0 s (stop: budget)</system-err></testcase>
<testcase name="stem_new" classname="arris-ops::split_prop"><failure message="m" type="test failure with exit code 101">$HP
property failed in shard 0 of 1, chunk 3: the sides: volumes: 98.8 vs 98.9 at crates/x.rs:236
minimal failing input: Case {}
replay the chunk alone with ARRIS_PROPTEST_SEED=ab ARRIS_PROPTEST_CHUNK=3 ARRIS_PROPTEST_CASES=625</failure></testcase>
<testcase name="variant_known" classname="arris-ops::sweep_prop"><failure message="m" type="test failure with exit code 101">$HP
property failed in shard 0 of 1, chunk 3: refused: Internal(Split) on f3
minimal failing input: 1</failure></testcase>
<testcase name="checker_known" classname="arris-ops::loft_prop"><failure message="m" type="test failure with exit code 101">$HP
property failed in shard 0 of 1, chunk 3: checker report: L5 f46: loop 0 intersects itself
S5 s14: f3 and f50 intersect
minimal failing input: 1</failure></testcase>
<testcase name="timeout_known::shard_3" classname="arris-ops::sweep_prop"><failure type="test timeout"></failure></testcase>
<testcase name="timeout_new::shard_3" classname="arris-ops::sweep_prop"><failure type="test timeout"></failure></testcase>
<testcase name="abort_new" classname="arris-geom::cone_prop"><failure message="m">$HP
property aborted in shard 0 of 1, chunk 3: Too many global rejects</failure></testcase>
<testcase name="kernel_panic" classname="arris-ops::blend_prop"><failure message="m">$HP
property failed in shard 0 of 1, chunk 3: Test failed: index out of bounds
minimal failing input: 1</failure><system-err>thread 'p' (2) panicked at crates/arris-ops/src/blend/walk.rs:812:9:
index out of bounds
</system-err></testcase>
<testcase name="passes_though_listed" classname="arris-ops::blend_prop"><system-err>prop-budget: ran 625 of 625 cases in 9.0 s (stop: ceiling)</system-err></testcase>
</testsuite></testsuites>
X
cat > known.txt <<'X'
# slug source class key
open-s5 arris-ops::blend_prop::stem_known assert nothing unchecked S
open-variant arris-ops::sweep_prop::variant_known variant Internal(Split)
open-checker arris-ops::loft_prop::checker_known checker L5
open-timeout arris-ops::sweep_prop::timeout_known timeout -
open-pass arris-ops::blend_prop::passes_though_listed assert whatever
X

touch none.txt
run() { set +e; "$script" "$@" >out.txt 2>err.txt; code=$?; set -e; }
has() { grep -qF -- "$1" out.txt && ok "$2" || { bad "$2 (no '$1')"; sed 's/^/    /' out.txt; }; }
hasnt() { grep -qF -- "$1" out.txt && { bad "$2 (found '$1')"; } || ok "$2"; }

run junit.xml known.txt
[ "$code" -eq 1 ] && ok "a new signature exits 1" || bad "exit $code, wanted 1"
has '`open-s5`: 2' "a known assert counts by slug, two shards"
has '`open-variant`: 1' "a known variant"
has '`open-checker`: 1' "a known checker row"
has '`open-timeout`: 1' "a known timeout"
hasnt 'open-pass' "a passing known test is quiet"
has 'arris-ops::split_prop::stem_new` assert `the sides: volumes' "a new assert prints its stem, shard suffix dropped"
has 'arris-ops::sweep_prop::timeout_new` timeout `-`' "a new timeout"
has 'arris-geom::cone_prop::abort_new` abort `-`' "a new abort"
has 'arris-ops::blend_prop::kernel_panic` panic `crates/arris-ops/src/blend/walk.rs:812`' "a kernel panic names its site, not the harness's"
hasnt 'prop.rs:607' "the harness's own panic site is no key"
has 'ARRIS_PROPTEST_SEED=ab ARRIS_PROPTEST_CHUNK=3 ARRIS_PROPTEST_CASES=625' "a replay line is kept when printed"
has 'prop-budget: 3 shards ran 1189 of 1875 cases' "the budget totals"

# A harness-only panic with a bare assertion message is an assert, not a panic.
cat > j2.xml <<X
<testsuites><testsuite><testcase name="a::shard_0" classname="c::b"><failure message="m">$HP
property failed in shard 0 of 4, chunk 3: assertion failed: \`(left == right)\`
  left: \`2\`,
 right: \`1\`: [Nurbs(NurbsCurve { degree: 5 }]
minimal failing input: 1</failure></testcase></testsuite></testsuites>
X
run j2.xml none.txt
has 'c::b::a` assert `assertion failed: `(left == right)` left`' "a harness-only panic is an assert keyed by its stem"

# The differential: one signature per case line, the class not the counts.
cat > j3.xml <<X
<testsuites><testsuite><testcase name="arris_and_the_oracle_agree" classname="arris::differential"><failure message="m">thread 't' panicked at crates/arris/tests/differential.rs:19:5:
differential: 1000 recipes
2 failing recipes:

case 222: Disagree(counts): Arris 16 vertices against 14
shrunk in 47 candidates: x

case 598: Internal: Internal(Split)
shrunk in 3 candidates: y
</failure></testcase></testsuite></testsuites>
X
run j3.xml none.txt
has 'differential `Disagree(counts)`' "a differential disagreement by its class"
has 'differential `Internal(Split)`' "a differential fault by its token"
printf 'open-diff arris::differential::arris_and_the_oracle_agree differential Disagree(counts)\n' > k3.txt
run j3.xml k3.txt
has '`open-diff`: 1' "a known differential class is counted"
has 'differential `Internal(Split)`' "another class in the same run stays new"

# Only known failures: green.
cat > j4.xml <<X
<testsuites><testsuite><testcase name="t::shard_0" classname="c::b"><failure type="test timeout"></failure></testcase></testsuite></testsuites>
X
printf 'x c::b::t timeout -\n' > k4.txt
run j4.xml k4.txt
[ "$code" -eq 0 ] && ok "only known signatures exit 0" || bad "exit $code, wanted 0"
has 'No new signature' "and says so"

GITHUB_STEP_SUMMARY=$dir/sum.md run j4.xml k4.txt
grep -q 'Nightly triage' sum.md && ok "the summary file is written" || bad "no summary file"

run missing.xml k4.txt
[ "$code" -eq 1 ] && ok "a missing JUnit is red" || bad "exit $code, wanted 1"
echo '<not xml' > broken.xml
run broken.xml k4.txt
[ "$code" -eq 1 ] && ok "an unreadable JUnit is red" || bad "exit $code, wanted 1"

[ "$fail" -eq 0 ] && echo "nightly-triage-test: all passed" || { echo "nightly-triage-test: FAILED"; exit 1; }
