#!/usr/bin/env bash
# Tests `tools/gate.sh`, the pre-commit gate's classifier (ADR-0032): path
# sets in, the printed gate asserted. Runs no cargo and no test; the doc
# check works in a scratch repo so it reads a staged diff of its own.
#
#   tools/gate-test.sh
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
gate="$PWD/tools/gate.sh"
fail=0

# key <key> <gate output> — the value of one line.
key() { sed -n "s/^$1=//p" <<<"$2"; }

# expect <what> <value> <pattern> — value must contain the fixed string.
has() { [[ "$2" == *"$3"* ]] || { echo "FAIL $1: expected '$3' in: $2" >&2; fail=1; }; }
lacks() { [[ "$2" != *"$3"* ]] || { echo "FAIL $1: did not expect '$3' in: $2" >&2; fail=1; }; }
equals() { [ "$2" = "$3" ] || { echo "FAIL $1: expected '$3', got '$2'" >&2; fail=1; }; }

plan() { "$gate" plan "$@"; }

# A geom-only change: geom's dependents, the boolean, provenance and split
# areas.
out=$(plan crates/arris-geom/src/curve.rs); f=$(key filter "$out")
has "geom" "$f" "rdeps(arris-geom)"
has "geom" "$f" "boolean|provenance|split"
lacks "geom" "$f" "sweep"
lacks "geom" "$f" "all()"
equals "geom cases" "$(key cases "$out")" 32

# An ops boolean change.
f=$(key filter "$(plan crates/arris-ops/src/boolean/pave.rs)")
has "boolean" "$f" "rdeps(arris-ops)"
has "boolean" "$f" "boolean|provenance|split"
lacks "boolean" "$f" "transform"

# An ops mirror change: the transform area, not boolean.
f=$(key filter "$(plan crates/arris-ops/src/mirror.rs)")
has "mirror" "$f" "rdeps(arris-ops)"
has "mirror" "$f" "^(transform|"
lacks "mirror" "$f" "boolean"

# An ops offset change: the offset area alone.
f=$(key filter "$(plan crates/arris-ops/src/offset/vertices.rs)")
has "offset" "$f" "rdeps(arris-ops)"
has "offset" "$f" "^(offset|"
lacks "offset" "$f" "boolean"

# A loft change: the loft area alone, plus rdeps (the differential and the
# recipe properties draw a loft).
for p in crates/arris-ops/src/sweep/loft.rs crates/arris-ops/src/sweep/loft_exact.rs; do
  f=$(key filter "$(plan "$p")")
  has "loft $p" "$f" "rdeps(arris-ops)"
  has "loft $p" "$f" "^(loft|"
  lacks "loft $p" "$f" "sweep"
  lacks "loft $p" "$f" "all()"
  lacks "loft $p" "$f" "binary_id(arris::corpus)\""
done
# A sweep along a path: the sweep area alone.
f=$(key filter "$(plan crates/arris-ops/src/sweep/along.rs)")
has "along" "$f" "^(sweep|"
lacks "along" "$f" "loft"
# The rest of sweep/ and blend/ is the whole corpus, deliberately.
for p in crates/arris-ops/src/sweep/extrude.rs crates/arris-ops/src/sweep/exact.rs \
  crates/arris-ops/src/sweep/mod.rs crates/arris-ops/src/blend/stripe.rs; do
  has "whole $p" "$(key filter "$(plan "$p")")" "rdeps(arris-ops) | binary_id(arris::corpus)"
done

# An ops file with no area of its own: the whole corpus.
f=$(key filter "$(plan crates/arris-ops/src/primitive.rs)")
has "ops other" "$f" "rdeps(arris-ops) | binary_id(arris::corpus)"

# A test file of a crate: that crate alone, no dependents.
f=$(key filter "$(plan crates/arris-geom/tests/fit.rs)")
equals "tests dir" "$f" "package(arris-geom)"

# arris-debug: its dependents and the whole corpus.
f=$(key filter "$(plan crates/arris-debug/src/lib.rs)")
has "debug" "$f" "rdeps(arris-debug) | binary_id(arris::corpus)"

# The Python binding: whatever file under it, its own tests and no corpus
# (it adds no geometry; its Python suite is CI's job, not the hook's).
for p in crates/arris-py/src/model.rs crates/arris-py/Cargo.toml \
  crates/arris-py/python/arris/_arris.pyi crates/arris-py/pyproject.toml; do
  f=$(key filter "$(plan "$p")")
  equals "py $p" "$f" "rdeps(arris-py)"
done
equals "py tests" "$(key filter "$(plan crates/arris-py/tests/test_io.py)")" "package(arris-py)"
# Alongside a kernel crate it adds only itself.
f=$(key filter "$(plan crates/arris-py/src/io.rs crates/arris-geom/src/curve.rs)")
has "py+geom" "$f" "rdeps(arris-py)"
has "py+geom" "$f" "rdeps(arris-geom)"
lacks "py+geom" "$f" "all()"

# The arris crate: the whole package, corpus included.
f=$(key filter "$(plan crates/arris/tests/docs_refs.rs)")
has "arris" "$f" "package(arris) | binary_id(arris::corpus)"

# A fixture, by area directory.
f=$(key filter "$(plan tests/fixtures/transform/box-mirror/fixture.json)")
has "fixture" "$f" "package(arris)"
has "fixture" "$f" "^(transform|"
lacks "fixture" "$f" "boolean"
# A fixture directory with no area rule: everything.
equals "fixture geom" "$(key filter "$(plan tests/fixtures/geom/x.json)")" "all()"
f=$(key filter "$(plan tests/fixtures/split/box-mid/fixture.json)")
has "split fixture" "$f" "^(split|"

# Anything the table cannot place: the whole profile.
for p in .config/nextest.toml Cargo.toml Cargo.lock tools/gate.sh fuzz/a.rs \
  .githooks/pre-commit .github/workflows/ci.yml no/such/place.txt crates/stray.txt; do
  equals "unplaced $p" "$(key filter "$(plan "$p")")" "all()"
done

# Prose plus code: the code's gate and the docs tests.
f=$(key filter "$(plan docs/ROADMAP.md crates/arris-geom/src/curve.rs)")
has "prose+code" "$f" "rdeps(arris-geom)"
has "prose+code" "$f" "binary_id(arris::docs_refs)"
# Prose alone reaches no crate.
f=$(key filter "$(plan docs/ROADMAP.md)")
lacks "prose" "$f" "rdeps"
has "prose" "$f" "binary_id(arris::docs_refs)"

# Nothing staged: nothing selected.
equals "empty" "$(key filter "$("$gate" plan)")" "none()"

# The full gate: every test, 256 cases, whatever the paths.
out=$(ARRIS_GATE=full plan crates/arris-math/src/lib.rs)
equals "full filter" "$(key filter "$out")" "all()"
equals "full cases" "$(key cases "$out")" 256
equals "full flag" "$(key full "$out")" yes

# No crate path ever yields a gate that leaves its own crate's tests out:
# the filter names the crate (rdeps or package) or selects everything.
while IFS= read -r p; do
  c=${p#crates/}; c=${c%%/*}
  f=$(key filter "$(plan "$p")")
  case "$f" in
    all\(\)) ;;
    *"rdeps($c)"*|*"package($c)"*) ;;
    *) echo "FAIL coverage $p: $f leaves $c's tests out" >&2; fail=1 ;;
  esac
done < <(git ls-files crates)

# Drift: every path pattern of an `arris-ops` arm matches a tracked file, so
# an arm cannot outlive its file; every fixture directory is an area of
# gate.sh or deliberately whole.
while IFS= read -r pat; do
  # shellcheck disable=SC2086
  found=no
  for g in ${pat//|/ }; do
    # shellcheck disable=SC2254
    while IFS= read -r t; do
      case "${t#crates/arris-ops/}" in $g) found=yes; break ;; esac
    done < <(git ls-files crates/arris-ops/src)
  done
  [ "$found" = yes ] || { echo "FAIL drift: arm '$pat' matches no tracked file" >&2; fail=1; }
done < <(sed -n '/^        arris-ops)/,/^        arris-py)/p' "$gate" |
  sed -n 's/^ *\(src\/[^)]*\)) .*/\1/p')
whole_dirs="geom regression sheet"
for d in tests/fixtures/*/; do
  d=${d%/}; d=${d##*/}
  grep -qE "^ +[a-z|]*\b$d\b[a-z|]*\)" <(sed -n '/^    tests\/fixtures/,/whole=yes ;;/p' "$gate") ||
    [[ " $whole_dirs " == *" $d "* ]] ||
    { echo "FAIL drift: tests/fixtures/$d is neither an area nor deliberately whole" >&2; fail=1; }
done

# Doc comments: read from the staged diff, in a scratch repo.
scratch=$(mktemp -d)
trap 'rm -rf "$scratch"' EXIT
(
  cd "$scratch"
  git init -q . && git config user.email t@t && git config user.name t
  mkdir -p tools crates/arris-geom/src
  cp "$gate" tools/gate.sh
  printf 'pub fn f() {}\n' >crates/arris-geom/src/lib.rs
  git add . && git commit -q -m base
  doc() { tools/gate.sh | sed -n 's/^doc=//p'; }
  printf 'pub fn f() { let _x = 1; }\n' >crates/arris-geom/src/lib.rs
  git add -A; [ "$(doc)" = no ] || { echo "FAIL doc: a code-only diff is not a doc change" >&2; exit 1; }
  printf '/// Says what it guarantees.\npub fn f() {}\n' >crates/arris-geom/src/lib.rs
  git add -A; [ "$(doc)" = yes ] || { echo "FAIL doc: an added /// line is" >&2; exit 1; }
  git commit -q -m doc
  printf 'pub fn f() {}\n' >crates/arris-geom/src/lib.rs
  git add -A; [ "$(doc)" = yes ] || { echo "FAIL doc: a removed /// line is" >&2; exit 1; }
  git commit -q -m undoc
  printf '# Notes\n' >crates/arris-geom/README.md
  git add -A; [ "$(doc)" = yes ] || { echo "FAIL doc: a .md under crates/ is" >&2; exit 1; }
) || fail=1

[ "$fail" = 0 ] && echo "gate-test: ok" || exit 1
