#!/usr/bin/env bash
# The pre-commit gate a set of paths needs, printed as `key=value` lines
# (ADR-0032). The classifier and nothing else: it runs no test, and it is the
# only place the rule lives, so the hook, the skills and `tools/gate-test.sh`
# agree.
#
#   tools/gate.sh                  the staged paths
#   tools/gate.sh plan <path>…     these paths (`doc` still reads the staged diff)
#   tools/gate.sh since <base> [<sha>]
#                                  the paths changed from <base> to <sha> (the
#                                  index when no <sha>), and `doc` read from
#                                  that same diff: the gate of a range, which
#                                  the verification of a commit (`<sha>^ <sha>`)
#                                  and a red's wider re-run (`<verified> HEAD`)
#                                  use (ADR-0078)
#
# Keys:
#   filter      a nextest filterset for `--profile fast -E`
#   cases       ARRIS_PROPTEST_CASES
#   diff_cases  ARRIS_DIFF_CASES
#   doc         yes when `cargo doc` is worth its 30 s: the staged diff adds
#               or removes a `///` or `//!` line, or a `.md` under `crates/`
#               (they are `include_str!`-ed) changed
#   full        yes under `ARRIS_GATE=full`: the old gate, every test at 256
#   tool_tests  the `tools/*-test.sh` scripts the changed tools have, and the
#               ones a placed config or workflow names, space separated;
#               `none()` with no prose means nothing else runs
#
# Classification, from the path:
#   crates/<c>/src (and Cargo.toml)   <c> and everything that depends on it
#                                     (`rdeps`), and the corpus areas below
#   crates/<c>/{tests,benches,examples}   <c> alone
#   crates/arris-{math,geom,topo,check,io}, arris-ops/src/boolean
#                                     corpus `boolean_*`, `provenance_*` and
#                                     `split_*` (split is built over the boolean)
#   arris-ops/src/sweep/{loft,loft_exact}.rs   the `loft` corpus area
#   arris-ops/src/sweep/along.rs      the `sweep` area (it uses `exact`, which
#                                     uses `revolve`, but its own change reaches
#                                     only sweeps along a path)
#   arris-ops/src/{offset,shell,transform,mirror}   that area's corpus tests
#                                     (offset/ reaches shell/ too: shell is built over it)
#   arris-ops/src/blend/              the `blend` area and the rows of every
#                                     fixture outside blend/, real/ and
#                                     regression/ that has a fillet or chamfer
#                                     op (named by test); the whole corpus when
#                                     anything outside blend/ names it (ADR-0085)
#   arris-ops/src/sweep otherwise     the whole corpus, on purpose
#   any other crate source (mesh, debug, the rest of ops)   the whole corpus
#   crates/arris-py/ (anything)       its own tests and its rdeps (none), no
#                                     corpus: the binding adds no geometry, and
#                                     the Python suite is CI's (ADR-0034 §4)
#   crates/arris/                     the whole `arris` package, corpus included
#   tests/fixtures/<area>/            that area's corpus tests, the
#                                     cancel record's name check for the
#                                     areas `arris-ops/tests/cancel.rs` runs,
#                                     and the STEP round trip of the staged
#                                     fixtures alone (`read_back=`, the
#                                     hook's `ARRIS_READ_BACK`, read by
#                                     `arris-io/tests/step_read.rs`)
#   prose (`*.md`, docs/, .agents/, .claude/)   the docs tests, nothing else
#   .github/*                         no nextest: the workflows run in CI, and
#                                     `tools/workflows-test.sh` (actionlint)
#                                     is their local check (ADR-0085 §1)
#   .config/nextest.toml              no nextest: `tools/profile-test.sh` holds
#                                     the `fast` filter to its slow set
#   tests/fixtures/regression/<slug>/ the corpus lint, the test that every
#                                     directory has a corpus test, and each
#                                     file naming `regression/<slug>` (a
#                                     tests/<f>.rs its binary, else its
#                                     package; corpus.rs's ignored rows are
#                                     the lint's business) (ADR-0085 §1)
#   tools/oracle/*                    `arris` and `arris-debug` whole, the
#                                     test binaries that call the oracle, and
#                                     `tools/oracle-test.sh` (ADR-0085 §1)
#   tools/<name>.sh (top level)       no nextest and no doctests: no Rust test
#                                     runs a top-level script; its own
#                                     `tools/<name>-test.sh` when there is one
#                                     (`tool_tests=`). The other files under
#                                     tools/ are read by tests and stay
#                                     `anything else`
#   anything else                     every test of the `fast` profile
#
# A path the table cannot place costs the whole profile, never less; nothing
# here yields a gate smaller than the changed crate's own tests. The corpus
# (`arris::corpus`) sits in `arris`, which depends on every crate, so a
# crate's `rdeps` reaches all of it; a partial area list subtracts it and adds
# the named areas back. `real_*` and the slow set are the profile's business.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

# The diff `doc` reads: the index, or the range given to `since`.
diffargs=(--cached)
if [ "${1:-}" = plan ]; then
  shift
  paths=$(printf '%s\n' "$@")
elif [ "${1:-}" = since ]; then
  [ $# -ge 2 ] && [ $# -le 3 ] || { echo "usage: gate.sh since <base> [<sha>]" >&2; exit 2; }
  if [ $# -eq 3 ]; then diffargs=("$2" "$3"); else diffargs=(--cached "$2"); fi
  paths=$(git diff "${diffargs[@]}" --name-only --diff-filter=ACMRD)
else
  paths=$(git diff --cached --name-only --diff-filter=ACMRD)
fi

CORPUS='binary_id(arris::corpus)'
# Corpus tests that are cheap and not an area of their own.
BASE='a|an|build|primitive'

terms=()        # filterset terms over packages
areas=()        # corpus test-name prefixes
corpus=none     # none | part | all
whole=no
prose=no
doc=no
read_back=()    # staged fixtures whose area has a STEP round trip
tool_tests=()   # tools/<name>-test.sh of the changed tools/<name>.sh
blend=no        # a file of arris-ops/src/blend/ changed
oracle=no       # a file of tools/oracle/ changed
slugs=()        # regression/<slug> directories with a changed file

add_term() {
  local t
  for t in ${terms[@]+"${terms[@]}"}; do [ "$t" = "$1" ] && return 0; done
  terms+=("$1")
}
add_area() {
  local a
  for a in ${areas[@]+"${areas[@]}"}; do [ "$a" = "$1" ] && return 0; done
  areas+=("$1")
}
add_tool_test() {
  local t
  for t in ${tool_tests[@]+"${tool_tests[@]}"}; do [ "$t" = "$1" ] && return 0; done
  tool_tests+=("$1")
}
need_corpus() { [ "$corpus" = all ] || corpus=$1; }

while IFS= read -r p; do
  [ -n "$p" ] || continue
  case "$p" in
    crates/*/*)
      c=${p#crates/}; c=${c%%/*}
      rest=${p#crates/$c/}
      case "$rest" in
        *.md) doc=yes ;;
      esac
      case "$rest" in
        tests/*|benches/*|examples/*)
          add_term "package($c)"
          [ "$c" = arris ] && corpus=all
          continue ;;
      esac
      add_term "rdeps($c)"
      case "$c" in
        arris-math|arris-geom|arris-topo|arris-check|arris-io)
          need_corpus part; add_area boolean; add_area provenance; add_area split ;;
        arris-ops)
          case "$rest" in
            src/boolean/*) need_corpus part; add_area boolean; add_area provenance; add_area split ;;
            src/sweep/loft.rs|src/sweep/loft_exact.rs) need_corpus part; add_area loft ;;
            src/sweep/along.rs) need_corpus part; add_area sweep ;;
            # The rest of sweep/ (extrude, revolve, exact, refuse, mod) feeds
            # nearly every recipe: extrude is in almost all, so the whole corpus.
            src/sweep/*) corpus=all ;;
            # blend/ is reached through `fillet` and `chamfer` alone: the
            # `blend` area and the rows of every fixture that calls one
            # (ADR-0085), unless the guard below finds another caller.
            src/blend/*) need_corpus part; add_area blend; blend=yes ;;
            src/offset/*) need_corpus part; add_area offset; add_area shell ;;
            src/shell/*) need_corpus part; add_area shell ;;
            src/transform.rs|src/mirror.rs) need_corpus part; add_area transform ;;
            *) corpus=all ;;
          esac ;;
        arris-py) ;;
        *) corpus=all ;;
      esac ;;
    crates/*) whole=yes ;;
    tests/fixtures/*/*)
      a=${p#tests/fixtures/}; a=${a%%/*}
      case "$a" in
        blend|boolean|build|loft|offset|primitive|provenance|query|shell|split|sweep|transform|real)
          add_term "package(arris)"; need_corpus part; add_area "$a"
          # The step-count record names every fixture of these areas; its
          # name check is cheap, its counts the full gate's.
          case "$a" in
            boolean|sweep|blend|build|provenance|transform|primitive|split)
              add_term "binary_id(arris-ops::cancel)" ;;
          esac
          # A fixture's STEP must read back to its measures. The areas' own
          # read-backs are in the slow set (blend's takes 14 min), so the
          # staged fixtures alone, by name.
          case "$a" in
            primitive|build|transform|boolean|loft|sweep|provenance|blend|offset|shell|split)
              add_term "(binary_id(arris-io::step_read) & test(=the_staged_fixtures_read_back))"
              n=${p#tests/fixtures/$a/}
              case "$n" in
                */*) n=${n%%/*}
                  for x in ${read_back[@]+"${read_back[@]}"} ""; do [ "$x" = "$n" ] && break; done
                  [ "$x" = "$n" ] || read_back+=("$n") ;;
              esac ;;
          esac ;;
        regression)
          # Placed for ADR-0085 §1: its rows are ignored, so the lint and
          # the readers below stand for it.
          add_term "binary_id(arris::corpus_lint)"
          add_term "(binary_id(arris::corpus) & test(=every_fixture_directory_has_a_corpus_test))"
          s=${p#tests/fixtures/regression/}; s=${s%%/*}
          for x in ${slugs[@]+"${slugs[@]}"} ""; do [ "$x" = "$s" ] && break; done
          [ "$x" = "$s" ] || slugs+=("$s") ;;
        *) whole=yes ;;
      esac ;;
    *.md|.agents/*|docs/*|.claude/*) prose=yes ;;
    # Placed for ADR-0085 §1: no nextest, the shell test each names.
    .github/*) add_tool_test tools/workflows-test.sh ;;
    .config/nextest.toml) add_tool_test tools/profile-test.sh ;;
    tools/oracle/*) oracle=yes; add_tool_test tools/oracle-test.sh ;;
    tools/*/*) whole=yes ;;
    tools/*.sh)
      t=${p#tools/}; t=${t%.sh}; t=${t%-test}
      if [ -f "tools/$t-test.sh" ]; then add_tool_test "tools/$t-test.sh"; fi ;;
    *) whole=yes ;;
  esac
done <<<"$paths"

# blend/ reaches the corpus through `fillet` and `chamfer` alone (ADR-0085).
# The rows of the fixtures that call one, outside the `blend` area (its prefix
# is a term of its own), `real` (out of `fast`) and `regression` (ignored),
# are named by test. The guard: another way into blend/, or a `lib.rs`
# re-export that is not exactly the two ops, and the corpus is whole again.
names=()
if [ "$blend" = yes ] && [ "$corpus" != all ]; then
  if grep -rnE 'blend::|crate::(fillet|chamfer)' crates/arris-ops/src --include='*.rs' |
    grep -vE '^crates/arris-ops/src/blend/' |
    grep -vE '^crates/arris-ops/src/lib\.rs:[0-9]+:pub use blend::\{chamfer, fillet\};$' |
    grep -q .; then
    corpus=all
  else
    dirs=$(grep -lE '"op"[[:space:]]*:[[:space:]]*"(fillet|chamfer)"' -r tests/fixtures --include=fixture.json 2>/dev/null |
      grep -vE '^tests/fixtures/(blend|real|regression)/' | sed 's#^tests/fixtures/##; s#/fixture\.json$##' || true)
    if [ -n "$dirs" ]; then
      # A row's name, by its "area/dir" string (several rows can share one).
      while IFS= read -r n; do
        [ -n "$n" ] && names+=("$n")
      done < <(grep -oE '^[[:space:]]*[a-z0-9_]+ => (run|part|query|variant) "[^"]+"' crates/arris/tests/corpus.rs |
        sed -E 's/^[[:space:]]*([a-z0-9_]+) => [a-z]+ "([^"]+)"/\2 \1/' |
        awk 'NR==FNR { want[$0]=1; next } ($1 in want) { print $2 }' <(printf '%s\n' "$dirs") - | sort -u)
    fi
  fi
fi

# The readers of a regression fixture, by name: each file under `crates/` that
# names `regression/<slug>`. A test file gives its binary, anything else its
# package. `corpus.rs` and `corpus_lint.rs` name every slug: the rows are
# ignored and the lint is already a term.
for s in ${slugs[@]+"${slugs[@]}"}; do
  while IFS= read -r f; do
    [ -n "$f" ] || continue
    c=${f#crates/}; c=${c%%/*}
    case "$f" in
      crates/arris/tests/corpus.rs|crates/arris/tests/corpus_lint.rs) ;;
      crates/*/tests/*.rs)
        b=${f#crates/$c/tests/}
        case "$b" in
          */*) add_term "package($c)" ;;
          *) add_term "binary_id($c::${b%.rs})" ;;
        esac ;;
      *) add_term "package($c)" ;;
    esac
  done < <(git grep -lE "regression/$s([^a-z0-9-]|\$)" -- crates 2>/dev/null || true)
done

# The oracle's callers: `arris` (the corpus) and `arris-debug` whole, and the
# test binaries elsewhere with a non-comment line that calls it (ADR-0085 §1).
if [ "$oracle" = yes ]; then
  add_term "package(arris)"
  add_term "package(arris-debug)"
  while IFS= read -r f; do
    [ -n "$f" ] || continue
    c=${f#crates/}; c=${c%%/*}
    case "$c" in arris|arris-debug) continue ;; esac
    b=${f#crates/$c/tests/}
    case "$b" in
      */*) add_term "package($c)" ;;
      *) add_term "binary_id($c::${b%.rs})" ;;
    esac
  done < <(git grep -nE 'oracle::|compare_stl|corpus::run|part::run|query::run' -- 'crates/*/tests/*.rs' |
    grep -vE '^[^:]+:[0-9]+:[[:space:]]*//' | cut -d: -f1 | sort -u)
fi

# A doc comment added or removed in a changed Rust file (staged, or the range).
if [ "$doc" = no ] && git diff "${diffargs[@]}" -U0 -- 'crates/*.rs' 2>/dev/null |
  grep -qE '^[+-][[:space:]]*//[/!]'; then
  doc=yes
fi

# `join SEP item…` — the items, separated by SEP (any length).
join() {
  local sep=$1 out="" first=1 x
  shift
  for x in "$@"; do
    if [ $first = 1 ]; then out=$x; first=0; else out="$out$sep$x"; fi
  done
  printf '%s' "$out"
}

if [ "${ARRIS_GATE:-}" = full ]; then
  filter='all()'
  cases=256
  full=yes
elif [ "$whole" = yes ]; then
  filter='all()'
  cases=16
  full=no
elif [ ${#terms[@]} -eq 0 ] && [ "$prose" = no ]; then
  filter='none()'
  cases=16
  full=no
else
  cases=16
  full=no
  parts=()
  if [ ${#terms[@]} -gt 0 ]; then
    u=$(join ' | ' "${terms[@]}")
    [ ${#terms[@]} -gt 1 ] && u="($u)"
    case "$corpus" in
      all) parts+=("$u | $CORPUS") ;;
      part)
        pre=$(join '|' "${areas[@]}")
        by_name=""
        if [ ${#names[@]} -gt 0 ]; then
          by_name=" | ($CORPUS & test(/^($(join '|' "${names[@]}"))\$/))"
        fi
        parts+=("($u & not $CORPUS) | ($CORPUS & test(/^(${pre}|${BASE})_/))$by_name") ;;
      none) parts+=("$u") ;;
    esac
  fi
  # Prose beside code: the docs tests, which the old gate ran with the rest.
  [ "$prose" = yes ] && parts+=("binary_id(arris::docs_refs) | binary_id(arris-check::doc_drift)")
  if [ ${#parts[@]} -eq 1 ]; then
    filter=${parts[0]}
  else
    wrapped=()
    for x in "${parts[@]}"; do wrapped+=("($x)"); done
    filter=$(join ' | ' "${wrapped[@]}")
  fi
fi

echo "filter=$filter"
echo "cases=$cases"
echo "diff_cases=$cases"
echo "doc=$doc"
echo "full=$full"
echo "read_back=$(join , ${read_back[@]+"${read_back[@]}"})"
echo "tool_tests=${tool_tests[*]+"${tool_tests[*]}"}"
