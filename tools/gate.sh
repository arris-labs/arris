#!/usr/bin/env bash
# The pre-commit gate a set of paths needs, printed as `key=value` lines
# (ADR-0032). The classifier and nothing else: it runs no test, and it is the
# only place the rule lives, so the hook, the skills and `tools/gate-test.sh`
# agree.
#
#   tools/gate.sh                  the staged paths
#   tools/gate.sh plan <path>…     these paths (`doc` still reads the staged diff)
#
# Keys:
#   filter      a nextest filterset for `--profile fast -E`
#   cases       ARRIS_PROPTEST_CASES
#   diff_cases  ARRIS_DIFF_CASES
#   doc         yes when `cargo doc` is worth its 30 s: the staged diff adds
#               or removes a `///` or `//!` line, or a `.md` under `crates/`
#               (they are `include_str!`-ed) changed
#   full        yes under `ARRIS_GATE=full`: the old gate, every test at 256
#
# Classification, from the path:
#   crates/<c>/src (and Cargo.toml)   <c> and everything that depends on it
#                                     (`rdeps`), and the corpus areas below
#   crates/<c>/{tests,benches,examples}   <c> alone
#   crates/arris-{math,geom,topo,check,io}, arris-ops/src/boolean
#                                     corpus `boolean_*` and `provenance_*`
#   arris-ops/src/{sweep,blend,transform,mirror}   that area's corpus tests
#   any other crate source (mesh, debug, the rest of ops)   the whole corpus
#   crates/arris/                     the whole `arris` package, corpus included
#   tests/fixtures/<area>/            that area's corpus tests
#   prose (`*.md`, docs/, .agents/)   the docs tests, nothing else
#   anything else                     every test of the `fast` profile
#
# A path the table cannot place costs the whole profile, never less; nothing
# here yields a gate smaller than the changed crate's own tests. The corpus
# (`arris::corpus`) sits in `arris`, which depends on every crate, so a
# crate's `rdeps` reaches all of it; a partial area list subtracts it and adds
# the named areas back. `real_*` and the slow set are the profile's business.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

if [ "${1:-}" = plan ]; then
  shift
  paths=$(printf '%s\n' "$@")
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
          need_corpus part; add_area boolean; add_area provenance ;;
        arris-ops)
          case "$rest" in
            src/boolean/*) need_corpus part; add_area boolean; add_area provenance ;;
            src/sweep.rs) need_corpus part; add_area sweep ;;
            src/blend.rs) need_corpus part; add_area blend ;;
            src/transform.rs|src/mirror.rs) need_corpus part; add_area transform ;;
            *) corpus=all ;;
          esac ;;
        *) corpus=all ;;
      esac ;;
    crates/*) whole=yes ;;
    tests/fixtures/*/*)
      a=${p#tests/fixtures/}; a=${a%%/*}
      case "$a" in
        blend|boolean|build|primitive|provenance|sweep|transform|real)
          add_term "package(arris)"; need_corpus part; add_area "$a" ;;
        *) whole=yes ;;
      esac ;;
    *.md|.agents/*|docs/*) prose=yes ;;
    *) whole=yes ;;
  esac
done <<<"$paths"

# A doc comment added or removed in a staged Rust file.
if [ "$doc" = no ] && git diff --cached -U0 -- 'crates/*.rs' 2>/dev/null |
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
  cases=32
  full=no
elif [ ${#terms[@]} -eq 0 ] && [ "$prose" = no ]; then
  filter='none()'
  cases=32
  full=no
else
  cases=32
  full=no
  parts=()
  if [ ${#terms[@]} -gt 0 ]; then
    u=$(join ' | ' "${terms[@]}")
    [ ${#terms[@]} -gt 1 ] && u="($u)"
    case "$corpus" in
      all) parts+=("$u | $CORPUS") ;;
      part)
        pre=$(join '|' "${areas[@]}")
        parts+=("($u & not $CORPUS) | ($CORPUS & test(/^(${pre}|${BASE})_/))") ;;
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
echo "diff_cases=32"
echo "doc=$doc"
echo "full=$full"
