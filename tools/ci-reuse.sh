#!/usr/bin/env bash
# The CI reuse gate (plans/ci-reuse-gate): whether a push needs the suite
# again. The suite costs hours; a commit that changes only version strings,
# the changelog and prose no test reads cannot change its verdict, so the
# nearest commit that already went green on `main` stands in for it.
#
#   tools/ci-reuse.sh <head>     key=value lines: skip, base, reason
#   tools/ci-reuse.sh --self-test   a scratch repository per case
#
# The base is `<head>` itself or the nearest of its first 20 first-parent
# ancestors with a successful `ci.yml` push run (`gh api`, `actions: read`).
# `skip=true` only when `git diff <base> <head>` is inside the allow-list:
#   - CHANGELOG.md, crates/*/CHANGELOG.md, AGENTS.md, SEED.md, .agents/**
#   - docs/**, bar ROADMAP.md, DATA-MODEL.md and plans/** (tests read those);
#     the same set `ci.yml`'s path filter already ignores
#   - Cargo.lock, when every changed line is a `version = "…"` line
#   - the root Cargo.toml, when it equals the base's once the workspace
#     package's version and the internal crates' exact pins are normalised
# Anything else, no green ancestor and any failed lookup are `skip=false`:
# the default is the full run, so a mistake costs hours and never a missed
# test. A pull request is never asked (`ci.yml`).
#
# The green lookup is one function; the self-test replaces it with
# ARRIS_CI_REUSE_GREEN (the green shas, space-separated) and
# ARRIS_CI_REUSE_LOOKUP_FAIL (any value: the lookup errors).
set -uo pipefail

depth=20

# 0: a successful `ci.yml` push run exists for the commit; 1: none; 2: the
# lookup itself failed.
green_run() {
  [ -z "${ARRIS_CI_REUSE_LOOKUP_FAIL:-}" ] || return 2
  if [ "${ARRIS_CI_REUSE_GREEN+set}" = set ]; then
    case " $ARRIS_CI_REUSE_GREEN " in *" $1 "*) return 0 ;; esac
    return 1
  fi
  [ -n "${GITHUB_REPOSITORY:-}" ] || return 2
  local n
  n="$(gh api "repos/$GITHUB_REPOSITORY/actions/workflows/ci.yml/runs?head_sha=$1&event=push&status=success&per_page=1" --jq .total_count 2>/dev/null)" || return 2
  case "$n" in '' | *[!0-9]*) return 2 ;; esac
  [ "$n" -gt 0 ]
}

# The root manifest with its version strings made equal: the workspace
# package's own and the internal crates' exact pins, each only in its own
# table. A dependency's requirement, in any other table or form, is not
# touched, so changing one stays a difference.
norm_manifest() {
  awk '
    /^\[/ { sec = $0 }
    sec == "[workspace.package]" && /^version = "/ { $0 = "version = \"V\"" }
    sec == "[workspace.dependencies]" && /^arris[a-z-]* *= *\{/ { gsub(/version = "=[^"]*"/, "version = \"V\"") }
    { print }'
}

manifest_ok() {
  local a b
  a="$(git show "$1:Cargo.toml" 2>/dev/null | norm_manifest; echo "${PIPESTATUS[0]}")" || return 1
  b="$(git show "$2:Cargo.toml" 2>/dev/null | norm_manifest; echo "${PIPESTATUS[0]}")" || return 1
  # The last line is `git show`'s status: a missing file is no equality.
  [ "${a##*$'\n'}" = 0 ] && [ "${b##*$'\n'}" = 0 ] && [ "$a" = "$b" ]
}

# Every changed line of Cargo.lock is a `version = "…"` line. A changed
# dependency would move its checksum too, which is not.
lock_ok() {
  local diff
  # Options, so a user's diff config (diff.interHunkContext merges hunks and
  # prints context) cannot change the answer. The headers before the first
  # `@@` are dropped by position, never by what a changed line starts with.
  diff="$(git diff -U0 --inter-hunk-context=0 --no-color --no-ext-diff --no-renames "$1" "$2" -- Cargo.lock)" || return 1
  ! printf '%s\n' "$diff" |
    awk '/^@@ /{ body = 1; next } body' |
    grep -Evq '^[-+]version = "[^"]*"$'
}

prose_ok() {
  case "$1" in
    docs/ROADMAP.md | docs/DATA-MODEL.md | docs/plans/*) return 1 ;;
    docs/* | .agents/* | AGENTS.md | SEED.md | CHANGELOG.md) return 0 ;;
    crates/*/*/*) return 1 ;;
    crates/*/CHANGELOG.md) return 0 ;;
  esac
  return 1
}

# `skip` last: a consumer that keeps the last value of a key (GITHUB_OUTPUT)
# cannot have it overridden by a line inside `reason`, which is one line.
emit() {
  local reason="${3//[$'\n\r']/ }"
  echo "reason=$reason"
  echo "base=$2"
  echo "skip=$1"
}

verdict() {
  local head sha base="" commits rc files p
  head="$(git rev-parse --verify -q "$1^{commit}")" || { emit false "" "unknown revision $1"; return 0; }
  commits="$(git rev-list --first-parent -n $((depth + 1)) "$head")" || { emit false "" "history unreadable"; return 0; }
  for sha in $commits; do
    green_run "$sha"
    rc=$?
    case "$rc" in
      0) base="$sha"; break ;;
      1) ;;
      *) emit false "" "the green lookup failed at ${sha:0:12}"; return 0 ;;
    esac
  done
  [ -n "$base" ] || { emit false "" "no green ancestor within $depth commits"; return 0; }
  files="$(git diff --name-only --no-renames "$base" "$head")" || { emit false "$base" "diff unreadable"; return 0; }
  while IFS= read -r p; do
    [ -n "$p" ] || continue
    case "$p" in
      Cargo.lock) lock_ok "$base" "$head" || { emit false "$base" "Cargo.lock changes more than versions"; return 0; } ;;
      Cargo.toml) manifest_ok "$base" "$head" || { emit false "$base" "Cargo.toml changes more than versions"; return 0; } ;;
      *) prose_ok "$p" || { emit false "$base" "$p is outside the allow-list"; return 0; } ;;
    esac
  done <<< "$files"
  emit true "$base" "only versions and prose differ from ${base:0:12}"
}

# -- self-test ---------------------------------------------------------------

mk_repo() {
  mkdir -p "$1"
  cd "$1" || exit 1
  git init -q
  git config user.name t
  git config user.email t@t
  git config commit.gpgsign false
  git config diff.interHunkContext 10
  # The directory name is built, not written: docs_refs reads `plans/<slug>`
  # in this file as a citation of a plan.
  pd=plans
  mkdir -p crates/x/src tests tools .github/workflows docs/$pd .agents
  cat > Cargo.toml <<'EOF'
[workspace.package]
version = "0.9.1"
edition = "2024"

[workspace.dependencies]
serde = { version = "1", default-features = false }
arris-math = { path = "crates/arris-math", version = "=0.9.1", default-features = false }

[workspace.dependencies.tab]
version = "1"
EOF
  cat > Cargo.lock <<'EOF'
[[package]]
name = "arris-math"
version = "0.9.1"

[[package]]
name = "arris-geom"
version = "0.9.1"

[[package]]
name = "serde"
version = "1.0.1"
checksum = "aaaa"
EOF
  echo "## Unreleased" > CHANGELOG.md
  echo state > AGENTS.md
  echo notes > docs/notes.md
  echo roadmap > docs/ROADMAP.md
  echo plan > docs/$pd/q.txt
  echo fn > crates/x/src/lib.rs
  echo t > tests/a.rs
  echo s > tools/a.sh
  echo w > .github/workflows/ci.yml
  git add -A
  git commit -qm base
}

# Runs one case: `mutate` is a shell snippet applied to a fresh repository
# whose root commit is green; the expected skip value follows.
case_run() {
  local name="$1" expect="$2" mutate="$3" green="${4:-base}" dir="$5" got base head
  (
    mk_repo "$dir/r"
    pd=plans
    base="$(git rev-parse HEAD)"
    eval "$mutate"
    git add -A
    git commit -qm head --allow-empty
    head="$(git rev-parse HEAD)"
    case "$green" in
      base) ARRIS_CI_REUSE_GREEN="$base" ;;
      head) ARRIS_CI_REUSE_GREEN="$head" ;;
      none) ARRIS_CI_REUSE_GREEN="" ;;
      fail) ARRIS_CI_REUSE_LOOKUP_FAIL=1 ;;
    esac
    export ARRIS_CI_REUSE_GREEN ARRIS_CI_REUSE_LOOKUP_FAIL
    verdict "$head" > "$dir/out"
  )
  got="$(sed -n 's/^skip=//p' "$dir/out")"
  if [ "$got" != "$expect" ]; then
    echo "ci-reuse --self-test: $name: expected skip=$expect, got skip=$got" >&2
    sed 's/^/  /' "$dir/out" >&2
    return 1
  fi
  return 0
}

self_test() {
  local fail=0
  # Each case runs in a subshell and its own scratch directory.
  run() {
    local d
    d="$(mktemp -d)"
    case_run "$1" "$2" "$3" "${4:-base}" "$d" || fail=1
    rm -rf "$d"
  }
  run "a version bump" true \
    'sed -i -E "0,/^version = .*/ s//version = \"0.10.0-dev\"/; s/version = \"=0.9.1\"/version = \"=0.10.0-dev\"/" Cargo.toml
     sed -i -E "s/^version = \"0.9.1\"/version = \"0.10.0-dev\"/" Cargo.lock'
  run "a changelog and AGENTS edit" true 'echo more >> CHANGELOG.md; echo more >> AGENTS.md'
  run "a docs note and an .agents edit" true 'echo more >> docs/notes.md; echo more >> .agents/r.md'
  run "no change at all" true ':'
  run "the head itself green" true 'echo more >> crates/x/src/lib.rs' head
  run "a source change" false 'echo more >> crates/x/src/lib.rs'
  run "a test change" false 'echo more >> tests/a.rs'
  run "a tools change" false 'echo more >> tools/a.sh'
  run "a workflow change" false 'echo more >> .github/workflows/ci.yml'
  run "a ROADMAP change" false 'echo more >> docs/ROADMAP.md'
  run "a plan change" false 'echo more >> docs/$pd/q.txt'
  run "a dependency requirement" false \
    'sed -i "s/serde = { version = \"1\"/serde = { version = \"2\"/" Cargo.toml'
  run "a version bump plus another manifest line" false \
    'sed -i -E "0,/^version = .*/ s//version = \"0.10.0-dev\"/" Cargo.toml; echo "x = 1" >> Cargo.toml'
  run "a table-form dependency requirement" false \
    'sed -i "/^\[workspace.dependencies.tab\]/,\$ s/version = \"1\"/version = \"2\"/" Cargo.toml'
  run "a source file moved under docs" false 'git mv crates/x/src/lib.rs docs/lib.rs'
  run "a docs note deleted" true 'git rm -q docs/notes.md'
  run "a crate changelog edit" true 'echo more >> crates/x/CHANGELOG.md'
  run "a changelog nested below a crate" false 'mkdir -p crates/x/y; echo more >> crates/x/y/CHANGELOG.md'
  run "a SEED edit" true 'echo more >> SEED.md'
  run "a crate manifest change" false 'echo "[package]" > crates/x/Cargo.toml'
  run "a lock checksum change" false \
    'sed -i -E "s/1.0.1/1.0.2/; s/aaaa/bbbb/" Cargo.lock'
  run "a lock package added" false 'printf "\n[[package]]\nname = \"y\"\nversion = \"1.0.0\"\n" >> Cargo.lock'
  run "a version bump riding with a source change" false \
    'sed -i -E "0,/^version = .*/ s//version = \"0.10.0-dev\"/" Cargo.toml; echo more >> crates/x/src/lib.rs'
  run "a source change in an earlier commit" false \
    'git checkout -q -b t; echo more >> crates/x/src/lib.rs; git commit -qam mid; echo more >> AGENTS.md'
  run "no green ancestor" false 'echo more >> AGENTS.md' none
  run "a failed lookup" false 'echo more >> AGENTS.md' fail
  run "a green base beyond the depth" false \
    'for i in $(seq 1 25); do echo $i >> AGENTS.md; git commit -qam c$i; done'
  if [ "$fail" != 0 ]; then
    return 1
  fi
  echo "ci-reuse --self-test: every case reads as expected"
}

if [ "${1:-}" = "--self-test" ]; then
  self_test
  exit $?
fi

if [ $# -ne 1 ]; then
  echo "usage: tools/ci-reuse.sh <head> | --self-test" >&2
  exit 2
fi

cd "$(dirname "$0")/.." || exit 2
verdict "$1"
