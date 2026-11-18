#!/usr/bin/env bash
# Check a contributor's commits and run the gate they reach (ADR-0091).
# Run from the checked-out PR head with all edits committed.
#
#   tools/pr-check.sh [--base <rev>]
#   tools/pr-check.sh --ci --base <merge-base>
#
# Local runs fetch canonical main unless a revision is supplied. CI gives
# the PR's merge base, skips the local setup checks, and runs clippy separately.
set -euo pipefail

usage() {
  echo "usage: tools/pr-check.sh [--ci] [--base <rev>]" >&2
  exit 2
}

ci=false
base_ref=
while [ $# -gt 0 ]; do
  case "$1" in
    --ci) ci=true; shift ;;
    --base)
      [ $# -ge 2 ] && [ -n "$2" ] && [ -z "$base_ref" ] || usage
      base_ref=$2
      shift 2 ;;
    *) usage ;;
  esac
done
if "$ci" && [ -z "$base_ref" ]; then usage; fi

cd "$(git rev-parse --show-toplevel)"
failed=0
finding() {
  printf 'pr-check: %s\n' "$*" >&2
  failed=1
}
run() {
  local label=$1
  shift
  printf 'pr-check: %s\n' "$label"
  "$@" || finding "$label failed"
}

if [ -z "$base_ref" ]; then
  git fetch --no-tags https://github.com/arris-labs/arris.git refs/heads/main
  base_ref=FETCH_HEAD
fi
base=$(git rev-parse --verify --quiet --end-of-options "$base_ref^{commit}") || {
  printf 'pr-check: invalid base revision %q\n' "$base_ref" >&2
  exit 2
}
if ! "$ci"; then
  if [ "$(git config --get core.hooksPath || true)" != .githooks ]; then
    finding 'core.hooksPath must be .githooks (git config core.hooksPath .githooks)'
  fi
  if ! git merge-base --is-ancestor "$base" HEAD; then
    finding "branch does not contain base ${base:0:12}; rebase onto it"
    # Check the branch's own changes even when upstream has advanced.
    base=$(git merge-base "$base" HEAD)
  fi
fi

subject_pattern='^(feat|fix|refactor|perf|test|docs|chore|build|ci|fixtures)\([a-z0-9-]+(,[a-z0-9-]+)*\): [a-z]'
email_pattern='^[^@[:space:]]+@[^@[:space:].]+(\.[^@[:space:].]+)+$'
trailer_pattern='^[[:blank:]]*(co-authored-by:|claude-session:|(🤖[[:blank:]]*)?generated with([[:blank:]:]|$))'
commits=$(git rev-list "$base..HEAD")
while IFS= read -r commit; do
  [ -n "$commit" ] || continue
  short=${commit:0:12}
  metadata=$(git show -s --format='%P%n%ae%n%s' "$commit")
  {
    IFS= read -r parents
    IFS= read -r email
    IFS= read -r subject
  } <<<"$metadata"
  if [[ "$parents" == *' '* ]]; then
    finding "$short is a merge commit"
  fi
  if [[ ! "$subject" =~ $subject_pattern || "$subject" == *. ]]; then
    finding "$short subject must be type(scope): lower-case summary without a period: $(printf '%q' "$subject")"
  fi
  if [[ ! "$email" =~ $email_pattern ]]; then
    finding "$short author email needs a dotted domain: $(printf '%q' "$email")"
  fi
  body=$(git show -s --format=%b "$commit")
  trailers=$(grep -Ei "$trailer_pattern" <<<"$body" || true)
  while IFS= read -r trailer; do
    [ -n "$trailer" ] || continue
    finding "$short disallowed trailer: $trailer"
  done <<<"$trailers"
done <<<"$commits"

while IFS= read -r -d '' status && IFS= read -r -d '' path; do
  case "$path" in
    CHANGELOG.md|docs/adr/*)
      finding "$(printf '%q' "$path") is edited by the maintainer at landing" ;;
  esac
  if [ "$status" = A ]; then
    case "${path,,}" in
      *.step|*.stp)
        case "$path" in
          tests/fixtures/*) ;;
          *) finding "place added STEP data under tests/fixtures/: $(printf '%q' "$path")" ;;
        esac ;;
    esac
  fi
done < <(git diff --no-renames --name-status -z "$base" HEAD)

# The test filter comes from the committed diff, so the build must see HEAD.
if ! git diff --quiet HEAD --; then
  finding 'commit tracked edits before checking the PR head'
  exit 1
fi
run 'layer rule' tools/check-layers.sh
run 'tolerance literals' tools/check-tolerance-literals.sh --diff "$base"
run 'cargo fmt' cargo fmt --all -- --check
if ! "$ci"; then
  run 'cargo clippy' cargo clippy --workspace --all-targets -- -D warnings
fi
run 'reached tests' tools/test-half.sh "$base" HEAD

[ "$failed" = 0 ] || exit 1
echo "pr-check: ok against ${base:0:12}"
