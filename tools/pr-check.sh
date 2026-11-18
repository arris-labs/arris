#!/usr/bin/env bash
# Check a contributor's commits and run the gate they reach (ADR-0091).
# Run from the checked-out PR head with all edits committed.
#
#   tools/pr-check.sh [--base <rev>]
#   tools/pr-check.sh --ci --base <merge-base>
#
# Local runs fetch canonical main unless a revision is supplied and verify
# a passing tip for the push hook. CI supplies the PR's merge base and runs
# clippy separately.
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
head=$(git rev-parse --verify HEAD)
if ! "$ci"; then
  if [ "$(git config --get core.hooksPath || true)" != .githooks ]; then
    finding 'core.hooksPath must be .githooks (git config core.hooksPath .githooks)'
  fi
  if ! git merge-base --is-ancestor "$base" "$head"; then
    finding "branch does not contain base ${base:0:12}; rebase onto it"
    # Check the branch's own changes even when upstream has advanced.
    base=$(git merge-base "$base" "$head")
  fi
fi

subject_pattern='^(feat|fix|refactor|perf|test|docs|chore|build|ci|fixtures)\([a-z0-9-]+(,[a-z0-9-]+)*\): [a-z]'
email_pattern='^[^@[:space:]]+@[^@[:space:].]+(\.[^@[:space:].]+)+$'
# Generated with footers do not always have a Git trailer separator.
trailer_pattern='^[[:blank:]]*(co-authored-by:|claude-session:|generated with:|🤖[[:blank:]]*generated with([[:blank:]:]|$)|generated with[[:blank:]]+[^,!?]*[^.,!?[:space:]][[:blank:]]*$)'
commits=$(git rev-list "$base..$head")
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
  footer=$(git show -s --format=%b "$commit" | awk 'BEGIN { RS = "" } { last = $0 } END { print last }')
  trailers=$(grep -Ei "$trailer_pattern" <<<"$footer" || true)
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
done < <(git diff --no-renames --name-status -z "$base" "$head")

# The test filter comes from the committed diff, so the build must see HEAD.
if ! git diff --quiet "$head" --; then
  finding 'commit tracked edits before checking the PR head'
  exit 1
fi
run 'layer rule' tools/check-layers.sh
run 'tolerance literals' tools/check-tolerance-literals.sh --diff "$base"
run 'cargo fmt' cargo fmt --all -- --check
if ! "$ci"; then
  run 'cargo clippy' cargo clippy --workspace --all-targets -- -D warnings
fi
run 'reached tests' tools/test-half.sh "$base" "$head"

[ "$failed" = 0 ] || exit 1
if ! "$ci"; then
  # A prose commit in a fresh clone inherits `none`. Use the ordinary
  # verifier's own base and committed worktree to establish a local pass.
  if [ "${ARRIS_VERIFY_LOCKED:-}" = 1 ] && tools/verify.sh --in-flight "$head"; then
    finding 'tip verification is waiting on this worktree; run pr-check from the topic branch checkout'
    exit 1
  fi
  if [ "$(tools/verify.sh --status "$head")" != pass ]; then
    if [ "${ARRIS_VERIFY_LOCKED:-}" = 1 ]; then
      finding 'tip verification needs the worktree lock; run pr-check from the topic branch checkout'
      exit 1
    fi
    echo "pr-check: verifying ${head:0:12} for the push hook"
    if ! tools/verify.sh "$head" || [ "$(tools/verify.sh --status "$head")" != pass ]; then
      finding "tip verification did not pass; see $(git rev-parse --git-common-dir)/arris-verify/$head.log"
    fi
  fi
fi

[ "$failed" = 0 ] || exit 1
if [ "$(git rev-parse --verify HEAD)" != "$head" ]; then
  finding 'HEAD changed during the check; rerun on the current tip'
  exit 1
fi
echo "pr-check: ok against ${base:0:12}"
