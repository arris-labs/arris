#!/usr/bin/env bash
# The semver gate (ADR-0027): a breaking change since the last release is
# allowed, pre-1.0, only when CHANGELOG.md announces it — a bullet under
# `### Breaking` in `## Unreleased`. cargo-semver-checks compares every
# published crate against the last `v*` tag as if the next release were a
# patch; the crates it flags must then be answered by that bullet, which is
# what makes `/release` pick a minor and tells a consumer what to fix.
# `arris-debug` is `publish = false` and cargo-semver-checks skips it.
#
#   tools/semver-gate.sh [BASELINE_REV]   default: the last v* tag
#   tools/semver-gate.sh --self-test      the verdict read, on coloured output
#
# CI runs it (`semver` job); `/release` step 2 runs it as the cross-check
# on the number it derives.
set -euo pipefail

cd "$(dirname "$0")/.."

# cargo-semver-checks styles its own output, and the workflow sets
# CARGO_TERM_COLOR=always. The styles land *inside* the line the verdict is
# read from — "<bold>Summary<reset> semver requires new major version" — so
# a `grep` for the sentence as written never matches and a real verdict
# reads as no verdict at all. Every read of the log goes through `plain`,
# which drops the escapes: the gate must decide on the API, never on how
# the tool painted its console (--self-test guards exactly this).
esc=$'\033'
plain() { sed "s/${esc}\[[0-9;]*[A-Za-z]//g"; }

self_test() {
    local out
    out="$(printf '\033[1m\033[31m     Summary\033[0m semver requires new major version: 1 major and 0 minor checks failed\n' | plain)"
    if ! printf '%s\n' "$out" | grep -q 'Summary semver requires new major'; then
        echo "semver-gate --self-test: a coloured verdict read as no verdict" >&2
        return 1
    fi
    echo "semver-gate --self-test: a coloured verdict reads"
}

if [ "${1:-}" = "--self-test" ]; then
    self_test
    exit $?
fi

# The last release before HEAD: on the tagged release commit itself (the
# tag's CI run) that is the tag before it, never HEAD against itself.
last_release() {
    local tag
    tag="$(git describe --tags --match 'v[0-9]*' --abbrev=0)"
    if [ "$(git rev-parse "$tag^{commit}")" = "$(git rev-parse HEAD)" ]; then
        tag="$(git describe --tags --match 'v[0-9]*' --abbrev=0 HEAD^)"
    fi
    echo "$tag"
}

base="${1:-$(last_release)}"

# Whether `## Unreleased` lists anything under `### Breaking`. On the tagged
# release commit itself `/release` has already turned `Unreleased` into the
# tag's `## X.Y.Z` section, so that section is read too: the same bullets,
# one commit later.
announced() {
    [ -f CHANGELOG.md ] || return 1
    local head_tag
    head_tag="$(git tag --points-at HEAD --list 'v[0-9]*' | head -n1)"
    awk -v rel="${head_tag#v}" '
        /^## /  { open = ($0 ~ /^## Unreleased/) || (rel != "" && index($0, "## " rel " ") == 1); breaking = 0; next }
        /^### / { breaking = open && ($0 ~ /^### Breaking/); next }
        breaking && /^- / { found = 1 }
        END { exit !found }
    ' CHANGELOG.md
}

raw="$(mktemp)"
verdict="$(mktemp)"
trap 'rm -f "$raw" "$verdict"' EXIT

echo "semver-gate: the workspace against $base, as a patch release"
if cargo semver-checks --workspace --baseline-rev "$base" --release-type patch 2>&1 | tee "$raw"; then
    echo "semver-gate: no breaking change since $base"
    exit 0
fi

# The console keeps the tool's own colouring; the verdict is read from the
# same text with the escapes dropped (see `plain`).
plain < "$raw" > "$verdict"

# A failure that is not a semver verdict (a build, a missing rev) is a
# failure of the gate itself, never excused by the changelog.
if ! grep -q 'Summary semver requires new' "$verdict"; then
    echo "::error::semver-gate: cargo semver-checks failed without a verdict" >&2
    exit 1
fi

# A "minor" verdict (a deprecation, say) is no break: pre-1.0 it rides in
# a patch release (.agents/rules/git.md §The version). Only a break —
# which cargo-semver-checks calls "major" at any version — must be named.
if ! grep -q 'Summary semver requires new major' "$verdict"; then
    echo "semver-gate: additions since $base that break nothing; a patch may carry them"
    exit 0
fi

if announced; then
    echo "semver-gate: breaking changes since $base, announced under ## Unreleased / ### Breaking"
    exit 0
fi

echo "::error::semver-gate: breaking changes since $base, and CHANGELOG.md's ## Unreleased has no ### Breaking bullet — name the type or signature and the one-line fix there (.agents/rules/git.md §The changelog)" >&2
exit 1
