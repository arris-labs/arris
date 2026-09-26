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
#
# CI runs it (`semver` job); `/release` step 2 runs it as the cross-check
# on the number it derives.
set -euo pipefail

cd "$(dirname "$0")/.."

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

# Whether `## Unreleased` lists anything under `### Breaking`.
announced() {
    [ -f CHANGELOG.md ] || return 1
    awk '
        /^## /  { unreleased = ($0 ~ /^## Unreleased/); breaking = 0; next }
        /^### / { breaking = unreleased && ($0 ~ /^### Breaking/); next }
        breaking && /^- / { found = 1 }
        END { exit !found }
    ' CHANGELOG.md
}

log="$(mktemp)"
trap 'rm -f "$log"' EXIT

echo "semver-gate: the workspace against $base, as a patch release"
if cargo semver-checks --workspace --baseline-rev "$base" --release-type patch 2>&1 | tee "$log"; then
    echo "semver-gate: no breaking change since $base"
    exit 0
fi

# A failure that is not a semver verdict (a build, a missing rev) is a
# failure of the gate itself, never excused by the changelog.
if ! grep -q 'Summary semver requires new' "$log"; then
    echo "::error::semver-gate: cargo semver-checks failed without a verdict" >&2
    exit 1
fi

# A "minor" verdict (a deprecation, say) is no break: pre-1.0 it rides in
# a patch release (.agents/rules/git.md §The version). Only a break —
# which cargo-semver-checks calls "major" at any version — must be named.
if ! grep -q 'Summary semver requires new major' "$log"; then
    echo "semver-gate: additions since $base that break nothing; a patch may carry them"
    exit 0
fi

if announced; then
    echo "semver-gate: breaking changes since $base, announced under ## Unreleased / ### Breaking"
    exit 0
fi

echo "::error::semver-gate: breaking changes since $base, and CHANGELOG.md's ## Unreleased has no ### Breaking bullet — name the type or signature and the one-line fix there (.agents/rules/git.md §The changelog)" >&2
exit 1
