#!/usr/bin/env bash
# Detaches the verification's tree (ADR-0078) at <sha> and prints its path.
# The caller holds the verification's lock (`arris-verify/lock`): the tree is
# one linked worktree, `arris-verify/tree` in the common git dir, moved from
# sha to sha so cargo rebuilds only what changed. Its build output is
# `<checkout>/target/verify`, never the checkout's `target/`: the tree's path
# is baked into test binaries (`env!("CARGO_MANIFEST_DIR")`) and cargo reuses
# a binary whose inputs it thinks unchanged.
#
#   tools/verify-tree.sh <sha>
#
# Used by `tools/verify.sh` and `tools/at-commit.sh`.
set -euo pipefail
[ $# -eq 1 ] || { echo "usage: tools/verify-tree.sh <sha>" >&2; exit 2; }
tree=$(realpath "$(git rev-parse --git-common-dir)")/arris-verify/tree
if [ ! -e "$tree/.git" ]; then
  git worktree prune
  git worktree add --detach --force "$tree" "$1" >/dev/null
else
  git -C "$tree" checkout --quiet --force --detach "$1"
fi
echo "$tree"
