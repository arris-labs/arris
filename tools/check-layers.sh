#!/usr/bin/env bash
# The layer rule (docs/ARCHITECTURE.md §Crates and the layer rule): a
# crate depends only on crates in a strictly lower layer. Walks `cargo
# metadata --no-deps` — declared edges, no resolution, no network — and fails
# on the first upper→lower or sibling edge. Dev-dependencies are exempt:
# a lower crate's tests may use `arris-debug`.
#
#   tools/check-layers.sh              check this workspace
#   tools/check-layers.sh --self-test  copy the workspace to a scratch dir,
#                                      add a forbidden edge, expect failure
#
# A crate not in the table below is an error: a new crate is placed in a
# layer in architecture first, then here.
set -euo pipefail

layer() {
  case "$1" in
    arris-math) echo 0 ;;
    arris-geom) echo 1 ;;
    arris-topo) echo 2 ;;
    arris-check) echo 3 ;;
    arris-ops | arris-mesh) echo 4 ;; # siblings: neither depends on the other
    arris-io) echo 5 ;; # depends on arris-mesh for the mesh formats (ADR-0013)
    arris-debug) echo 6 ;;
    arris) echo 7 ;;
    arris-py) echo 8 ;; # the Python binding: depends on the facade alone (ADR-0034)
    *)
      echo "check-layers: '$1' is not in the layer table; add it to docs/ARCHITECTURE.md and to $0" >&2
      exit 2
      ;;
  esac
}

root="${ARRIS_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

scratch=""
cleanup() { if [ -n "$scratch" ]; then rm -rf "$scratch"; fi; }
trap cleanup EXIT

self_test() {
  scratch="$(mktemp -d)"
  cp -r "$root/Cargo.toml" "$root/crates" "$root/tools" "$scratch/"
  # arris-math → arris-check is the canonical forbidden edge.
  printf '\n[dependencies.arris-check]\npath = "../arris-check"\n' >>"$scratch/crates/arris-math/Cargo.toml"
  if ARRIS_ROOT="$scratch" "$scratch/tools/check-layers.sh" >/dev/null 2>&1; then
    echo "check-layers self-test: FAILED — the forbidden edge arris-math → arris-check was not detected" >&2
    exit 1
  fi
  echo "check-layers self-test: ok (forbidden edge detected)"
}

if [ "${1:-}" = "--self-test" ]; then
  self_test
  exit 0
fi

cd "$root"
# One line per declared edge: "<crate> <dependency> <kind>", kind "normal"
# for the null kind cargo prints for ordinary dependencies.
edges="$(cargo metadata --format-version 1 --no-deps \
  | jq -r '.packages[] | .name as $n | .dependencies[] | "\($n) \(.name) \(.kind // "normal")"')"

members="$(cargo metadata --format-version 1 --no-deps | jq -r '.packages[].name' | sort)"
is_member() { grep -qx "$1" <<<"$members"; }

checked=0
bad=0
while read -r from to kind; do
  [ -n "$from" ] || continue
  [ "$kind" = "dev" ] && continue
  is_member "$to" || continue
  checked=$((checked + 1))
  lf="$(layer "$from")"
  lt="$(layer "$to")"
  if [ "$lt" -ge "$lf" ]; then
    echo "check-layers: forbidden edge $from (layer $lf) → $to (layer $lt)" >&2
    bad=$((bad + 1))
  fi
done <<<"$edges"

if [ "$bad" -ne 0 ]; then
  echo "check-layers: $bad forbidden edge(s); see docs/ARCHITECTURE.md §Crates" >&2
  exit 1
fi
echo "check-layers: ok ($checked workspace edges checked)"
