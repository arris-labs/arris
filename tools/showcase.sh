#!/usr/bin/env bash
# Regenerates docs/assets/showcase.png from crates/arris-py/examples/bracket.py
# and the render_labelled example.
#
#   tools/showcase.sh
#
# Deterministic: rerun leaves docs/assets/showcase.png unchanged.
set -euo pipefail

cd "$(dirname "$0")/.."

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

if ! compgen -G "crates/arris-py/python/arris/_arris*.so" > /dev/null; then
  cargo build -p arris-py --features extension-module
  cp target/debug/libarris_py.so crates/arris-py/python/arris/_arris.so
fi

PYTHONPATH=crates/arris-py/python python3 crates/arris-py/examples/bracket.py "$tmp"
cargo run --quiet -p arris-debug --example render_labelled -- \
  "$tmp/bracket.mesh" "$tmp/bracket.labels.json" docs/assets/showcase.png iso
