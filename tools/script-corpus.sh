#!/usr/bin/env bash
# Runs the script-corpus refusal histogram (ADR-0067).
#
# Reads every crates/arris-py/corpus/*/outcome.json, writes
# target/script-corpus/histogram.md, and prints it.
set -euo pipefail

cd "$(dirname "$0")/.."
out_dir="target/script-corpus"
mkdir -p "$out_dir"

cargo run --release -p arris-debug --example script_corpus -- "$out_dir"
