#!/usr/bin/env bash
# The test half of the gate (ADR-0078): the `fast` nextest profile over the
# crates and corpus areas a range reaches, the doctests, and `cargo doc` when
# a doc comment changed. Run in the tree to test; `tools/verify.sh` runs it on
# the verification's tree and the pre-commit hook inline over a red's range.
#
#   tools/test-half.sh <base> [<sha>]   (no <sha>: the staged index, the hook's
#                                        inline run)
set -euo pipefail
{ [ $# -eq 1 ] || [ $# -eq 2 ]; } || { echo "usage: tools/test-half.sh <base> [<sha>]" >&2; exit 2; }
cd "$(git rev-parse --show-toplevel)"
gate=$(tools/gate.sh since "$@")
gate_value() { sed -n "s/^$1=//p" <<<"$gate"; }
for t in $(gate_value tool_tests); do
  echo "test-half: $t"
  "$t"
done
# A range that reaches no crate and no prose (tools alone) builds nothing.
[ "$(gate_value filter)" != 'none()' ] || exit 0
echo "test-half: cargo nextest --profile fast ($(gate_value cases) cases)"
ARRIS_PROPTEST_CASES=$(gate_value cases) ARRIS_DIFF_CASES=$(gate_value diff_cases) \
  ARRIS_READ_BACK=$(gate_value read_back) cargo nextest run --workspace --profile fast \
  -E "$(gate_value filter)" --cargo-quiet --status-level fail --no-tests pass
echo "test-half: cargo test --doc"
cargo test --workspace --doc --quiet
if [ "$(gate_value doc)" = yes ]; then
  echo "test-half: cargo doc"
  RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps --quiet
fi
