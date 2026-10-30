#!/usr/bin/env bash
# Runs a command in its own transient systemd user scope with a memory
# bound, so memory pressure ends the gate (or oomd picks it) and not the
# terminal's scope that holds the session.
#
#   tools/gate-scope.sh <command> [args…]
#
# The hook and `tools/full-gate.sh` re-run themselves through this. Where
# `systemd-run --user` does not work, or `ARRIS_GATE_SCOPE=off`, or the
# command is already inside a scope (`ARRIS_GATE_IN_SCOPE`), it runs the
# command unchanged. A command ended by SIGKILL (exit 137) inside a scope
# prints the bound that most likely ended it and exits 137.
#
# The bound (⚠ plans/development-throughput OPEN 1, settled): MemoryHigh
# 40G throttles reclaim first, MemoryMax 48G kills, swap 0 so a runaway
# fails fast rather than thrashing; on this machine's 61 GiB that leaves
# the terminal and the other project's builds about 13 GiB. Override with
# ARRIS_GATE_MEMORY_HIGH / ARRIS_GATE_MEMORY_MAX / ARRIS_GATE_MEMORY_SWAP.
set -uo pipefail
[ $# -gt 0 ] || { echo "usage: tools/gate-scope.sh <command> [args…]" >&2; exit 2; }

if [ "${ARRIS_GATE_SCOPE:-on}" = off ] || [ -n "${ARRIS_GATE_IN_SCOPE:-}" ] ||
  ! command -v systemd-run >/dev/null ||
  ! systemd-run --user --scope --quiet true >/dev/null 2>&1; then
  exec "$@"
fi

high=${ARRIS_GATE_MEMORY_HIGH:-40G}
max=${ARRIS_GATE_MEMORY_MAX:-48G}
swap=${ARRIS_GATE_MEMORY_SWAP:-0}
systemd-run --user --scope --quiet --collect \
  -p MemoryHigh="$high" -p MemoryMax="$max" -p MemorySwapMax="$swap" \
  env ARRIS_GATE_IN_SCOPE=1 "$@"
rc=$?
if [ "$rc" = 137 ]; then
  echo "gate-scope: killed (SIGKILL), most likely by the gate scope's memory bound" >&2
  echo "gate-scope: MemoryMax=$max MemoryHigh=$high MemorySwapMax=$swap; raise ARRIS_GATE_MEMORY_MAX or set ARRIS_GATE_SCOPE=off" >&2
fi
exit "$rc"
